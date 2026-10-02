"""Run the real launch and verify ROS transport, motion and recovery.

Run with the built workspace sourced. A dedicated ROS_DOMAIN_ID isolates this
test from other robots. JSON and launch logs are saved even on failure.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tabletop_interfaces.action import PickPlace
from tabletop_interfaces.msg import ObjectArray


class Probe(Node):
    def __init__(self):
        super().__init__('tabletop_validation')
        self.frames = {}
        self.objects = None
        self.object_count = 0
        self.statuses = []
        self.bridge = CvBridge()
        self.listeners = [
            self.create_subscription(Image, '/camera/rgb', lambda m: self.frames.update(rgb=m), 10),
            self.create_subscription(Image, '/camera/depth', lambda m: self.frames.update(depth=m), 10),
            self.create_subscription(CameraInfo, '/camera/info', lambda m: self.frames.update(info=m), 10),
            self.create_subscription(ObjectArray, '/perception/objects', self.observe, 10),
            self.create_subscription(String, '/task/status', lambda m: self.statuses.append(m.data), 10),
        ]
        self.command = self.create_publisher(String, '/task/command', 10)
        self.cancel = self.create_publisher(String, '/task/cancel', 10)
        self.reset = self.create_client(Trigger, '/scene/reset')
        self.arm = ActionClient(self, PickPlace, '/arm/pick_place')

    def observe(self, message):
        self.objects = message
        self.object_count += 1

    def until(self, condition, timeout=20):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if condition():
                return
        raise AssertionError(f'Timed out after {timeout}s; statuses={self.statuses[-10:]}')

    def future(self, future, timeout=45):
        self.until(future.done, timeout)
        return future.result()

    def fresh(self):
        count = self.object_count
        self.until(lambda: self.object_count > count)
        return self.objects

    def reset_scene(self):
        response = self.future(self.reset.call_async(Trigger.Request()))
        assert response.success, response.message
        # Discard queued observations of the previous scene.
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        self.fresh()
        assert {o.color for o in self.objects.objects} == {'red', 'green', 'blue'}

    def goal(self, color, destination):
        request = PickPlace.Goal()
        obj = next(o for o in self.objects.objects if o.color == color)
        request.pick = obj.position
        request.place = Point(x=0.45, y=0.25 if destination == 'left' else -0.25, z=0.025)
        return request

    def placed(self, color, destination):
        self.fresh()
        obj = next(o for o in self.objects.objects if o.color == color)
        target = (0.45, 0.25 if destination == 'left' else -0.25)
        assert math.dist((obj.position.x, obj.position.y), target) < 0.055, str(obj)

    def task_result(self, since):
        self.until(lambda: any(s.startswith('{') for s in self.statuses[since:]), 50)
        return json.loads(next(s for s in self.statuses[since:] if s.startswith('{')))


def run(probe, check):
    probe.until(lambda: probe.reset.service_is_ready() and probe.arm.server_is_ready()
                and probe.command.get_subscription_count() > 0 and probe.cancel.get_subscription_count() > 0
                and probe.objects is not None and len(probe.frames) == 3, 30)
    names = {name for name, ns in probe.get_node_names_and_namespaces()}
    assert {'tabletop_simulator', 'tabletop_perception', 'tabletop_tasks'} <= names, names
    check('all_three_nodes_launched', sorted(names))
    probe.until(lambda: len({(m.header.stamp.sec, m.header.stamp.nanosec)
                             for m in probe.frames.values()}) == 1)
    rgb, depth, info = (probe.frames[k] for k in ('rgb', 'depth', 'info'))
    assert rgb.encoding == 'rgb8' and depth.encoding == '32FC1'
    assert rgb.width == depth.width == info.width == 320
    assert rgb.height == depth.height == info.height == 320
    assert info.k[0] > 0 and info.k[0] == info.k[4]
    rgb_array = probe.bridge.imgmsg_to_cv2(rgb, 'rgb8')
    depth_array = probe.bridge.imgmsg_to_cv2(depth, '32FC1')
    assert rgb_array.shape == (320, 320, 3)
    assert np.isfinite(depth_array).all() and (depth_array > 0).all()
    assert {o.color for o in probe.objects.objects} == {'red', 'green', 'blue'}
    assert probe.objects.header.frame_id == 'world'
    check('rgb_depth_camera_info_and_objects', {'colors': [o.color for o in probe.objects.objects],
                                               'rgb_encoding': rgb.encoding, 'depth_encoding': depth.encoding})

    since = len(probe.statuses)
    probe.command.publish(String(data='red blue left'))
    probe.until(lambda: any(s.startswith('Rejected: Specify') for s in probe.statuses[since:]))
    check('invalid_command_rejected')

    since = len(probe.statuses)
    probe.command.publish(String(data='red left'))
    record = probe.task_result(since)
    assert record['success'], record
    probe.placed('red', 'left')
    check('task_command_full_pick_place', record)

    probe.reset_scene()
    check('scene_reset_and_new_observations')
    handle = probe.future(probe.arm.send_goal_async(probe.goal('blue', 'right')))
    assert handle.accepted
    result = probe.future(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success, str(result)
    probe.placed('blue', 'right')
    check('direct_action_full_pick_place', {'success': result.result.success, 'message': result.result.message})

    probe.reset_scene()
    since = len(probe.statuses)
    probe.command.publish(String(data='green right'))
    probe.until(lambda: 'lift' in probe.statuses[since:])
    probe.command.publish(String(data='blue left'))
    probe.until(lambda: 'Rejected: task already running' in probe.statuses[since:])
    check('busy_task_rejected')
    response = probe.future(probe.reset.call_async(Trigger.Request()))
    assert not response.success and response.message == 'Arm is busy', str(response)
    check('reset_while_busy_rejected')
    rejected = probe.future(probe.arm.send_goal_async(probe.goal('red', 'left')))
    assert not rejected.accepted
    check('busy_action_rejected')
    probe.cancel.publish(String(data='cancel'))
    record = probe.task_result(since)
    assert not record['success'] and 'Canceled' in record['message'], record
    check('task_cancel_after_grasp', record)

    probe.reset_scene()
    handle = probe.future(probe.arm.send_goal_async(probe.goal('green', 'left')))
    assert handle.accepted
    response = probe.future(handle.cancel_goal_async())
    assert response.goals_canceling
    result = probe.future(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_CANCELED and not result.result.success
    check('direct_action_cancel')

    probe.reset_scene()
    request = probe.goal('red', 'left')
    request.place.x = float('nan')
    handle = probe.future(probe.arm.send_goal_async(request))
    assert handle.accepted
    result = probe.future(handle.get_result_async())
    assert result.status == GoalStatus.STATUS_ABORTED and 'Non-finite' in result.result.message
    check('invalid_action_aborted')

    probe.reset_scene()
    since = len(probe.statuses)
    probe.command.publish(String(data='Положи синий кубик в правую коробку'))
    record = probe.task_result(since)
    assert record['success'], record
    probe.placed('blue', 'right')
    check('recovery_and_russian_command', record)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--output-dir', type=Path, default=Path('artifacts/headless'))
    parser.add_argument('--domain-id', type=int, default=83)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    args.output_dir = args.output_dir.resolve()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['ROS_LOG_DIR'] = str(args.output_dir / 'ros_logs')
    report = {'gui': args.gui, 'checks': [], 'passed': False}
    results_file = args.output_dir / f'results-{time.time_ns()}.jsonl'
    report['results_file'] = str(results_file)
    def check(name, detail=True):
        report['checks'].append({'name': name, 'detail': detail})
        print(f'PASS {name}: {detail}', flush=True)
    process = None
    probe = None
    rclpy.init()
    try:
        with (args.output_dir / 'launch.log').open('w') as log:
            process = subprocess.Popen([
                'ros2', 'launch', 'tabletop_robot', 'demo.launch.py',
                f'gui:={str(args.gui).lower()}', 'realtime:=true', 'seed:=42',
                f'results_file:={results_file}',
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            probe = Probe()
            run(probe, check)
            records = [json.loads(line) for line in results_file.read_text().splitlines()]
            assert len(records) == 3 and sum(r['success'] for r in records) == 2, records
            check('results_journal', records)
            assert process.poll() is None, 'launch exited unexpectedly'
            report['passed'] = True
    except Exception:
        report['error'] = traceback.format_exc()
        print(report['error'], flush=True)
    finally:
        if probe is not None:
            probe.destroy_node()
        rclpy.shutdown()
        if process is not None and process.poll() is None:
            # launch forwards SIGINT itself; signaling its group sends it twice.
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        (args.output_dir / 'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
