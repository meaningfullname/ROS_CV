import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Point
from tabletop_interfaces.msg import ObjectArray
from tabletop_interfaces.action import PickPlace
from .commands import parse_command
from .config import TARGETS


class TaskNode(Node):
    def __init__(self):
        super().__init__('tabletop_tasks')
        self.declare_parameter('results_file', 'results.jsonl')
        self.results_file = Path(self.get_parameter('results_file').value).expanduser()
        self.objects = []
        self.observed_at = 0.
        self.busy = False
        self.started_at = 0.
        self.command = ''
        self.goal = None
        self.client = ActionClient(self, PickPlace, '/arm/pick_place')
        self.status = self.create_publisher(String, '/task/status', 10)
        self.create_subscription(ObjectArray, '/perception/objects', self.observe, 10)
        self.create_subscription(String, '/task/command', self.submit, 10)
        self.create_subscription(String, '/task/cancel', self.cancel, 10)
        self.create_timer(1., self.check_deadline)

    def report(self, text):
        self.status.publish(String(data=text))
        self.get_logger().info(text)

    def observe(self, message):
        self.objects = message.objects
        self.observed_at = time.monotonic()

    def submit(self, message):
        if self.busy:
            self.report('Rejected: task already running')
            return
        try:
            command = parse_command(message.data)
            if time.monotonic() - self.observed_at > 2:
                raise ValueError('No fresh camera observations')
            candidates = [obj for obj in self.objects if obj.color == command.color]
            if len(candidates) != 1:
                raise ValueError('Expected exactly one matching object')
            if not self.client.server_is_ready():
                raise ValueError('Arm action server unavailable')
        except ValueError as error:
            self.report(f'Rejected: {error}')
            return
        self.busy = True
        self.command = message.data
        self.started_at = time.monotonic()
        request = PickPlace.Goal()
        request.pick = candidates[0].position
        request.place = Point(x=TARGETS[command.destination][0],
                              y=TARGETS[command.destination][1], z=0.025)
        self.report('Task accepted')
        future = self.client.send_goal_async(request, feedback_callback=self.feedback)
        future.add_done_callback(self.accepted)

    def feedback(self, message):
        self.report(message.feedback.stage)

    def accepted(self, future):
        try:
            self.goal = future.result()
            if not self.goal.accepted:
                self.finish(False, 'Arm rejected goal')
                return
            self.goal.get_result_async().add_done_callback(self.completed)
        except Exception as error:
            self.finish(False, str(error))

    def completed(self, future):
        try:
            result = future.result().result
            self.finish(result.success, result.message)
        except Exception as error:
            self.finish(False, str(error))

    def finish(self, success, message):
        record = {'command': self.command, 'success': success, 'message': message,
                  'elapsed_seconds': round(time.monotonic() - self.started_at, 3)}
        self.busy = False
        self.goal = None
        try:
            self.results_file.parent.mkdir(parents=True, exist_ok=True)
            with self.results_file.open('a') as stream:
                stream.write(json.dumps(record, ensure_ascii=False) + '\n')
        except OSError as error:
            self.get_logger().error(f'Cannot save result: {error}')
        self.report(json.dumps(record, ensure_ascii=False))

    def cancel(self, message):
        if self.goal is not None:
            self.goal.cancel_goal_async()

    def check_deadline(self):
        if self.busy and time.monotonic() - self.started_at > 50:
            self.cancel(None)
            self.report('Waiting for arm cancellation after deadline')


def main():
    rclpy.init()
    node = TaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
