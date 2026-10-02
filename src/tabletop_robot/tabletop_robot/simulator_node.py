import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from tabletop_interfaces.action import PickPlace
from .world import TabletopWorld, MotionStopped


class SimulatorNode(Node):
    def __init__(self):
        super().__init__('tabletop_simulator')
        self.declare_parameter('gui', False)
        self.declare_parameter('seed', 42)
        self.declare_parameter('realtime', False)
        self.world = TabletopWorld(self.get_parameter('gui').value,
                                  self.get_parameter('seed').value)
        self.lock = threading.RLock()
        self.state_lock = threading.Lock()
        self.busy = False
        self.bridge = CvBridge()
        self.rgb = self.create_publisher(Image, '/camera/rgb', 10)
        self.depth = self.create_publisher(Image, '/camera/depth', 10)
        self.info = self.create_publisher(CameraInfo, '/camera/info', 10)
        group = ReentrantCallbackGroup()
        self.server = ActionServer(self, PickPlace, '/arm/pick_place', self.execute,
                                   callback_group=group, goal_callback=self.accept,
                                   cancel_callback=lambda goal: CancelResponse.ACCEPT)
        self.create_service(Trigger, '/scene/reset', self.reset, callback_group=group)
        self.create_timer(0.2, self.publish_camera, callback_group=group)

    def accept(self, request):
        with self.state_lock:
            if self.busy:
                return GoalResponse.REJECT
            self.busy = True
            return GoalResponse.ACCEPT

    def reset(self, request, response):
        with self.state_lock:
            response.success = not self.busy
            if response.success:
                with self.lock:
                    self.world.seed += 1
                    self.world.reset()
            response.message = 'Scene reset' if response.success else 'Arm is busy'
        return response

    def publish_camera(self):
        if not self.lock.acquire(blocking=False):
            return
        try:
            if self.busy:
                return
            rgb, depth, focal = self.world.camera()
        finally:
            self.lock.release()
        stamp = self.get_clock().now().to_msg()
        for publisher, array, encoding in [(self.rgb, rgb, 'rgb8'),
                                            (self.depth, depth, '32FC1')]:
            message = self.bridge.cv2_to_imgmsg(array, encoding)
            message.header.stamp = stamp
            message.header.frame_id = 'camera_optical'
            publisher.publish(message)
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = 'camera_optical'
        info.width = info.height = 320
        info.k = [focal, 0., 159.5, 0., focal, 159.5, 0., 0., 1.]
        self.info.publish(info)

    def execute(self, goal):
        start = time.monotonic()
        result = PickPlace.Result()
        stop = lambda: goal.is_cancel_requested or time.monotonic() - start > 45
        def feedback(stage):
            message = PickPlace.Feedback()
            message.stage = stage
            goal.publish_feedback(message)
        try:
            pick = goal.request.pick
            place = goal.request.place
            with self.lock:
                result.success = self.world.pick_place(
                    (pick.x, pick.y, pick.z), (place.x, place.y, place.z),
                    stop, feedback, self.get_parameter('gui').value or
                    self.get_parameter('realtime').value)
            result.message = 'Placed successfully' if result.success else 'Placement verification failed'
            if result.success:
                goal.succeed()
            else:
                goal.abort()
        except MotionStopped as error:
            result.message = str(error)
            if goal.is_cancel_requested:
                goal.canceled()
            else:
                goal.abort()
        except Exception as error:
            result.message = str(error)
            goal.abort()
        finally:
            with self.state_lock:
                self.busy = False
        result.elapsed_seconds = float(time.monotonic() - start)
        return result


def main():
    rclpy.init()
    node = SimulatorNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.world.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
