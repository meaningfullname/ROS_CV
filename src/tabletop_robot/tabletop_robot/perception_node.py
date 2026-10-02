import rclpy
import message_filters
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tabletop_interfaces.msg import DetectedObject, ObjectArray
from .vision import detect_objects


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('tabletop_perception')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(ObjectArray, '/perception/objects', 10)
        self.subscribers = [message_filters.Subscriber(self, kind, topic)
                            for kind, topic in [(Image, '/camera/rgb'),
                                                (Image, '/camera/depth'),
                                                (CameraInfo, '/camera/info')]]
        self.sync = message_filters.TimeSynchronizer(self.subscribers, 10)
        self.sync.registerCallback(self.process)

    def process(self, rgb, depth, info):
        objects = detect_objects(self.bridge.imgmsg_to_cv2(rgb, 'rgb8'),
                                 self.bridge.imgmsg_to_cv2(depth, '32FC1'), info.k[0])
        message = ObjectArray()
        message.header.stamp = rgb.header.stamp
        message.header.frame_id = 'world'
        for color, position in objects:
            obj = DetectedObject()
            obj.color = color
            obj.position.x, obj.position.y, obj.position.z = position
            obj.confidence = 1.0
            message.objects.append(obj)
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
