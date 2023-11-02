import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from felix.scripts.camera import Camera
import cv2
from cv_bridge import CvBridge, CvBridgeError

'''
header:
  stamp:
    sec: 1692294375
    nanosec: 199846006
  frame_id: ''
height: 720
width: 1280
encoding: bgr8
is_bigendian: 0
step: 3840

'''
class CameraNode(Node):

    def __init__(self):
        super().__init__("camera_node")
        self.camera = Camera()
        self.bridge = CvBridge()
        self.video_publisher = self.create_publisher(Image,"video_source/raw", 10)
        
        self.create_timer(.01, self.publish_video)
        self.x = 0.0

    def log(self, txt: str):
        self.get_logger().info(txt)


    def publish_video(self):
        try:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "camera"
            image = self.camera.read()
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            msg = self.bridge.cv2_to_imgmsg(image_rgb, encoding="rgb8", header=header)
            self.video_publisher.publish(msg)
        except Exception as ex:
            self.get_logger().warn(f"Could not get image {ex.__str__()}")


    def shutdown(self):
        self.camera.stop()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node=node)
    node.shutdown()
    rclpy.shutdown()