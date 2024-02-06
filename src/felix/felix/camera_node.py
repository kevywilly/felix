
import rclpy
from rclpy.node import Node
import cv2
from cv2 import VideoCapture
import atexit
from typing import Any
from felix.config.settings import settings
from sensor_msgs.msg import Image
from felix.vision.camera import ArgusCamera
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError

class CameraNode(Node):
    def __init__(self):
        super().__init__("camera", parameter_overrides=[])
        self._sensor_mode = settings.DEFAULT_SENSOR_MODE
        self._sensor_id = 0
        self._camera = ArgusCamera(sensor_id=self._sensor_id, sensor_mode=self._sensor_mode)
        self.bridge = CvBridge()
        
        self._value: Any = None
        
        self.create_timer(1.0/self._sensor_mode.framerate,self._read)
        self._image_publisher = self.create_publisher(Image,settings.Topics.raw_video,5)
        
    
        self.get_logger().info("Camera node is up!")

        
    def _read(self):
        if self._camera.read():
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "camera"
            msg = self.bridge.cv2_to_imgmsg(self._camera.value, encoding="rgb8", header=header)
            self._image_publisher.publish(msg)
        else:
            self.get_logger().warn(f"Can't receive frame for cap{self._sensor_id}")
    

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    rclpy.shutdown()