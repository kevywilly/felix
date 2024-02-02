
import rclpy
from rclpy.node import Node
import cv2
from cv2 import VideoCapture
import atexit
from typing import Any
import felix.vision.image_utils as image_utils
from felix.config.settings import settings
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError

class CameraNode(Node):
    def __init__(self):
        super().__init__("camera", parameter_overrides=[])
        self.bridge = CvBridge()
        self._sensor_id = 0
        self._value: Any = None
        self._sensor_mode = settings.DEFAULT_SENSOR_MODE
        self._cap = self._init_camera()
        self.create_timer(1.0/self._sensor_mode.framerate,self._read)
        self._image_publisher = self.create_publisher(Image,settings.Topics.raw_video,5)
        
        atexit.register(self._release)

        self.get_logger().info("Camera node is up!")

    def _init_camera(self) -> VideoCapture:
        cap = VideoCapture(self._sensor_mode.to_nvargus_string(self._sensor_id))
        ret, frame = cap.read()
        if not ret:
            raise Exception("Could not initialize camera")
        else:
            self._value = frame
            return cap
    
    def _convert_color(self, frame):
        # CONVERT TO RGB
        return cv2.cvtColor(frame, cv2.COLOR_YUV2RGB_I420) # type: ignore

    def _undistort(self, frame):
        return cv2.undistort(frame, settings.CAMERA_MATRIX, settings.DISTORTION_COEFFICIENTS) # type: ignore
       
        
    def _read(self):
        ret, frame = self._cap.read()

        if not ret:
            self.get_logger().warn(f"Can't receive frame for cap{self._sensor_id}")
        else:
            frame = self._convert_color(frame)
            frame = self._undistort(frame)
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "camera"
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8", header=header)
            self._image_publisher.publish(msg)
    

    def _release(self):
        if self._cap:
            self._cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    rclpy.shutdown()