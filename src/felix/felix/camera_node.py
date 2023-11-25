import rclpy
import atexit
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from felix.common.camera import Camera
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

CAMERA_WIDTH=1640
CAMERA_HEIGHT=1280
CAMERA_FPS = 30
CAMERA_SENSOR_MODE = 3

bridge = CvBridge()

class CameraNode(Node):

    def __init__(self):
        super().__init__("camera_node", parameter_overrides=[])

        self._init_camera(0)

        self.video_publisher = self.create_publisher(Image,"video_source/raw", 5)
        self.create_timer(1.0/CAMERA_FPS, self.publish_video)

        self.x = 0.0
        atexit.register(self.shutdown)


    def _init_camera(self, sensor_id: int = 0):
        
        self.camera_0 = cv2.VideoCapture(self._gst_str(sensor_id), cv2.CAP_GSTREAMER)

        ret, frame = self.camera_0.read()

        if not ret:
            raise RuntimeError('Could not read image from camera.')


    def _gst_str(self, sensor_id: int = 0) -> str:
        #camera_0_ = cv::VideoCapture("nvarguscamerasrc sensor_id=0 sensor_mode=3 ! video/x-raw(memory:NVMM), width=(int)1640, height=(int)1232,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)I420 ! appsink max-buffers=1 drop=true");
    
        return f"nvarguscamerasrc sensor_id={sensor_id} sensor_mode={CAMERA_SENSOR_MODE} ! video/x-raw(memory:NVMM), width=(int){CAMERA_WIDTH}, height=(int){CAMERA_HEIGHT},format=(string)NV12, framerate=(fraction){CAMERA_FPS}/1 ! nvvidconv ! video/x-raw, format=(string)I420 ! appsink max-buffers=1 drop=true";

    def _release(self):
        if self.camera_0:
            self.camera_0.release()

    def publish_video(self):
        try:

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "camera"
            ret, frame = self.camera_0.read()
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_I420)
            msg = bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8", header=header)
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