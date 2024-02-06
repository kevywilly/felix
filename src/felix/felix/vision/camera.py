from felix.vision.sensor_mode import CameraSensorMode
import cv2
from cv2 import VideoCapture
from felix.config.settings import settings
import felix.vision.image_utils as image_utils
import atexit

def cv2_to_jpeg_bytes(frame):
        x = cv2.cvtColor(frame , cv2.COLOR_BGR2RGB) # type: ignore
        return bytes(cv2.imencode('.jpg', x)[1]) # type: ignore

def convert_I420_2_RGB_color(frame):
        # CONVERT TO RGB
        return cv2.cvtColor(frame, cv2.COLOR_YUV2RGB_I420) # type: ignore

def undistort(frame):
        return cv2.undistort(frame, settings.CAMERA_MATRIX, settings.DISTORTION_COEFFICIENTS) # type: ignore

class ArgusCamera:
    def __init__(self, sensor_id, sensor_mode: CameraSensorMode):
        self._sensor_id = sensor_id
        self._sensor_mode = sensor_mode
        self.value = None
        self.jpeg = None
        self._cap = self._init_camera()
        atexit.register(self.release)


    def _init_camera(self):
        cap = VideoCapture(self._sensor_mode.to_nvargus_string(self._sensor_id))
        ret, frame = cap.read()
        if not ret:
            raise Exception("Could not initialize camera")
        else:
            self.value = frame
            return cap    
    
    def read(self) -> bool:
        ret, frame = self._cap.read()

        if not ret:
            print(f"Can't receive frame for cap{self._sensor_id}")
            return False
        else:
            frame = convert_I420_2_RGB_color(frame)
            frame = undistort(frame)
            self.value = frame
            self.jpeg = cv2_to_jpeg_bytes(frame)
            
        return True
    
    def release(self):
        if self._cap:
            self._cap.release()