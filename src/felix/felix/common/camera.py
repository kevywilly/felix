import atexit
import cv2
import numpy as np
import logging

class Camera:
    
    def __init__(self, source: str = "csi://0", width: int = 1280, height: int = 720, fps: int =  30, capture_width: int = 1280, capture_height: int = 720):
        
        self.width = width
        self.height = height
        self.fps = fps
        self.capture_width = capture_width
        self.capture_height = capture_height
        self.source = source
        

        self.value = np.empty((self.height, self.width, 3), dtype=np.uint8)

        self.start()

        atexit.register(self.stop)

    def start(self):
        try:
            self.cap = cv2.VideoCapture(self._gst_str(), cv2.CAP_GSTREAMER)

            re, image = self.cap.read()

            if not re:
                raise RuntimeError('Could not read image from camera.')

            self.value = image
            
        except:
            self.stop()
            raise RuntimeError(
                'Could not initialize camera.  Please see error trace.')

    def read(self):
       
        re, image = self.cap.read()
            
        if re:
            self.value = image
        else:
            print("could not get image")

        return self.value

                
    def _gst_str(self):
        return 'nvarguscamerasrc sensor-mode=3 ! video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, framerate=(fraction)%d/1 ! nvvidconv ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! appsink' % (
                self.capture_width, self.capture_height, self.fps, self.width, self.height)
    
  

    def stop(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        
            
    def restart(self):
        self.stop()
        self.start()
        
    