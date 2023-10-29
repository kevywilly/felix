from typing import List, Optional, Any
import torchvision
import os

class CameraConfig:

    def __init__(self, 
        width: int = 1280, 
        height: int = 720, 
        fps: float = 30, 
        capture_width: int = 1280, 
        capture_height:int = 720,
        stereo: bool = False):

        self.width = width
        self.height = height
        self.fps = fps
        self.capture_width = capture_width
        self.capture_height = capture_height
        self.stereo = stereo

class TrainingProfile:

    def __init__(self, data_root: str, name: str, classifier: str, categories: List[str], epochs: int = 30, learning_rate: float = 0.001, momentum: float = 0.9):
        self.name = name
        self.filename = name.lower().replace(" ","_")
        self.classifier = classifier
        self.categories = categories
        self.data_root = data_root
        self.model_root = os.path.join(data_root,"models")
        self.best_model_folder = os.path.join(self.model_root,"best")
        self.best_model_file = os.path.join(self.best_model_folder,self.filename+".pth")
        self.training_data_path = os.path.join(self.data_root, self.filename)
        self.num_categories = len(self.categories)


    def load_model(self, *args, **kwargs):
        if self.classifier.lower() == "alexnet":
            return torchvision.models.alexnet(*args, **kwargs)
        elif self.classifier.lower() == "resnet18":
            return torchvision.models.resnet18(*args, **kwargs)
        else:
            raise "Invalid model specified, please use 'alexnet' or 'resnet18"

      
class AppSettings:

    class Topics:
        raw_video: str = "/video_source/raw"
        
    class Robot:
        wheel_radius: float = 95.00
        wheel_base: float = 150.00
        wheel_x_offset: float = 145.00
        body_length: float = 144.00
        body_width: float = 126.00

    class Motion:
        linear_velocity_multiple: float = 0.2
        angular_velocity_multiple: float = 0.4

    Training: TrainingProfile = TrainingProfile(
            data_root="/felix/data",
            name="obstacle3d",
            classifier="alexnet",
            categories=["forward", "left", "right"]
        )   
    

    Camera: CameraConfig = CameraConfig(
        width = 1280, height=720, fps=30, capture_width=1280, capture_height=720, stereo=False
    )

    debug: bool = False

settings = AppSettings