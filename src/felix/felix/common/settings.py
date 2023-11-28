from typing import List, Optional, Any, Dict
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

    def __init__(
        self, 
        data_root: str, 
        name: str, 
        classifier: str, 
        categories: List[str], 
        velocity_map: Dict,
        epochs: int = 30, 
        learning_rate: float = 0.001, 
        momentum: float = 0.9
        ):
        self.name = name
        self.filename = name.lower().replace(" ","_")
        self.classifier = classifier
        self.categories = categories
        self.velocity_map = velocity_map
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
            raise Exception("Invalid model specified, please use 'alexnet' or 'resnet18")

      
class AppSettings:

    class Topics:
        raw_video: str = "/left/image_raw"
        
    class Robot:
        wheel_radius: float = 95.00/1000
        wheel_base: float = 150.00/1000
        wheel_x_offset: float = 145.00/1000
        body_length: float = 144.00/1000
        body_width: float = 126.00/1000


    class Motion:
        """
        calibrating linear velocity
        rpm_min_max: 80.24951177174123,333.69585487749697, vel_min_max: 0.1270617269719236,0.5283517702227035, avg_adj_factor: 0.40684325290805

        angular
        rpm_min_max: -321.83081750356047,0.0, vel_min_max: 0.16695583936676953,3.4243431655481014, avg_adj_factor: 0.283714400265866

        see calibrate_speed.py
        """ 

        rpm_min: float = 80.25
        rpm_max: float = 333.7
        vx_min: float = 0.127
        vx_max: float = 0.29
        vx_adjustment_factor = 0.68

        vz_min: float = 0.167
        vz_max: float = 1.8
        vz_adjustment_factor = 0.68
      
        linear_velocity_multiple: float = 0.2
        angular_velocity_multiple: float = 0.4
        autodrive_speed: float = 0.35

    class Db:
        path: str = "/felix/data/db/felix_db.sqlite"

    Training: TrainingProfile = TrainingProfile(
            data_root="/felix/data",
            name="obstacle",
            classifier="alexnet",
            categories=["free","blocked"],
            velocity_map={"free": (1.0,0,0), "blocked": (0,0,1.0)}
        )   
    

    Camera: CameraConfig = CameraConfig(
        width = 1280, height=720, fps=30, capture_width=1280, capture_height=720, stereo=False
    )

    debug: bool = False

settings = AppSettings

