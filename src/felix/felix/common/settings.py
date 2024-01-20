from typing import List, Optional, Any, Dict
import os

class TrainingType:
    OBSTACLE="OBSTACLE"
    PATH="PATH"

class TrainingProfile:

    def __init__(
        self, 
        data_root: str, 
        name: str, 
        type: str,
        classifier: str, 
        categories: List[str], 
        velocity_map: Dict,
        epochs: int = 30, 
        learning_rate: float = 0.001, 
        momentum: float = 0.9
        ):
        self.type = type
        self.name = name
        self.filename = name.lower().replace(" ","_")
        self.classifier = classifier
        self.categories = categories
        self.velocity_map = velocity_map
        self.data_root = data_root
        self.planning_path = os.path.join(data_root,"planning")
        self.snapshot_path = os.path.join(data_root,"snapshots")
        self.model_root = os.path.join(data_root,"models")
        self.best_model_folder = os.path.join(self.model_root,"best")
        self.best_model_file = os.path.join(self.best_model_folder,self.filename+".pth")
        self.training_data_path = os.path.join(self.data_root, self.filename)
        self.num_categories = len(self.categories)
        self.onnx_folder = os.path.join(self.model_root,"onnx")
        self.onnx_file = os.path.join(self.onnx_folder,self.filename+".onnx")
        self.trt_folder = os.path.join(self.model_root, "trt")
        self.trt_file = os.path.join(self.trt_folder,f"{self.filename}.trt")

obstacle3d_profile= TrainingProfile(
    data_root="/felix/data",
    name="obstacle3d",
    type=TrainingType.OBSTACLE,
    classifier="alexnet",
    categories=["forward","left","right"],
    velocity_map={"forward": (0.5,0,0), "left": (0,0,0.3), "right": (0,0,-0.3)}
)

obstacle_profile= TrainingProfile(
    data_root="/felix/data",
    name="obstacle",
    type=TrainingType.OBSTACLE,
    classifier="alexnet",
    categories=["blocked","free"],
    velocity_map={"blocked": (0,0,0.3), "free": (0.5,0,0), }
)  

path_profile= TrainingProfile(
    data_root="/felix/data",
    name="path_planning",
    type=TrainingType.PATH,
    classifier="alexnet",
    categories=[],
    velocity_map={}
)   


class AppSettings:

    class Topics:
        raw_video: str = "/left/image_raw"
        cmd_vel: str = "/cmd_vel"
        autodrive: str = "/autodrive"
        
    class Robot:
        wheel_radius: float = 65.00/2000.0
        wheel_base: float = 140.0/1000.0
        track_width: float = 130.00/1000
        wheel_x_offset: float = 71.5/1000
        body_length: float = 152/1000
        body_width: float = 126.00/1000
        encoder_resolution: int = int(1000/48)


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

    
    
    class Camera:
        width: int = 1640
        height: int = 1232
        fps: float = 30
        capture_width: int = 1640
        capture_height:int = 1232
        stereo: bool = False
        fov: int=160
    

    Training: TrainingProfile = obstacle3d_profile

    debug: bool = False

settings = AppSettings

