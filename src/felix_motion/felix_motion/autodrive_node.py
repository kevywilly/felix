import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool

from felix.scripts.settings import settings
import felix.scripts.image_utils as image_utils
import os
import numpy as np
import torch
import torch.nn.functional as F
import torchvision
from enum import Enum

torch.hub.set_dir(settings.Training.model_root)

class DriveState(Enum):
    STOPPED = 0
    DRIVING = 1
    AVOIDING = 2

class AutoDriveNode(Node):

    def __init__(self):
        super().__init__("autodrive_node", parameter_overrides=[])

        self.log("Started autodrive node.")
        self.drive_state = DriveState.STOPPED
        self.mean = 255.0 * np.array([0.485, 0.456, 0.406])
        self.stdev = 255.0 * np.array([0.229, 0.224, 0.225])
        self.normalize = torchvision.transforms.Normalize(self.mean, self.stdev)
        self._load_model()
        self.running = False
        

        # publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.log("Created Drivetrain Publisher")

        # subscriptions
        self.image_subscription = self.create_subscription(Image, "/video_source/raw", self.autodrive, 10)
        self.log("Subscribed to Image Topic")

        # service
        self.srv = self.create_service(SetBool, "set_autodrive_state", self.set_run_state)
        
        self.log("Initialized autodrive node.")
        

    def set_run_state(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if self.running == request.data:
            response.message = "Autodrive already in requested state."
            response.success = False
        else:
            response.message = f"Setting autodrive state to {'on' if request.data else 'off'}"
            response.success = True
            self.start() if request.data == True else self.stop()
        
        self.get_logger().info(response.message)
        
        return response
    def toggle_status(self, request, response):
        self.log(f"Setting autodrive running state to {request.on}")
        self.start() if request.on else self.stop()
        response.status = self.running
        return response

    def log(self, txt: str):
        self.get_logger().info(txt)


    def stop_drivetrain(self):
        msg = Twist()
        self.drivetrain_publisher.publish(msg)
        self.drive_state = DriveState.STOPPED

    def start(self):
        self.running = True
        self.drive_state = DriveState.STOPPED


    def stop(self):
        self.running = False

    def _load_model(self):
        self.log("Preparing model...")
        
        model_file = settings.Training.best_model_file  

        self.log(f"Looking for saved model {model_file}")  
        has_model = os.path.isfile(model_file)

        if has_model: 
            self.log(f"Found saved model.") 
        else:
            self.log(f"Did not saved model. Proceeding without it.") 

        self.model = settings.Training.load_model(pretrained=(not has_model))
        self.device = torch.device('cuda')

        cat_count = settings.Training.num_categories

        # create model
        if settings.Training.classifier == "alexnet":
            self.model.classifier[6] = torch.nn.Linear(self.model.classifier[6].in_features, cat_count)
        elif settings.Training.classifier == "resnet18":
            self.model.fc = torch.nn.Linear(512, cat_count)
            self.model.eval().half()

        if has_model:
            self.log(f"Loading saved state ... {model_file}")
            self.model.load_state_dict(torch.load(model_file))
        else:
            self.log("Skipping saved state load, model does not exist yet.")

        self.model = self.model.to(self.device)

        self.log("model ready...")

    def _preprocess(self, sensor_image):
        cuda_image = image_utils.sensor_image_to_cuda(sensor_image)
        x = image_utils.resize_cuda(cuda_image, 224, 224)
        x = np.transpose(x, (2, 0, 1))
        x = torch.as_tensor(x, device='cuda').float()
        x = self.normalize(x)
        x = x[None, ...]
        return x

    def predict(self, sensor_image):
        input = self._preprocess(sensor_image=sensor_image)
        output = self.model(input)
        output = F.softmax(output, dim=1)
        return output

    def _assign_predictions(self, y, categories):
        categories = sorted(settings.Training.categories.copy())
        d = {}
        for index, cat in enumerate(categories):
            d[cat] = float(y.flatten()[index])
        predictions = sorted(d.items(),key=lambda x:x[1], reverse=True)
        return predictions

    def autodrive(self, sensor_image: Image):
        if not self.running:
            self.drive_state = DriveState.STOPPED
            return

        y = self.predict(sensor_image)

        predictions = self._assign_predictions(y, settings.Training.categories)

        self.log(f"Prediction: {predictions}")

        k,v = predictions[0]

        new_state = DriveState.DRIVING if k == "forward" else DriveState.AVOIDING
        
        if new_state != self.drive_state:
            self.drive_state = new_state
            if not settings.debug:
                t = Twist()
                vel_vector = settings.Training.velocity_map.get(k)

                t.linear.x, t.linear.y, t.angular.z = [v * settings.Motion.autodrive_speed for v in vel_vector]
                
                self.cmd_vel_publisher.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = AutoDriveNode()
    rclpy.spin(node=node)
    node.stop()
    torch.cuda.empty_cache()
    rclpy.shutdown()
    





    