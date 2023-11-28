import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from felix.common.settings import settings
import felix.common.image_utils as image_utils
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

        self.get_logger().info("Started autodrive node.")
        self.drive_state = DriveState.STOPPED
        self.mean = 255.0 * np.array([0.485, 0.456, 0.406])
        self.stdev = 255.0 * np.array([0.229, 0.224, 0.225])
        self.normalize = torchvision.transforms.Normalize(self.mean, self.stdev)
        self._load_model()
        self.running = False
        self.create_subscription(Bool, "autodrive", self.set_run_state, 10)
        # publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_subscription = self.create_subscription(Image, settings.Topics.raw_video, self.autodrive, 10)
        
        self.get_logger().info("Initialized autodrive node.")
        

    def set_run_state(self, msg: Bool):
        self.running = msg.data
        if msg.data:
            self.stop()
        else:
            self.start()
        
        self.get_logger().info(f"Set autodrive state to: {'on' if msg.data else 'off'}")


    def stop_drivetrain(self):
        msg = Twist()
        self.cmd_vel_publisher.publish(msg)
        self.drive_state = DriveState.STOPPED

    def start(self):
        self.running = True
        self.drive_state = DriveState.STOPPED

    def stop(self):
        self.running = False

    def _load_model(self):
        self.get_logger().info("Preparing model...")
        
        model_file = settings.Training.best_model_file  

        self.get_logger().info(f"Looking for saved model {model_file}")  
        has_model = os.path.isfile(model_file)

        if has_model: 
            self.get_logger().info(f"Found saved model.") 
        else:
            self.get_logger().info(f"Did not saved model. Proceeding without it.") 

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
            self.get_logger().info(f"Loading saved state ... {model_file}")
            self.model.load_state_dict(torch.load(model_file))
        else:
            self.get_logger().info("Skipping saved state load, model does not exist yet.")

        self.model = self.model.to(self.device)

        self.get_logger().info("model ready...")

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

        self.get_logger().info(f"Prediction: {predictions}")

        k,v = predictions[0]

        new_state = DriveState.DRIVING if k == "free" else DriveState.AVOIDING
        
        if new_state != self.drive_state:
            self.drive_state = new_state
            if not settings.debug:
                t = Twist()
                vel_vector = settings.Training.velocity_map.get(k)
                if not vel_vector:
                    vel_vector = [0,0,0]

                t.linear.x, t.linear.y, t.angular.z = [v * settings.Motion.autodrive_speed for v in vel_vector]
                
                self.cmd_vel_publisher.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = AutoDriveNode()
    rclpy.spin(node=node)
    node.stop()
    torch.cuda.empty_cache()
    rclpy.shutdown()
    





    