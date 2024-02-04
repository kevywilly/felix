import os
import rclpy
import math
import torchvision
import torch
import torchvision.transforms as transforms
import torch.nn.functional as F
import cv2
import PIL.Image
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image as ImageMsg
from rclpy.node import Node
from felix.config.settings import settings

from felix.motion.kinematics import Kinematics

from cv_bridge import CvBridge

data_path = settings.Training.planning_path
best_path = settings.Training.best_model_folder

BEST_MODEL_PATH = os.path.join(settings.Training.best_model_folder,"planning.pth")

model = torchvision.models.resnet18(pretrained=False)
model.fc = torch.nn.Linear(512, 2)
model.load_state_dict(torch.load(BEST_MODEL_PATH))

class RoadFollower(Node):
    def __init__(self):
        super().__init__("road_follower", parameter_overrides=[])
        self.device = torch.device('cuda')
        self.model = model.to(self.device)
        self.model = model.eval().half()

        self.mean = torch.Tensor([0.485, 0.456, 0.406]).cuda().half()
        self.stdev = torch.Tensor([0.229, 0.224, 0.225]).cuda().half()


        #self.normalize = torchvision.transforms.Normalize(self.mean, self.stdev)

        self.create_subscription(ImageMsg, settings.Topics.raw_video, self.got_image, 5)
        self.nav_publisher = self.create_publisher(Odometry, settings.Topics.cmd_nav,10)

        self.angle = 0.0
        self.angle_last = 0.0
        self.bridge = CvBridge()

    def preprocess(self, image):
        image = PIL.Image.fromarray(image)
        image = transforms.functional.to_tensor(image).to(self.device).half()
        image.sub_(self.mean[:, None, None]).div_(self.stdev[:, None, None])
        return image[None, ...]
    
    def got_image(self, msg):

        #image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        image = PIL.Image.fromarray(img)
        image = transforms.functional.to_tensor(image).to(self.device).half()
        image.sub_(self.mean[:, None, None]).div_(self.stdev[:, None, None])
        image = image[None, ...]
        #h,w,_ = image.shape
        xy = model(image).detach().float().cpu().numpy().flatten()
        x = xy[0]
        y = (0.5 - xy[1]) / 2.0

        self.angle = np.arctan2(x, y)
        self.angle_last = self.angle
        self.get_logger().info(f'predict x: {x} y: {y} a: {math.degrees(self.angle)}')

        

        #odom = Kinematics.xywh_to_nav_target(x,y,w,h)
        #self.get_logger().info(f'Odom: {odom.twist.twist.linear.x}, {odom.twist.twist.angular.z}')
        #self.nav_publisher.publish(odom)
        
def main(args=None):
    rclpy.init(args=args)
    node = RoadFollower()
    rclpy.spin(node)
    rclpy.shutdown()