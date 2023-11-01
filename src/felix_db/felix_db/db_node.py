import sqlite3

import rclpy
from rclpy.node import Node
from felix.scripts.database import Database, Motion
from felix_interfaces.msg import MotionData
import felix.scripts.image_utils as image_utils
from json import JSONEncoder
import json
import numpy as np
import cv2 as cv


class NumpyArrayEncoder(JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return JSONEncoder.default(self, obj)


class DbNode(Node):
    def __init__(self):
        super().__init__("db_node", parameter_overrides=[])
        self.db = Database()
        self.create_subscription(MotionData, '/motion', self.got_motion_data, 10)

    def got_motion_data(self, msg: MotionData):
        
        img = image_utils.sensor_image_to_cv2(msg.image)
        img = cv.resize(img, (240,240), interpolation = cv.INTER_AREA)
        j = {"image": img}
        encoded = json.dumps(j, cls=NumpyArrayEncoder)
        #m.image = encoded
        
        m = Motion(
            vx=msg.v0.linear.x, 
            vy=msg.v0.linear.y,
            vz=msg.v0.angular.z,
            rx=msg.v1.linear.x,
            ry=msg.v1.linear.y,
            rz=msg.v1.angular.z,
            image=encoded
        )
        
        self.save_data(m)

    def save_data(self, m: Motion):
        self.get_logger().info(f"saving motion")
        self.db.insert_motion(m)

def main(args=None):
    rclpy.init(args=args)
    node = DbNode()
    rclpy.spin(node)
    try:
        rclpy.shutdown()
    except:
        pass