import rclpy
from felix.common.rosmaster import Rosmaster
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from rclpy.node import Node
from felix.common.settings import settings

from typing import Optional

import math
import time
import atexit

import numpy as np

TWIST_ZERO = Twist()

velocity_factor = 2*settings.Robot.wheel_radius*math.pi
tick_delay = 0.5
max_wheel_velocity = 0.29
LEFT=0
RIGHT=2

class MotionNode(Node):
    def __init__(self):
        super().__init__("motion_node", parameter_overrides=[])
        self.bot = Rosmaster(car_type=0, com="/dev/ttyUSB0")
        self.bot.create_receive_threading()
        self.create_subscription(Twist, "/cmd_vel", self.handle_cmd_vel, 10)
        self.tick_timer = self.create_timer(0.5, self.ticks_callback)
        self.wheel_radius = settings.Robot.wheel_radius
        self.wheel_base = settings.Robot.wheel_base
        self.prev_ticks = np.array(self.bot.get_motor_encoder())
        self.left_target_velocity = 0
        self.right_target_velocity = 0
        self.left_velocity = 0
        self.right_velocity = 0
        self.linear_x = 0
        self.linear_y = 0
        self.angular_z = 0
        self.target_linear_x = 0
        self.target_angular_z = 0
        
        self.attitude = self.bot.get_imu_attitude_data()

        atexit.register(self.stop)
        
    def forward_kinematics(self, left_wheel_velocity, right_wheel_velocity):
        # Calculate linear and angular velocity
        x = (self.wheel_radius / 2) * (left_wheel_velocity + right_wheel_velocity)
        z = (self.wheel_radius / self.wheel_base) * (right_wheel_velocity - left_wheel_velocity)

        return x,z
    
    def inverse_kinematics(self, x, z):
        # Calculate left and right wheel velocities
        left = x - (self.wheel_base / 2) * z
        right = x + (self.wheel_base / 2) * z
        return left, right

    def stop(self):
        self.bot.set_motor(0,0,0,0)

    def handle_cmd_vel(self, msg: Twist):
        
        x = msg.linear.x*0.25 #/3.33
        z = msg.angular.z*0.25 #/2.0/1.685

        left, right = self.inverse_kinematics(x, z)
        #self.bot.set_car_motion(x*3.33,0,z*3.33/2.0)
        self.bot.set_car_motion(x,0,z)
        self.target_linear_x = x
        self.target_angular_z = z
       

    def ticks_callback(self, *args):

        ticks = np.array(self.bot.get_motor_encoder())
        ticks_offset = ticks - self.prev_ticks

        rps_left = (ticks_offset[LEFT]/1000.0)/tick_delay
        rps_right = (ticks_offset[RIGHT]/1000.0)/tick_delay
        self.vel_left = rps_left*math.pi*self.wheel_radius
        self.vel_right = rps_right*math.pi*self.wheel_radius
        self.linear_x, self.angular_z = self.forward_kinematics(self.vel_left, self.vel_right)

        #self.get_logger().info(f"rpm: ({rps_left*60},{rps_right*60}), vtarget: ({self.target_linear_x},{self.target_angular_z}), vactual: ({self.linear_x},{self.angular_z})")
        
        self.prev_ticks = ticks
        


def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    rclpy.spin(node)
    rclpy.shutdown()

