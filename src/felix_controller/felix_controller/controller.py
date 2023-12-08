import rclpy
from typing import Optional
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from rclpy.node import Node
from felix.common.settings import settings
from felix_controller.scripts.rosmaster import Rosmaster

import math
import time
import atexit
import numpy as np

TWIST_ZERO = Twist()

velocity_factor = 2*settings.Robot.wheel_radius*math.pi
tick_delay = 1.0
max_wheel_velocity = 0.29
LEFT=0
RIGHT=2


# Wheel layout
#  0 2
#  1 3

mechanum_matrix = np.array(
            [
                [1,1,1,1],
                [-1,1,1,-1],
                [-1,-1,1,1]
            ]
        )

max_linear_x = 0.33
max_linear_y = 0.33
max_angular_z = 2.5

def _limit_velocity(value, max_value) -> float:
    return value*min(value, max_value)/value if value != 0 else 0

class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller", parameter_overrides=[])
        self.bot = Rosmaster(car_type=2, com="/dev/ttyUSB0")
        self.bot.create_receive_threading()
        self.create_subscription(Twist, "/cmd_vel", self.handle_cmd_vel, 10)
        self.tick_timer = self.create_timer(tick_delay, self.ticks_callback)
        
        self.wheel_radius = settings.Robot.wheel_radius
        self.robot_radius = settings.Robot.wheel_base/2.0
        self.track_width = settings.Robot.track_width
        self.wheel_base = settings.Robot.wheel_base
        self.wheel_circumference = 2*math.pi*self.wheel_radius
        self.wheel_angles = [math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4]
        self.ticks_per_meter = 1000/self.wheel_circumference

        
        self.prev_ticks = np.array(self.bot.get_motor_encoder())
        
        self.velocity = (0,0,0)
        self.target_velocity = (0,0,0)

        atexit.register(self.stop)

    def calculate_robot_velocity(self, np_wheel_velocities):
     
        x = sum(np_wheel_velocities*mechanum_matrix[0])/4.0
        y = sum(np_wheel_velocities*mechanum_matrix[1])/4.0
        z = sum(np_wheel_velocities*mechanum_matrix[2])/(2*(self.wheel_base + self.track_width))
        
        return (x,y,z)
    
    def stop(self):
        self.bot.set_motor(0,0,0,0)


    def handle_cmd_vel(self, msg: Twist):
        
        x = msg.linear.x * max_linear_x
        y = msg.linear.y * max_linear_y
        z = msg.angular.z * max_angular_z

        self.bot.set_car_motion(x,y,z)
        self.target_velocity = (x,y,z)
        
    def ticks_callback(self, *args):

        ticks = np.array(self.bot.get_motor_encoder())
        ticks_offset = ticks - self.prev_ticks
        rps = ticks_offset/(1000.0 * tick_delay)

        wheel_velocities = rps*self.wheel_circumference
        
        self.velocity = self.calculate_robot_velocity(wheel_velocities)
        if(sum(self.velocity) != 0):
            self.get_logger().info(f"RPM: {self.velocity}") 
        
        self.prev_ticks = ticks
        


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

