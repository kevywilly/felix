import rclpy

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
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

NAV_FREQUENCY = 100 #HZ

def _limit_velocity(value, max_value) -> float:
    return value*min(value, max_value)/value if value != 0 else 0


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("calibration", parameter_overrides=[])
        
        self.bot = Rosmaster(car_type=2, com="/dev/ttyUSB0")
        self.bot.create_receive_threading()
        self.wheel_radius = settings.Robot.wheel_radius
        self.robot_radius = settings.Robot.wheel_base/2.0
        self.track_width = settings.Robot.track_width
        self.wheel_base = settings.Robot.wheel_base
        self.wheel_circumference = 2*math.pi*self.wheel_radius
        self.wheel_angles = [math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4]
        self.ticks_per_meter: int = int(settings.Robot.encoder_resolution/self.wheel_circumference)
        self.robot_circumference = math.pi*(self.wheel_base+self.track_width)
        self.ticks_per_robot_rotation: int = int(settings.Robot.encoder_resolution/self.robot_circumference)

        self.get_max_wheel_velocity()
        

        atexit.register(self.stop)

    def get_max_wheel_velocity(self):

        self.get_logger().info("Detecting Max Wheel Velocity")
        self.get_logger().info(f"Encoder Resolution: {settings.Robot.encoder_resolution}")
        self.get_logger().info(f"Wheel Radius: {settings.Robot.wheel_radius}")
        self.get_logger().info(f"Wheel Circumference (2piR): {self.wheel_circumference}")
        self.get_logger().info(f"Ticks Per Meter (resolution/circumference): {self.ticks_per_meter}")

        
        self.bot.set_car_motion(1,0,0)

        seconds = 5
        time.sleep(seconds)
        np0 = np.array(self.bot.get_motor_encoder())
        

        time.sleep(seconds)
        np1 = np.array(self.bot.get_motor_encoder())
       
        self.bot.set_car_motion(0,0,0)


        ticks_per_second = (np1-np0)/seconds
        meters_per_second = self.ticks_per_meter/ticks_per_second
        max_meters_per_second = max(meters_per_second)
        rotating_meters_per_second = max_meters_per_second*np.array([1,1,-1.0,-1.0])
        x,_,_ = self.calculate_robot_velocity(meters_per_second)
        _,_,z = self.calculate_robot_velocity(rotating_meters_per_second)

        self.get_logger().info("============================================")
        self.get_logger().info(f"ticks_per_second: {ticks_per_second}")
        self.get_logger().info(f"meters_per_second: {meters_per_second}")
        self.get_logger().info(f"max_ticks_per_second: {max_meters_per_second}")
        self.get_logger().info(f"max_wheel_velocity: {max_wheel_velocity}")
        self.get_logger().info(f"max_linear_velocity: {x}")
        self.get_logger().info(f"max_angular_velocity: {z}")
        

    def calculate_robot_velocity(self, np_wheel_velocities):
     
        x = sum(np_wheel_velocities*mechanum_matrix[0])/4.0
        y = sum(np_wheel_velocities*mechanum_matrix[1])/4.0
        z = sum(np_wheel_velocities*mechanum_matrix[2])/(2*(self.wheel_base + self.track_width))
        # degrees = z*seconds*57.2958
        return (x,y,z)
    
    def stop(self):
        self.bot.set_car_motion(0,0,0)

    

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    rclpy.spin_once(node)
    rclpy.shutdown()

