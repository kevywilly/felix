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


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller", parameter_overrides=[])
        
        self.bot = Rosmaster(car_type=2, com="/dev/ttyUSB0")
        self.bot.create_receive_threading()
        self.create_subscription(Odometry, "/cmd_nav", self.handle_nav, 10)
        self.create_subscription(Twist, "/cmd_vel", self.handle_cmd_vel, 10)
        self.nav_timer = self.create_timer(1.0/NAV_FREQUENCY, self.nav_callback)
        self.wheel_radius = settings.Robot.wheel_radius
        self.robot_radius = settings.Robot.wheel_base/2.0
        self.track_width = settings.Robot.track_width
        self.wheel_base = settings.Robot.wheel_base
        self.wheel_circumference = 2*math.pi*self.wheel_radius
        self.wheel_angles = [math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4]
        self.ticks_per_meter: int = int(settings.Robot.encoder_resolution/self.wheel_circumference)
        self.robot_circumference = math.pi*(self.wheel_base+self.track_width)
        self.ticks_per_robot_rotation: int = int(settings.Robot.encoder_resolution/self.robot_circumference)
        self.velocity = (0,0,0)

        self.abort_move = False

        self._reset_nav()
        
        atexit.register(self.stop)

    def _reset_nav(self):
        self.nav_running = False
        self.nav_delta = 0
        self.nav_delta_target = 0
        self.nav_yaw = self.bot.get_imu_attitude_data()[2]
        self.nav_start_time = time.time()
        self.bot.set_car_motion(0,0,0)

        
    def _start_nav(self, target):
        self.nav_delta = 0
        self.nav_delta_target = abs(target)
        self.nav_start_time = time.time()
        self.nav_running = True
        
    def nav_callback(self, *args):
        if not self.nav_running:
            return
        
        if self.abort_move or (self.nav_delta >= self.nav_delta_target) or ((time.time() - self.nav_start_time) > 5):
            self.abort_move = False
            self._reset_nav()
            
        else:
            new_yaw = self.bot.get_imu_attitude_data()[2]
            self.nav_delta += abs(new_yaw-self.nav_yaw)
            self.nav_yaw = new_yaw
    

    def handle_nav(self, msg: Odometry):

        vx = msg.twist.twist.linear.x * settings.Motion.max_robot_linear_velocity
        vz = msg.twist.twist.angular.z * settings.Motion.max_robot_angular_velocity
        
        degrees = int(math.degrees(msg.pose.pose.orientation.z))

        self.get_logger().info(f"Got nav request: deg: {degrees}, vx:{vx}, vz: {vz}")

        self._reset_nav()

        if degrees == 0:
            return
        
        self._start_nav(degrees)
        self.bot.set_car_motion(vx,0,vz)
        

    def calculate_robot_velocity(self, np_wheel_velocities):
     
        x = sum(np_wheel_velocities*mechanum_matrix[0])/4.0
        y = sum(np_wheel_velocities*mechanum_matrix[1])/4.0
        z = sum(np_wheel_velocities*mechanum_matrix[2])/(2*(self.wheel_base + self.track_width))
        # degrees = z*seconds*57.2958
        return (x,y,z)
    
    def stop(self):
        self.bot.set_motor(0,0,0,0)


    def handle_cmd_vel(self, msg: Twist):
        
        self.abort_move = True

        x = msg.linear.x * max_linear_x
        y = msg.linear.y * max_linear_y
        z = msg.angular.z * max_angular_z

        self.bot.set_car_motion(x,y,z)
        
    

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

