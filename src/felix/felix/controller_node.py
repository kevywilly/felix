from felix.motion.mecanum_robot import MecanumRobot
import rclpy

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from felix.config.settings import settings
from felix.motion.rosmaster import Rosmaster
from felix.motion.kinematics import Kinematics

import math
import time
import atexit
import numpy as np

TWIST_ZERO = Twist()

velocity_factor = 2*settings.robot.wheel_radius*math.pi
tick_delay = 1.0
max_wheel_velocity = 0.29
LEFT=0
RIGHT=2


NAV_FREQUENCY = 100 #HZ


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller", parameter_overrides=[])
        
        self.mecanum = MecanumRobot(
            wheel_radius=settings.robot.wheel_radius, 
            L = settings.robot.wheel_base,
            W = settings.robot.track_width,
            max_rpm = settings.robot.max_rpm
        )

        self.max_linear = self.mecanum.max_linear_velocity()
        self.max_angular = self.mecanum.max_angular_velocity()

        self.bot = Rosmaster(car_type=2, com=settings.robot.yaboom_port)
        self.bot.create_receive_threading()

        # Subscriptions
        self.create_subscription(Odometry, "/cmd_nav", self._handle_nav, 10)
        self.create_subscription(Twist, "/cmd_vel", self._handle_cmd_vel, 10)

        # Publishers
        self.motion_publisher = self.create_publisher(Twist, "/motion_data", 5)
        

        # Timers
        self.nav_timer = self.create_timer(1.0/NAV_FREQUENCY, self._nav_timer_callback)
        self.motion_timer = self.create_timer(0.5, self._motion_timer_callback)

        # Vars
        self.nav_delta = 0
        self.nav_delta_target = 0
        self.abort_move = False

        self._reset_nav()
        
        atexit.register(self.stop)

        self.get_logger().info("Controller is UP!")
        self.get_logger().info(f"MAX Linear: {self.max_linear}")
        self.get_logger().info(f"MAX Angular: {self.max_angular}")

    def _apply_velocity(self,x,y,z):

        wheel_velocities = np.array(self.mecanum.forward_kinematics(x*self.max_linear, y*self.max_linear, math.tanh(z)*self.max_angular))
        
        
        fl,fr,rl,rr = self.mecanum.mps_to_motor_power(wheel_velocities)*100

        self.get_logger().info(f'{fl},{fr},{rl},{rr}')

        
        self.bot.set_motor(fl,rl,fr,rr)

    def _motion_timer_callback(self, *args):
        t = Twist()
        x,y,z = self.bot.get_motion_data()
        t.linear.x = x
        t.linear.y = y
        t.angular.z = z
        self.motion_publisher.publish(t)

    def _nav_timer_callback(self, *args):
        if not self.nav_running:
            return
        
        if self.abort_move or (self.nav_delta >= self.nav_delta_target) or ((time.time() - self.nav_start_time) > 5):
            self.abort_move = False
            self._reset_nav()
            
        else:
            new_yaw = self.bot.get_imu_attitude_data()[2]
            self.nav_delta += abs(new_yaw-self.nav_yaw)
            self.nav_yaw = new_yaw

    def _reset_nav(self):
        self.nav_running = False
        self.nav_delta = 0
        self.nav_delta_target = 0
        self.nav_yaw = self.bot.get_imu_attitude_data()[2]
        self.nav_start_time = time.time()
        
        
    def _start_nav(self, target):
        self.nav_delta = 0
        self.nav_delta_target = abs(target)
        self.nav_start_time = time.time()
        self.nav_running = True

    # Handlers
         
    def _handle_cmd_vel(self, msg: Twist):
    
        self.abort_move = True

        x = msg.linear.x 
        y = msg.linear.y 
        z = msg.angular.z

        self._apply_velocity(x,y,z)

    def _handle_nav(self, msg: Odometry):

        vx = msg.twist.twist.linear.x 
        vz = msg.twist.twist.angular.z
        
        degrees = int(math.degrees(msg.pose.pose.orientation.z))

        self.get_logger().info(f"Got nav request: deg: {degrees}, vx:{vx}, vz: {vz}")

        self._reset_nav()

        if degrees == 0:
            return
        
        self._start_nav(degrees)
        self._apply_velocity(vx,0,vz)
    
    def stop(self):
        self.bot.set_motor(0,0,0,0)

   

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

