from felix.motion.mecanum_robot import MecanumRobot
import rclpy

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from felix.config.settings import settings
from felix.motion.rosmaster import Rosmaster
from felix.motion.kinematics import Kinematics
from felix.utils.system import SystemUtils

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

def scale(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller", parameter_overrides=[])

        self.max_linear = settings.robot.mecanum.max_linear_velocity
        self.max_angular = settings.robot.mecanum.max_angular_velocity

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
        self.stop()
        self._reset_nav()
        
        atexit.register(self.stop)

        self.get_logger().info("Controller is UP!")
        self.get_logger().info(f"MAX Linear: {self.max_linear}")
        self.get_logger().info(f"MAX Angular: {self.max_angular}")

    def _power_cap(self, power):
        return min(power,100)*settings.robot.motor_power_factor
    
    def _apply_velocity(self,x,y,z):

        vx = scale(x, -1.0, 1.0, -self.max_linear, self.max_linear)
        vy = scale(y, -1.0, 1.0, -self.max_linear, self.max_linear)
        omega = scale(z, -1.0, 1.0, -self.max_angular, self.max_angular)

        self.get_logger().info(f'Scaled Velocity: {vx},{vy},{omega}')
        self.bot.set_car_motion(vx,vy,omega)
        return
        vx = x*self.max_linear
        vy = y*self.max_linear
        omega = math.tanh(z*self.max_angular/2.0)

        V: np.ndarray = self.mecanum.forward_kinematics(vx,vy,omega)
        
        fl,fr,rl,rr = (self.mecanum.mps_to_motor_power(V) * 100 * settings.robot.motor_power_factor).astype(int)

        self.get_logger().info(f'{fl},{fr},{rl},{rr}')
        
        self.bot.set_motor(
            self._power_cap(fl),
            self._power_cap(rl),
            self._power_cap(fr),
            self._power_cap(rr)
        )

    def _motion_timer_callback(self, *args):
        t = Twist()
        try:
            x,y,z = self.bot.get_motion_data()
            t.linear.x = x
            t.linear.y = y
            t.angular.z = z
        except:
            pass
        
        self.motion_publisher.publish(t)

    def _nav_timer_callback(self, *args):

        if not SystemUtils.wifi_up():
            self._reset_nav()
            return

        if not self.nav_running:
            return
        
        if (self.nav_delta >= self.nav_delta_target):
            self._apply_velocity(settings.NAV_LINEAR_VELOCITY,0,0)
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
    
        self._reset_nav()

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

        #if degrees == 0:
        #    return
        
        self._start_nav(degrees)
        self._apply_velocity(vx,0,vz)
    
    def stop(self):
        self._reset_nav()
        self.bot.set_motor(0,0,0,0)

   

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.stop()
    rclpy.shutdown()

