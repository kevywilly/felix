import rclpy
from felix_motion.scripts.rosmaster import Rosmaster
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from rclpy.node import Node
from felix.scripts.settings import settings
from felix_interfaces.msg import MotionData
import felix.scripts.image_utils as image_utils
from typing import Optional

import time
import atexit

import numpy as np

class MotionNode(Node):
    def __init__(self):
        super().__init__("motion_node", parameter_overrides=[])
        self.bot = Rosmaster(car_type=2, com="/dev/ttyUSB0")
        self.bot.create_receive_threading()
        self.create_subscription(Twist, "/cmd_vel", self.handle_cmd_vel, 10)
        self.create_subscription(Image, settings.Topics.raw_video, self.image_callback, 10)
        self.motion_publisher = self.create_publisher(MotionData, '/motion', 10)
        self.tick_timer = self.create_timer(1.0, self.ticks_callback)

        self.prev_twist = Twist()
        self.twist = Twist()
        self.image: Optional[Image] = None
        
        self.prev_ticks = np.array(self.bot.get_motor_encoder())
        self.prev_time = time.time()
        self.rpms = [0,0,0,0]
        self.speeds = [0,0,0,0]
        

        self.attitude = self.bot.get_imu_attitude_data()

        atexit.register(self.stop)
        
    def image_callback(self, msg: Image):
        self.image = msg

    def stop(self):
        self.bot.set_car_motion(0, 0, 0)

    def print_speeds(self):
        for i in range(4):
            self.get_logger().info(f'm{i}: rpm: {self.rpms[i]} v: {self.speeds[i]}')
    
    def ticks_callback(self, *args):

        ticks = np.array(self.bot.get_motor_encoder())
        time_now = time.time()

        ticks_offset = ticks - self.prev_ticks
        elapsed_time = time_now - self.prev_time

        rpms = [0,0,0,0]
        speeds = [0,0,0,0]
        bad = False
        for i in range(4):
            rpm = (ticks_offset[i] / 720.0)/(elapsed_time / 60.0)
            speeds[i] = rpm * settings.Robot.wheel_radius*2*3.14/60000.0
            rpms[i] = int(rpm)

            if rpms[i] > 1000:
                bad = True

        if not bad:
            self.rpms = rpms
            self.speeds = speeds
        
        self.prev_ticks = ticks
        self.prev_time = time_now

        # self.print_speeds()
    
    def handle_cmd_vel(self, msg: Twist):
        
        twist = Twist()
        twist.linear.x = msg.linear.x * settings.Motion.linear_velocity_multiple
        twist.linear.y = msg.linear.y * settings.Motion.linear_velocity_multiple
        twist.linear.z = msg.angular.z * settings.Motion.angular_velocity_multiple

        self.prev_twist = self.twist
        self.twist = twist

        self.bot.set_car_motion(twist.linear.x, twist.linear.y, twist.angular.z)

        if self.prev_twist and self.image:
            if self.prev_twist != self.twist:
                m = MotionData()
                m.v0 = self.prev_twist
                m.v1 = self.twist
                m.image = self.image
                self.motion_publisher.publish(m)
        

        #self.get_logger().debog(f"set_motion: {(twist.linear.x,twist.linear.y, twist.angular.z)}")


def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    rclpy.spin(node)
    rclpy.shutdown()

