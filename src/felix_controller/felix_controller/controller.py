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

def _limit_velocity(value, max_value) -> float:
    return value*min(value, max_value)/value if value != 0 else 0

class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller", parameter_overrides=[])
        
        self.bot = Rosmaster(car_type=2, com="/dev/ttyUSB0")
        self.bot.create_receive_threading()
        self.create_subscription(Odometry, "/cmd_nav", self.handle_cmd_nav, 10)
        self.create_subscription(Twist, "/cmd_vel", self.handle_cmd_vel, 10)
        self.create_subscription(Vector3, "/cmd_turn", self.turn, 10)
        self.create_subscription(Vector3, "/cmd_move", self.move, 10)
        self.tick_timer = self.create_timer(tick_delay, self.ticks_callback)
        self.wheel_radius = settings.Robot.wheel_radius
        self.robot_radius = settings.Robot.wheel_base/2.0
        self.track_width = settings.Robot.track_width
        self.wheel_base = settings.Robot.wheel_base
        self.wheel_circumference = 2*math.pi*self.wheel_radius
        self.wheel_angles = [math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4]
        self.ticks_per_meter: int = int(settings.Robot.encoder_resolution/self.wheel_circumference)
        self.robot_circumference = math.pi*(self.wheel_base+self.track_width)
        self.ticks_per_robot_rotation: int = int(settings.Robot.encoder_resolution/self.robot_circumference)
        self.prev_ticks = np.array(self.bot.get_motor_encoder())
        self.velocity = (0,0,0)
        self.target_velocity = (0,0,0)

        self.abort_move_or_turn = False
        
        atexit.register(self.stop)

    
    
    def move(self, msg: Vector3):
        meters = msg.x
        if meters == 0:
            return
        self.bot.set_car_motion(msg.y,0,0)
        start_ticks, _, _, _ = self.bot.get_motor_encoder()
        self.wait_ticks(start_ticks, int(self.ticks_per_meter/meters))
        self.bot.set_car_motion(0,0,0)

    def wait_ticks(self,start_ticks: int, ticks: int):
        
        ticks_count = 0
        t=0
        while ticks_count < ticks:
            if(self.abort_move_or_turn):
                self.abort_move_or_turn = False
                break
            t,_,_,_ = self.bot.get_motor_encoder()
            ticks_count = abs(t-start_ticks)
            
            # self.get_logger().info(f"t: {t}")

        # self.get_logger().info(f"ticks: {ticks}, ticks_count: {ticks_count}, start: {start}, t: {t}")
        return
    
    def handle_cmd_nav(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vz = msg.twist.twist.angular.z
        degrees = int(msg.pose.pose.orientation.z*57.2958)
        if degrees == 0:
            return
        
        delta = 0
        yaw = self.bot.get_imu_attitude_data()[2]
        self.bot.set_car_motion(vx,0,vz)
        self.get_logger().info(f"turning {degrees} degrees with velocity ({vx},0,{vz}).")
        start_time = time.time()

        while True:
            if(self.abort_move_or_turn):
                self.abort_move_or_turn = False
                break
            time.sleep(0.01)
            new_yaw = self.bot.get_imu_attitude_data()[2]
            delta += abs(new_yaw-yaw)
            
            yaw = new_yaw

            if delta >= abs(degrees):
                break

            # stop if more than 10 seconds
            if time.time() - start_time > 10:
                self.get_logger().warning(f"turn time out - operation aborted.")
                break

        self.bot.set_car_motion(0,0,0)
        

    def turn(self, msg: Vector3):
        degrees = int(msg.x)
        if degrees == 0:
            return
        vel = msg.y
        delta = 0
        yaw = self.bot.get_imu_attitude_data()[2]
        self.bot.set_car_motion(0,0,vel)
      
        self.get_logger().info(f"turning {degrees} degrees with angular_velocity {vel}.")
        start_time = time.time()
        while True:
            if(self.abort_move_or_turn):
                self.abort_move_or_turn = False
                break

            time.sleep(0.01)
            new_yaw = self.bot.get_imu_attitude_data()[2]
            delta += abs(new_yaw-yaw)
            
            yaw = new_yaw

            if delta >= degrees:
                break

            # stop if more than 10 seconds
            if time.time() - start_time > 10:
                self.get_logger().warning(f"turn time out - operation aborted.")
                break

        self.bot.set_car_motion(0,0,0)

    def calculate_robot_velocity(self, np_wheel_velocities):
     
        x = sum(np_wheel_velocities*mechanum_matrix[0])/4.0
        y = sum(np_wheel_velocities*mechanum_matrix[1])/4.0
        z = sum(np_wheel_velocities*mechanum_matrix[2])/(2*(self.wheel_base + self.track_width))
        # radians = z*seconds*57.2958
        return (x,y,z)
    
    def stop(self):
        self.bot.set_motor(0,0,0,0)


    def handle_cmd_vel(self, msg: Twist):
        
        self.abort_move_or_turn = True

        x = msg.linear.x * max_linear_x
        y = msg.linear.y * max_linear_y
        z = msg.angular.z * max_angular_z

        self.bot.set_car_motion(x,y,z)
        self.target_velocity = (x,y,z)
        
    def ticks_callback(self, *args):

        ticks = np.array(self.bot.get_motor_encoder())
        ticks_offset = ticks - self.prev_ticks
        rps = ticks_offset/(settings.Robot.encoder_resolution * tick_delay)

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

