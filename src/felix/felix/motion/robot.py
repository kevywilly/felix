import math
from abc import ABC
class Robot(ABC):
    def __init__(self,
        wheel_radius: float,
        wheel_base: float,
        track_width: float,
        max_rpm: int,
        gear_ratio: float,
        yaboom_port: str = '/dev/myserial', 
        fov: int = 160,
        motor_voltage=12
    ):
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.track_width = track_width
        self.max_rpm = max_rpm
        self.gear_ratio = gear_ratio
        self.yaboom_port = yaboom_port
        self.fov = fov
        self.motor_voltage=motor_voltage
        self.wheel_circumference = 2*math.pi*self.wheel_radius
        self.wheel_angles = [math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4]
        self.robot_circumference = math.pi*(self.wheel_base+self.track_width)
        self.encoder_resolution = 2.0/self.gear_ratio
        self.ticks_per_meter = int(self.encoder_resolution/self.wheel_circumference)
        self.ticks_per_robot_rotation = int(self.encoder_resolution/self.robot_circumference)
        self.max_wheel_angular_velocity = self.max_rpm*(2*math.pi)/60
        self.motor_power_factor = motor_voltage/12.0
        

