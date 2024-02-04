from pydantic import BaseModel, computed_field
import math
from typing import List
from felix.motion.kinematics import Kinematics

class Robot(BaseModel):
    yaboom_port: str = '/dev/myserial'
    wheel_radius: float
    wheel_base: float
    track_width: float
    max_rpm: int
    gear_ratio: float 
    fov: int = 160

    @computed_field
    @property
    def wheel_circumference(self) -> float:
        return 2*math.pi*self.wheel_radius
    
    @computed_field
    @property
    def wheel_angles(self) -> List[float]:
        return [math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4]
    
    @computed_field
    @property
    def ticks_per_meter(self) -> int:
        return int(self.encoder_resolution/self.wheel_circumference)
            
    
    @computed_field
    @property
    def robot_circumference(self) -> float:
        return math.pi*(self.wheel_base+self.track_width)
    

    @computed_field
    @property
    def ticks_per_robot_rotation(self) -> int:
        return int(self.encoder_resolution/self.robot_circumference)

    @computed_field
    @property
    def max_wheel_angular_velocity(self) -> float:
        return self.max_rpm*(2*math.pi)/60
    

    @computed_field
    @property
    def encoder_resolution(self) -> float:
        return 2.0/self.gear_ratio
    
    @computed_field
    @property
    def max_robot_linear_velocity(self) -> float:
        return Kinematics.calculate_robot_velocity(
            rpm_front_left = self.max_wheel_angular_velocity,
            rpm_front_right = self.max_wheel_angular_velocity,
            rpm_rear_left = self.max_wheel_angular_velocity,
            rpm_rear_right = self.max_wheel_angular_velocity,
            L = self.wheel_base,
            W = self.track_width,
            R = self.wheel_radius
        )
    
    @computed_field
    @property
    def max_robot_angular_velocity(self) -> float:
        return Kinematics.calculate_robot_velocity(
            rpm_front_left = self.max_wheel_angular_velocity,
            rpm_front_right = -self.max_wheel_angular_velocity,
            rpm_rear_left = self.max_wheel_angular_velocity,
            rpm_rear_right = -self.max_wheel_angular_velocity,
            L = self.wheel_base,
            W = self.track_width,
            R = self.wheel_radius
        )
    
