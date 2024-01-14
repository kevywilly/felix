#!/usr/bin/python3

import time
from rosmaster import Rosmaster
from src.felix.felix.common.kinematics import Kinematics
import numpy as np
import math
import atexit
import math

bot = Rosmaster(car_type=2)
bot.create_receive_threading()

wheel_radius: float = 95.00/1000
wheel_base: float = 150.00/1000


# Example usage
wheel_velocities = [1.0, 2.0, 1.5, 2.5]  # Example wheel velocities in m/s
wheel_radius = 0.1  # Example wheel radius in meters
robot_width = 0.6   # Example robot width in meters
robot_length = 0.8  # Example robot length in meters

linear_velocity, angular_velocity = Kinematics.calculate_mecanum_velocities(wheel_velocities, wheel_radius, robot_width, robot_length)
print("Linear Velocity:", linear_velocity, "m/s")
print("Angular Velocity:", angular_velocity, "rad/s")

class VelocityCalculatorResult:
    def __init__(self, requested_velocity: float, actual_velocity: float):
        self.requested_velocity = requested_velocity
        self.actual_velocity = actual_velocity
        self.ratio = self.requested_velocity/self.actual_velocity

    def __str__(self):
        return f"requested: {self.requested_velocity}, actual: {self.actual_velocity}, ratio: {self.ratio}"

class VelocityCalculator:
    def __init__(self, ticks_per_rev: float = 2000, wheel_radius_meters = 97.0/2000.0, wheel_base = 0.23):

        self.ticks_per_rev: float = ticks_per_rev
        self.sleep_seconds: float = 1.0
        self.wheel_radius_meters = wheel_radius_meters
        self.results = []

        atexit.register(self.stop)

    def stop(self):
        bot.set_car_motion(0,0,0)
    
    def start(self):
        speeds = [0.1, .2, .3, .4, .5]

        for s in speeds:
            ar = []
            bot.set_motor(s,0,0,0)
            time.sleep(self.sleep_seconds)
            for i in range(5):
                ticks0 = np.array(bot.get_motor_encoder())
                time.sleep(self.sleep_seconds)
                ticks1 = np.array(bot.get_motor_encoder())
                diff = ticks1-ticks0
                ar.append(diff)
                rpm = (diff/self.ticks_per_rev)*60
                print(rpm)
            
            avg_ticks = np.average(np.array(ar))
            rps = (avg_ticks/self.ticks_per_rev)/self.sleep_seconds
            rpm = rps * 60.0
            
            mps = rps * 2.0*math.pi*self.wheel_radius_meters
            

            result = VelocityCalculatorResult(requested_velocity=s, actual_velocity=mps)
            print(result)
            self.results.append(result)

        bot.set_car_motion(0,0,0)

calculator = VelocityCalculator(ticks_per_rev=1000, wheel_radius_meters=95.0/2000)
calculator.start()