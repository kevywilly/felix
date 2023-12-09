#!/usr/bin/python3
import atexit
from felix_controller.scripts.rosmaster import Rosmaster

RADIANS=0.0174533

bot = Rosmaster(car_type=2, com="/dev/ttyUSB0")
bot.create_receive_threading()

def stop():
    bot.set_car_motion(0,0,0)

atexit.register(stop)


_,_,yaw = bot.get_imu_attitude_data()
degrees = 360
radians = degrees * 0.0174533


def map(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

def turn_degrees(degrees, vel):
    
    yaw = bot.get_imu_attitude_data()[2]

    bot.set_car_motion(0,0,vel if degrees >=0 else -vel)
    delta = 0
    print(f"turn_degrees: {degrees}")

    while True:
        time.sleep(0.01)
        new_yaw = bot.get_imu_attitude_data()[2]
        delta += abs(new_yaw-yaw)
        print(delta)
        yaw = new_yaw
        if delta >= degrees:
            break
      

        

def turn_degrees2(degrees, vel):
    
    _,_,yaw = bot.get_imu_attitude_data()
    new_yaw = yaw
    target = int(yaw) + int(degrees)
    target = target - 360*int(target/360) # make sure <= 360
    target = target - 179*int(target/179) # make sure between -179 and 179
    dir = 1 if degrees >= 0 else -1
    
    bot.set_car_motion(0,0,vel*dir)
    target_delta = target-yaw
    print(f"turn_degrees: {degrees}, target_delta")
    while True:
        time.sleep(0.002)
        _,_,new_yaw = bot.get_imu_attitude_data()
        delta = new_yaw-yaw
        if delta >= target_delta:
            break
        yaw = new_yaw

        print(f"old: {yaw} new: {new_yaw}, target: {target}")
        

        
       
    print("done")
    bot.set_car_motion(0,0,00)

#while True:
#    roll,pitch,yaw = bot.get_imu_attitude_data()
#    print(f"roll: {roll}, pitch: {pitch}, yaw: {yaw}")

import time
time.sleep(1)
turn_degrees(180, 0.3)