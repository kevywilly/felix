#!/usr/bin/python
import time
import atexit

from felix.common.rosmaster import Rosmaster
from felix.common.settings import settings
from felix.common.kinematics import Kinematics


speed = 0
bot = Rosmaster(car_type=2, com="/dev/ttyUSB0", debug=False)
bot.create_receive_threading()

def stop():
    bot.set_car_motion(0,0,0)

def get_ticks():
    tl,_,tr,_ = bot.get_motor_encoder()
    return (tl,tr)

atexit.register(stop)

def calibrate_linear():
    print("--------\ncalibrating linear velocity\n")
    count: float = 0.0
    avg_adj_factor = 1
    max_rpm = 0.0
    min_rpm = 1000.0
    max_vel = 0.0
    min_vel = 1.0
    for j in range(5,25, 1):
        speed = j/100.0
        for i in range(5):
            bot.set_car_motion(speed,0.0,0.0)
            t0 = time.time()
            tl0,tr0 = get_ticks()
            time.sleep(1)
            tl1,tr1 = get_ticks()
            elapsed_time = time.time() - t0
            rpm = Kinematics.calc_rpm(tl1-tl0, elapsed_time, 360)
            rps = rpm/60.0
            vx,vy,vz = Kinematics.calc_velocity(rps,rps, settings.Robot.wheel_base, settings.Robot.wheel_radius)
            adj_factor = speed/vx if speed > 0 else 0
            if count > 0:
                max_vel = vx if vx > max_vel else max_vel
                min_vel = vx if vx < min_vel else min_vel
                max_rpm = rpm if rpm > max_rpm else max_rpm
                min_rpm = rpm if rpm < min_rpm else min_rpm
                avg_adj_factor = ((avg_adj_factor*count)+adj_factor)/(count+1)
                #print(f"ticks: {tl1-tl0}, rpm: {int(rpm)}, rps: {rps}, speed: {speed}, vx: {vx}, ratio: {adj_factor}, adj: {adj_factor}")
            
            count = count + 1

    print(f"\n---------\nrpm_min_max: {min_rpm},{max_rpm}, vel_min_max: {min_vel},{max_vel}, avg_adj_factor: {avg_adj_factor}")


def calibrate_angular():
    print("--------\ncalibrating angular velocity\n")
    count: float = 0.0
    avg_adj_factor = 1
    max_rpm = 0.0
    min_rpm = 1000.0
    max_vel = 0.0
    min_vel = 1.0
    for j in range(5,100,5):
        speed = j/100.0
        for i in range(5):
            bot.set_car_motion(0,0.0,speed)
            t0 = time.time()
            tl0,tr0 = get_ticks()
            time.sleep(1)
            tl1,tr1 = get_ticks()
            elapsed_time = time.time() - t0
            rpmL = Kinematics.calc_rpm(tl1-tl0, elapsed_time)
            rpmR = Kinematics.calc_rpm(tr1-tr0, elapsed_time)
            rpsR = rpmR/60.0
            rpsL = rpmL/60.0

            vx,vy,vz = Kinematics.calc_velocity(rpsL,rpsR, settings.Robot.wheel_base, settings.Robot.wheel_radius)

            adj_factor = speed/vz if vz > 0 else 1

            if count > 0:
                max_vel = vz if vz > max_vel else max_vel
                min_vel = vz if vz < min_vel else min_vel
                max_rpm = rpmL if rpmL > max_rpm else max_rpm
                min_rpm = rpmL if rpmL < min_rpm else min_rpm
                avg_adj_factor = ((avg_adj_factor*count)+adj_factor)/(count+1)
                # print(f"rpm: {int(rpmL)}, rps: {rpsL}, speed: {speed}, vz: {vz}, ratio: {adj_factor}, adj: {adj_factor}")
            
            count = count + 1

    print(f"\n---------\nrpm_min_max: {min_rpm},{max_rpm}, vel_min_max: {min_vel},{max_vel}, avg_adj_factor: {avg_adj_factor}")

def test_rpm():
    
    bot.set_motor(100,0,0,0)
    time.sleep(2)
 
    t = time.time()
    ticks1, _,_,_ = bot.get_motor_encoder()
    time.sleep(0.1)
    ticks2, _,_,_ = bot.get_motor_encoder()

    print(ticks1,ticks2)

#calibrate_linear()
calibrate_angular()

#test_rpm()