import time
from rosmaster import Rosmaster
bot = bot = Rosmaster(car_type=0, com="/dev/ttyUSB0")
bot.create_receive_threading()
a,_,_,_ = bot.get_motor_encoder()
bot.set_motor(2,0,0,0)
time.sleep(1)
b,_,_,_ = bot.get_motor_encoder()
bot.set_motor(0,0,0,0)

print(a,b)
print(b-a)