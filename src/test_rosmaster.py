#!/usr/bin/python3
from felix_motion.scripts.rosmaster import Rosmaster
bot = Rosmaster(car_type=2, com="/dev/ttyUSB0")

bot.set_motor(100,0,0,0)
bot.set_motor(0,0,0,0)

help(bot)


bot.set_motor(50,0,0,0)
bot.get_motion_pid()
bot.set_car_motion(0.5,0,0)
bot.set_car_type
bot.get