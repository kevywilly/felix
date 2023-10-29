#!/usr/bin/python3

bot = Rosmaster()
help(bot)


bot.set_motor(50,0,0,0)
bot.get_motion_pid()
bot.set_car_motion(0.5,0,0)
bot.set_car_type

bot.get