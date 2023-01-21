#!/usr/bin/env python
# This file is part of the distribution (https://github.com/bts7960)
# Copyright (c) 2022 Jorge Paise Ribeiro.
# 
# This program is free software: you can redistribute it and/or modify  
# it under the terms of the GNU General Public License as published by  
# the Free Software Foundation, version 3.
#
# This program is distributed in the hope that it will be useful, but 
# WITHOUT ANY WARRANTY; without even the implied warranty of 
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License 
# along with this program. If not, see <http://www.gnu.org/licenses/>.

import sys, tty, termios, os, readchar
from bts7960 import rpi_JGB37545

def getch():
   ch = readchar.readchar()
   return ch

def printscreen(mt, curr_power):
   os.system('clear')
   print("========== Status ==========")
   print("Current power:  ", curr_power)
   print("RPM:  ", mt.get_motor_speed())
   print("Speed:  ", mt.get_wheel_speed())
   print("========== Menu ==========")
   print("a: Increment Power")
   print("s: Decrement Power")
   print("g: Update Status")
   print("x: Stop Motor")
   print("q: Quit")

def main():
   curr_power = 0
   
   mt = rpi_JGB37545.motor(hall_sensor=12, 
            bts_L_EN=27, 
            bts_R_EN=13, 
            bts_L_PWM=6, 
            bts_R_PWM=5, 
            wheel_diameter = 0.1)
   
   """mt = rpi_JGB37545.motor(hall_sensor=20, 
            bts_L_EN=27, 
            bts_R_EN=16, 
            bts_L_PWM=26, 
            bts_R_PWM=19, 
            wheel_diameter = 0.1)
   """      
   mt.set_motor_power(curr_power)
   printscreen(mt, curr_power)
         
   while True:
      char = getch()

      if (char == "a"):
         curr_power = curr_power + 0.1
         if curr_power > 1:
            curr_power = 1

      if (char == "s"):
         curr_power = curr_power - 0.1
         if curr_power < -1:
            curr_power = -1
            
      if (char == "x"):
         curr_power = 0

      if (char == "g"):
         mt.get_motor_speed()

      if (char == "q"):
         mt.stop()
         print("Program Terminated")
         break
      elif (char != ""):
         mt.set_motor_power(curr_power)
         printscreen(mt, curr_power)
         
      char = ""

   del mt
   return 0

if __name__ == '__main__':
    sys.exit(main())  # next section explains the use of sys.exit