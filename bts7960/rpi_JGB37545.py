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
#
# This library was made to support BTS7960 controller + JGB37-545B-12V-319 motor
# 
# references used:
#  - https://custom-build-robots.com/raspberry-pi-robot-cars/big-rob-motor-driver-bts7960b/8155
#  - https://forums.raspberrypi.com/viewtopic.php?f=37&t=119476&hilit=cycle#p812499

import time
import RPi.GPIO as GPIO
from threading import Timer, Lock

MAX_COUNT_MOVING_AVERAGE = 20
PWM_MAX = 100

class motor():
    def __init__(self, hall_sensor=17, bts_L_EN=13, bts_R_EN=19, bts_L_PWM=6, bts_R_PWM=12, wheel_diameter = 0.1):
        self._WHEEL_CIRCUMFERENCE = wheel_diameter * 3.141592654
        self._HS = hall_sensor
        self._L_EN = bts_L_EN
        self._R_EN = bts_R_EN
        self._L_PWM = bts_L_PWM
        self._R_PWM = bts_R_PWM
        
        GPIO.setmode(GPIO.BCM)
        #GPIO.setwarnings(False)
        
        # setting up input & output ports
        GPIO.setup(self._HS, GPIO.IN, pull_up_down = GPIO.PUD_UP)
        GPIO.setup(self._L_EN, GPIO.OUT)
        GPIO.setup(self._R_EN, GPIO.OUT)
        GPIO.setup(self._L_PWM, GPIO.OUT)
        GPIO.setup(self._R_PWM, GPIO.OUT)
        
        # attach callback to HS trigger
        GPIO.add_event_detect(self._HS, GPIO.FALLING, callback = self.got_hs_pulse) 

        # setting PWM output settings
        self._motor_pwm_l = GPIO.PWM(self._L_PWM, 10000)
        self._motor_pwm_r = GPIO.PWM(self._R_PWM, 10000)
        
        # start duty cycle
        self._motor_pwm_l.start(0)
        self._motor_pwm_r.start(0)
        
        # initialize duty cycle
        self._motor_pwm_l.ChangeDutyCycle(0)
        self._motor_pwm_r.ChangeDutyCycle(0)

        # initialize variables
        self.motor_speed = 0.0
        self.wheel_speed = 0.0
        self.wheel_distance = 0.0
        self._sensor_timer = 0
        self._dynamic_moving_average = [0] * ( MAX_COUNT_MOVING_AVERAGE + 1 )
        self._dynamicMovingAverageCount = 0 
        
        self._stop_run = False
        self.lock = Lock()
        #self._set_motor_mode("forward")
        
        # initialize timer for timeouting hall sensor readings
        self._update_clock()

    def __del__(self):
        if not self._stop_run:
            self.stop()

    def stop(self):
        self.lock.acquire()
        self._stop_run = True
        GPIO.remove_event_detect(self._HS)
        self._timer.cancel()
        self.lock.release()        
        GPIO.output(self._L_EN, False)
        GPIO.output(self._R_EN, False)
        GPIO.cleanup()        
        

    def _update_clock(self):
        self.lock.acquire()
        if self._stop_run:
            self.lock.release()
            return
        # timeout without readings
        if self._sensor_timer > 0:
            self._sensor_timer = self._sensor_timer - 1
        else:
            self.motor_speed = 0.0
        #print(f"Motor speed: {self.motor_speed} wheel_speed: {self.wheel_speed} wheel_distance: {self.wheel_distance}")
        self.lock.release()

        self._timer = Timer(0.1, self._update_clock)
        self._timer.start()
        
    def got_hs_pulse(self, channel):
        """
        got_hs_pulse is called when a hall sensor trigger is received

        :param channel: channel descriptions 
        :return: none
        """ 
        ctime = time.time()
        self._dynamic_moving_average[MAX_COUNT_MOVING_AVERAGE] = ctime
        deltaTime = ctime - self._dynamic_moving_average[MAX_COUNT_MOVING_AVERAGE - self._dynamicMovingAverageCount]
        
        for i in range(MAX_COUNT_MOVING_AVERAGE):
            self._dynamic_moving_average[i] = self._dynamic_moving_average[i+1]       
        
        if deltaTime == 0.0:
            speed = 0.0
        else:
            speed = self._dynamicMovingAverageCount / deltaTime

        if self._dynamicMovingAverageCount < MAX_COUNT_MOVING_AVERAGE:
            self._dynamicMovingAverageCount = self._dynamicMovingAverageCount + 1
        
        self.lock.acquire()
        self._sensor_timer = MAX_COUNT_MOVING_AVERAGE # use MAX_COUNT as number of timeout samples
        self.motor_speed = speed * 60 / (18.8*16)  # rpm  - 18.8:1 gearbox ratio, 16 PPR hall sensor 
        self.wheel_speed = (self.motor_speed / 60) * self._WHEEL_CIRCUMFERENCE # m/s
        self.wheel_distance += self.wheel_speed * deltaTime # meters
        self.lock.release()


    def _set_motor_mode(self, mode: str):
      if mode == "reverse":
         GPIO.output(self._L_EN, True)
         #GPIO.output(self._R_EN, False) # not working on my configuration
         GPIO.output(self._R_EN, True) 
      elif mode == "forward":
         #GPIO.output(self._L_EN, False) # not working on my configuration
         GPIO.output(self._L_EN, True) 
         GPIO.output(self._R_EN, True)
      elif mode == "stop":
         GPIO.output(self._L_EN, False)
         GPIO.output(self._R_EN, False)

    def set_motor_power(self, power: float):
        """
        set_motor_power sets the motor power

        :param power: float from -1.0 to 1.0
        :return: none
        """ 
        if power < 0:
            self._set_motor_mode("reverse")
            pwm = -int(PWM_MAX * power)
            if pwm > PWM_MAX:
                pwm = PWM_MAX
            self._motor_pwm_l.ChangeDutyCycle(pwm)
            self._motor_pwm_r.ChangeDutyCycle(0)	  
            
        elif power > 0:
            self._set_motor_mode("forward")
            pwm = int(PWM_MAX * power)
            if pwm > PWM_MAX:
                pwm = PWM_MAX
            self._motor_pwm_l.ChangeDutyCycle(0)
            self._motor_pwm_r.ChangeDutyCycle(pwm)
        else:
            # Stop both pwm output
            self._set_motor_mode("stop")
            self._motor_pwm_l.ChangeDutyCycle(0)
            self._motor_pwm_r.ChangeDutyCycle(0)

    def get_motor_speed(self) -> int:
        """
        get_motor_speed get the current motor rpms 

        :return: motor rotations [rpm]
        """ 
        return int(self.motor_speed)

    def get_wheel_speed(self) -> float:
        """
        get_wheel_speed get the current wheel speed

        :return: wheel speed [m/s]
        """ 
        return self.wheel_speed
    
    def get_wheel_odom(self) -> float:
        """
        get_wheel_odom get the distance travelled by the wheel

        :return: wheel distance [m]
        """ 
        return self.wheel_distance