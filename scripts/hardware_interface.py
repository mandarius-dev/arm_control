import time 
import RPi.GPIO as GPIO

from math import degrees
from board import SCL, SDA

import busio

from adafruit_motor import servo 
from adafruit_pca9685 import PCA9685

from hardware_fixed_servo import FixedServo
from hardware_continous_servo import ContinousServo

class  HardwareInterface:
	def __init__(self, actuation_range):
		self.actuation_range = actuation_range

		i2c = busio.I2C(SCL, SDA)

		self.pca = PCA9685(i2c)
		self.pca.frequency = 100

		self.servos = []
		self.init_servos()

	def init_servos(self):
		for i in range(len(self.actuation_range)):
			if(self.actuation_range[i][0] == 0):
				print(i, "fixed")
				self.servos.append(FixedServo(self.actuation_range[i][1], self.actuation_range[i][2], self.pca, self.actuation_range[i][3], i))
			else:
				print(i, "cont")
				self.servos.append(ContinousServo(self.actuation_range[i][1]))

	def actuate_servos(self, angels_rad):
		for i in range(len(angels_rad)):
			#print(i, ": ",degrees(angels_rad[i]))
			self.servos[i].to_angle(degrees(angels_rad[i]))

	def actuate_servo(self, servo_num, angle_rad):
		print(type(self.servos[servo_num]))
		self.servos[servo_num].to_angle(degrees(angle_rad))
