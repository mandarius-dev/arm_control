
import RPi.GPIO as GPIO
import time

from hardware_servo import Servo

class ContinousServo(Servo):

	def __init__(self, pin):
		super().__init__(pin)

		self.frequency = 50

		self.servo = GPIO.PWM(self.pin, self.frequency)

		# 157 - old
		self.velocity_deg_sec = 157

		self.stop_duty = 7
		self.start_duty_cw = 6
		self.start_duty_ccw = 8

		self.current_angle_deg = 0

	def to_angle(self, angle_deg):
		angle_to_move = angle_deg - self.current_angle_deg
		self.current_angle_deg = self.current_angle_deg + angle_to_move

		if(angle_to_move < 0):
			self.move_angle(abs(angle_to_move), 0)
		else:
			self.move_angle(angle_to_move, 1)

	def move_angle(self, angle_deg, rotation_direction):
		on_time = angle_deg/self.velocity_deg_vec

		if(rotation_direction == 1):
			self.servo.ChangeDutyCycle(self.start_duty_cw)
		else:
			self.servo.ChangeDutyCycle(self.start_duty_ccw)
