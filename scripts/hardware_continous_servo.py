import RPi.GPIO as GPIO
import time

from hardware_servo import Servo

class ContinousServo(Servo):

	def __init__(self, pin):
		super().__init__(pin)

		print(self.pin)

		print(GPIO.getmode())

		if(GPIO.getmode() == -1):
			GPIO.setmode(GPIO.BOARD)
		else:
			if(GPIO.getmode() == 11):
				GPIO.setup(24, GPIO.OUT)
				GPIO.cleanup()
				GPIO.setmode(GPIO.BOARD) 

		GPIO.setup(self.pin, GPIO.OUT)

		self.frequency = 50

		self.servo = GPIO.PWM(self.pin, self.frequency)
		self.servo.start(0)

		self.velocity_deg_sec = 157

		self.stop_duty = 7
		self.start_duty_cw = 6
		self.start_duty_ccw = 8

		self.current_angle_deg = 0

		self.time_start = -1
		self.servo_start = False
		self.on_time = 0

	def to_angle(self, angle_deg):
		#print(self.pin, ": ",angle_deg)
		#print("angle ", angle_deg)

		angle_to_move = angle_deg - self.current_angle_deg
		self.current_angle_deg = self.current_angle_deg + angle_to_move

		#print("angle_to_move ", angle_to_move)

		if(angle_to_move < 0):
			self.move_angle(abs(angle_to_move), 0)
		else:
			self.move_angle(angle_to_move, 1)

	def move_angle(self, angle_deg, rotation_direction):
		#on_time = angle_deg/self.velocity_deg_sec

		if(not self.servo_start):
			self.time_start = time.time()
			self.on_time = angle_deg/self.velocity_deg_sec

		#print(on_time)

		if(not self.servo_start and rotation_direction == 1):
			#print("start_cw")
			self.servo.ChangeDutyCycle(self.start_duty_cw)
		if(not self.servo_start and rotation_direction == 0):
			self.servo.ChangeDutyCycle(self.start_duty_ccw)

		self.servo_start = True

		if(time.time() - self.time_start >= self.on_time):
			self.servo_start = False
			self.servo.ChangeDutyCycle(self.stop_duty)

		#print("start_sleep")
		#time.sleep(on_time)
		#print("end_sleep")
		#self.servo.ChangeDutyCycle(self.stop_duty)
