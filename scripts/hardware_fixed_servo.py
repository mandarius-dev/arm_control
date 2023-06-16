from adafruit_motor import servo

from hardware_servo import Servo

class FixedServo(Servo):
	def __init__(self, min_deg, max_deg, pca_board, zero_position_deg, servo_num):
		super().__init__(servo_num)

		self.min = min
		self.max = max
		self.zero_position_deg = zero_position_deg

		self.servo = servo.Servo(pca_board.channels[self.pin], actuation_range=max_deg-min_deg)

	def to_angle(self, angle_deg):
		self.servo.angle = angle_deg + self.zero_position_deg

	def range_map(self, angle):
		return angle_deg + self.zero_position_deg
