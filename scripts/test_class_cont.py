import RPi.GPIO as GPIO
from hardware_continous_servo import ContinousServo
import time

#GPIO.setmode(GPIO.BOARD)
#GPIO.setup(12, GPIO.OUT)
#servo = GPIO.PWM(12, 50)
#servo.start(0)
#time.sleep(1)

#servo.ChangeDutyCycle(6)
#time.sleep(10)

cont_servo = ContinousServo(12)
cont_servo.to_angle(90)
time.sleep(2)
cont_servo.to_angle(0)
time.sleep(2)

cont_servo.servo.stop()

#servo.stop()
GPIO.cleanup()
