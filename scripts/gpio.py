import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

GPIO.setup(33, GPIO.OUT)

servo = GPIO.PWM(33, 50)

servo.start(0)
time.sleep(1)
print("servo start")

velocity_deg_sec = 157
stop_duty = 7
start_duty_cw = 6
start_duty_ccw = 8

current_angle_degrees = 0

def go_to_angle(angle_degrees):
	global current_angle_degrees
	angle_to_move = angle_degrees - current_angle_degrees
	current_angle_degrees = current_angle_degrees + angle_to_move
	if(angle_to_move < 0):
		move_angle(abs(angle_to_move), 0)
	else:
		move_angle(angle_to_move, 1)

def move_angle(angle_degrees, sens_rotation):
	#current_angle_degrees = current_angle_degrees + angle_degrees
	on_time = angle_degrees/velocity_deg_sec
	print(on_time)
	if(sens_rotation == 1):
		servo.ChangeDutyCycle(start_duty_cw)
	else:
		servo.ChangeDutyCycle(start_duty_ccw)
	time.sleep(on_time)
	servo.ChangeDutyCycle(stop_duty)

go_to_angle(2)
print(current_angle_degrees)
time.sleep(2)
#go_to_angle(4)
#print(current_angle_degrees)
#time.sleep(0.01)
go_to_angle(90)
print(current_angle_degrees)
time.sleep(2)
#go_to_angle(26)
#print(current_angle_degrees)
#time.sleep(1)


#move_angle(12)
#time.sleep(0.01)
#go_to_angle(13)
#time.sleep(0.01)
#go_to_angle(14)


#print("Full speed?")
#servo.ChangeDutyCycle(2)
#time.sleep(3)

#print("slower")
#servo.ChangeDutyCycle(6)
#time.sleep(1)

#print("stop")
#servo.ChangeDutyCycle(7)
#time.sleep(1)

#print("ccw")
#servo.ChangeDutyCycle(6)
#time.sleep(10)

#print("cw")
#servo.ChangeDutyCycle(6)
#time.sleep(4)

#duty = 2
#while(duty < 12):
#	servo.ChangeDutyCycle(duty)
#	time.sleep(1)
#	duty = duty + 1

#servo.ChangeDutyCycle(2)
#time.sleep(1)
#servo.ChangeDutyCycle(0)
#time.sleep(1)

servo.stop()
GPIO.cleanup()
print("done")
