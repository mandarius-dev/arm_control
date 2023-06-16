
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time

from board import SCL, SDA
import busio

# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
# You can optionally provide a finer tuned reference clock speed to improve the accuracy of the
# timing pulses. This calibration will be specific to each board and its environment. See the
# calibration.py example in the PCA9685 driver.
# pca = PCA9685(i2c, reference_clock_speed=25630710)
pca.frequency = 100

# To get the full range of the servo you will likely need to adjust the min_pulse and max_pulse to
# match the stall points of the servo.
# This is an example for the Sub-micro servo: https://www.adafruit.com/product/2201
# servo7 = servo.Servo(pca.channels[7], min_pulse=580, max_pulse=2350)
# This is an example for the Micro Servo - High Powered, High Torque Metal Gear:
#   https://www.adafruit.com/product/2307
# servo7 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2600)
# This is an example for the Standard servo - TowerPro SG-5010 - 5010:
#   https://www.adafruit.com/product/155
# servo7 = servo.Servo(pca.channels[7], min_pulse=400, max_pulse=2400)
# This is an example for the Analog Feedback Servo: https://www.adafruit.com/product/1404
# servo7 = servo.Servo(pca.channels[7], min_pulse=600, max_pulse=2500)
# This is an example for the Micro servo - TowerPro SG-92R: https://www.adafruit.com/product/169
# servo7 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2400)

# The pulse range is 750 - 2250 by default. This range typically gives 135 degrees of
# range, but the default is to use 180 degrees. You can specify the expected range if you wish:
#servo7 = servo.Servo(pca.channels[1], actuation_range=300)

servo7 = servo.Servo(pca.channels[0])
servo6 = servo.Servo(pca.channels[1])

print("angle 0")
servo6.angle = 0
time.sleep(5)

print("angle 90")
servo6.angle = 90
time.sleep(5)

#print("angle 180")
#servo6.angle = 180
#time.sleep(5)


#print("start")
#servo6.angle = 90
#time.sleep(3)

#i = 90
#print("to 50")
#while i >=50:
#	servo6.angle = i
#	i = i-1
#	time.sleep(0.04)

#print("sleep")
#time.sleep(2)

#print("to 90")
#while i <= 90:
#	servo6.angle = i
#	i = i+1
#	time.sleep(0.04)




#print("start")

# We sleep in the loops to give the servo time to move into position.
#i = 30

#for i in range(130):
#    servo6.angle = i
#    time.sleep(0.04)
#time.sleep(0.02)

#max_angle = 180
#for i in range(max_angle):
#    servo7.angle = i
#    time.sleep(0.02)

#for i in range(130):
#     servo6.angle = 130 - i
#     time.sleep(0.04)

#time.sleep(0.02)
#for i in range(max_angle):
#    servo7.angle = max_angle - i
#    time.sleep(0.02)

#for i in range(90):
#    servo6.angle = i 
#    time.sleep(0.02)

#for i in range(90):
#    servo6.angle = 90 - i
#    time.sleep(0.02)

# You can also specify the movement fractionally.
#fraction = 0.0
#while fraction < 1.0:
#   servo7.fraction = fraction
#    fraction += 0.01
#    time.sleep(0.03)

print("end of execution")

pca.deinit()
