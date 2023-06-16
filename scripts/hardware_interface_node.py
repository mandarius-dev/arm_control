 #!/usr?bin/env python3

import rospy
import message_filters
from std_msgs.msg import Float64

from hardware_interface import HardwareInterface

actuation_range = [[1,33], [0,-90, 90, 90], [0,-155, 155, 155], [1,32], [0,-90, 90, 90], [1,12], [0, -90, 0, 0]]

hd = HardwareInterface(actuation_range)

def callback(joint1, joint2, joint3, joint4, joint5, joint6):
	angels = []
	#angels = [joint1.data, joint2.data, joint3.data, joint4.data, joint5.data, joint6.data]
	angels.append(joint1.data)
	angels.append(joint2.data)
	angels.append(joint3.data)
	angels.append(joint4.data)
	angels.append(joint5.data)
	angels.append(joint6.data)
	print(angels)
	hd.actuate_servos(angels)

def gripper_callback(angle_rad):
	hd.actuate_servo(6, angle_rad.data)
	print("gripper")

if __name__ == '__main__':

	rospy.init_node('hardware_interface_node')

	joint1 = message_filters.Subscriber('/arm_joint_controll/joint_1_position_controller/command', Float64)
	joint2 = message_filters.Subscriber('/arm_joint_controll/joint_2_position_controller/command', Float64)
	joint3 = message_filters.Subscriber('/arm_joint_controll/joint_3_position_controller/command', Float64)
	joint4 = message_filters.Subscriber('/arm_joint_controll/joint_4_position_controller/command', Float64)
	joint5 = message_filters.Subscriber('/arm_joint_controll/joint_5_position_controller/command', Float64)
	joint6 = message_filters.Subscriber('/arm_joint_controll/joint_6_position_controller/command', Float64)

	rospy.Subscriber('/arm_joint_controll/joint_griper_left_position_controller/command', Float64, gripper_callback)

	#print(len(actuation_range))

	ts = message_filters.ApproximateTimeSynchronizer([joint1, joint2, joint3, joint4, joint5, joint6], 10, 10, allow_headerless=True)
	ts.registerCallback(callback)
	rospy.spin()
