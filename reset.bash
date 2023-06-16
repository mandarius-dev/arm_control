rosservice call /gazebo/reset_world

rostopic pub -1 /arm_joint_controll/joint_1_position_controller/command std_msgs/Float64 0
rostopic pub -1 /arm_joint_controll/joint_2_position_controller/command std_msgs/Float64 0
rostopic pub -1 /arm_joint_controll/joint_3_position_controller/command std_msgs/Float64 0
rostopic pub -1 /arm_joint_controll/joint_4_position_controller/command std_msgs/Float64 0
rostopic pub -1 /arm_joint_controll/joint_5_position_controller/command std_msgs/Float64 0
rostopic pub -1 /arm_joint_controll/joint_6_position_controller/command std_msgs/Float64 0

rostopic pub -1 /arm_joint_controll/joint_griper_left_position_controller/command std_msgs/Float64 0
rostopic pub -1 /arm_joint_controll/joint_griper_right_position_controller/command std_msgs/Float64 0