#include <ros/ros.h>
#include <vector>
#include <iomanip>

#include <std_msgs/Float64.h>

#include <arm_move/Commands/InstructionCommand.h>

#include <arm_move/Source/KinematicsSource.h>

class PathExecutor
{
private:
    ros::Publisher joint_1_pub;
    ros::Publisher joint_2_pub;
    ros::Publisher joint_3_pub;
    ros::Publisher joint_4_pub;
    ros::Publisher joint_5_pub;
    ros::Publisher joint_6_pub;
    
    ros::Publisher gripper_actuator;

    // Sleep time between publishing commands
    double sleep_time;
    // Dimenstions of the manipulator
    double dimensions[6];
    // Time in which the gripper actuates
    double gripper_actuation;

    // Executes the instructions form the buffer
    void execute_path(std::vector<InstructionCommand> instruction_buffer, std::vector<std::vector<double>> full_path_points, int start_index, int end_index);
    // Close or open the gripper to a certain angle
    void actuate_gripper(double angle);

    // Publish the rotations for the joints
    void publish_joints(double *joints);

    // Copy elements from a std vector to a classic one
    void copy_state(double *where, std::vector<double> from);
public:
    // Get the parameters from the param server and initializes the topics
    PathExecutor(ros::NodeHandle *nh);
    
    // Start the execution of the path, if a loop index exists start an infinite loop
    void execute_instructions(std::vector<InstructionCommand> instruction_buffer, std::vector<std::vector<double>> full_path_points, int loop_index);
};

#pragma once