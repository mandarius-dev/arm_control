#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
//#include <gazebo_msgs/LinkStates.h>
#include <control_msgs/JointControllerState.h>

#include <arm_move/Commands/PlanCommand.h>
#include <arm_move/Commands/InstructionCommand.h>

#include <arm_move/Source/Cube.h>

#include <arm_move/Source/KinematicsSource.h>

class TrajectoryGeneration
{
private:
    // Distance along the trajectory between two points
    double step;
    double dimensions[6];
    // Length of the control points
    double spline_distance_ratio;

    std::vector<InstructionCommand> instruction_buffer;
    std::vector<std::vector<double>> full_path_points;

    std::fstream debug_file;
    //geometry_msgs::Pose tcp_current_pose;
    ros::Subscriber tcp_pose;
    ros::Publisher data_pub;

    // Copy values from one array to another
    void copy_state(double *where, double *from);
    void copy_state(double *where, std::vector<double> from);

    // Recursive De Casteljau algorithm
    void de_casteljaus(std::vector<Eigen::Vector3d> control_points, double t, Eigen::Vector3d &spline_point);
    /* Using the control points the point on the spline are generated using the 
        De Casteljau algorithm */
    void generate_spline_points(std::vector<Eigen::Vector3d> control_points, std::vector<Eigen::Vector3d> &spline_points);
    /* Generates the control points of a given segment taking into account 
        the geometry of the previous segment, if there are intermediate point and 
        the geometry of the next segment */
    void generate_control_points(Eigen::Vector3d start_point, bool has_start_geometry, Eigen::Vector3d start_geometry, Eigen::Vector3d end_point, bool has_end_geometry, Eigen::Vector3d end_geometry, std::vector<Eigen::Vector3d> &control_points);
    // Generates points on the spline that passes through CERTAIN points
    void generate_path_length(std::vector<Eigen::Vector3d> path_points, std::vector<Eigen::Vector3d> &spline_points);
    // Generates points on the spline that passes through the ALL the given points
    void generate_spline(std::vector<Eigen::Vector3d> path_points, std::vector<Eigen::Vector3d> &spline_points);

    // Generates the orientation on the spline that passes through ALL of the given orientation
    void generate_rotations(std::vector<Eigen::Vector3d> path_rotations, std::vector<Eigen::Vector3d> &spline_rotations);
    /* Generates the orientation on the spline that passes through CERTAIN, 
        of the given orientation*/
    void generate_rotation_length(Eigen::Vector3d start_rotation, Eigen::Vector3d end_rotation, std::vector<Eigen::Vector3d> &spline_rotations);

    // Get the curent pose of the manipulator
    //void tcp_pose_callback(const gazebo_msgs::LinkStates &pose);

public:
    // Construct a new Trajectory Generation:: Trajectory Generation object and gets the paramters from the param server
    TrajectoryGeneration(ros::NodeHandle *nh);

    // Generate a line trajectory 
    void line(double *pose_start, double *pose_end, std::vector<std::vector<double>> &full_path_points);
    // Generates a circle trajectory
    void circle(double *pose_start, double *pose_intermediate, double *pose_end, std::vector<std::vector<double>> &full_path_points);
    // Generates a spline through the given points and with the given rotations
    void spline(std::vector<Eigen::Vector3d> path_points, std::vector<Eigen::Vector3d> path_rotations, std::vector<std::vector<double>> &full_path_points);

    // Linear interpolation between two sets of rotations
    void ptp_joints(double *state_start, double *state, std::vector<std::vector<double>> &full_path_points);
    // Linera interpolation between two points
    void ptp_coord(double *state_start, double *pose, std::vector<std::vector<double>> &full_path_points);

    // Generate the instruction buffer depending on the plan buffer
    bool generate_instruction_buffer(std::vector<PlanCommand> plan_buffer);

    // Print the path buffer
    void print_full_path_points();
    // Print the instruction buffer
    void print_instruction_buffer();

    // Print the values of an array
    void print_values(std::string msg, double *values, int n);

    std::vector<InstructionCommand> get_instruction_buffer();
    std::vector<std::vector<double>> get_full_path_points();
};

#pragma once
