#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Dense>

#include <iostream>
#include <fstream>

#include <arm_move/Source/KinematicsSource.h>

#include <arm_move/Source/Cube.h>
#include <arm_move/Commands/PlanCommand.h>
#include <arm_move/Commands/InstructionCommand.h>


class Workarea
{
private:
    std::vector<PlanCommand> plan_buffer;
    std::vector<InstructionCommand> instruction_buffer;
    std::vector<std::vector<double>> full_path_points;

    double dimensions[6];

    double z_offcet;

    double link_radius = 1.5;

    std::vector<Cube> forbidden_zones;
    std::ifstream forbidden_zones_file;
    std::string forbidden_zones_path;

    // Check if the points are inside the work area
    bool check_workspace_limits();
    // Self collision test
    bool check_self_forbidden_colide();

    // Check if a point is inside a capsule 
    bool check_point_inside_capsule(Eigen::Vector3d start_point, Eigen::Vector3d end_point, Eigen::Vector3d check_point, double radius);

    // Copy elements from a classic vector to a classic one 
    void copy_state(double *where, double *from);
    void copy_state(double *where, std::vector<double> from);
public:
    // Construct a new Workarea:: Workarea object and get the parameters
    Workarea(ros::NodeHandle *nh);

    void set_buffers( std::vector<PlanCommand> _plan_buffer, std::vector<InstructionCommand> _instruction_buffer, std::vector<std::vector<double>> _full_path_points);

    // Read the coordinates of the forbidden zones from the file
    bool read_forbidden_zones();
    // Check if the points are NOT in he forbidden zones and if they are in the workarea
    bool check_workarea();
};

