#include <iostream>
#include <vector>

/**
 * Structure class that stores the processed command read from the file
 * 
 */
class PlanCommand
{
private:
    std::string name;

    double pose[6];
    double argument;
    std::vector<std::vector<double>> spline_pose;
public:
    PlanCommand(std::string _name, double x, double y, double z, double rx, double ry, double rz);
    PlanCommand(std::string _name, double _argument);
    PlanCommand(std::string _name, std::vector<std::vector<double>> _spline_pose);
    PlanCommand(std::string _name, std::vector<double> _spline_pose);

    std::string get_name();
    double* get_pose();
    int get_argument();
    std::vector<std::vector<double>> get_spline_pose();

    void add_spline_pose(std::vector<double> spline_pose);
};

PlanCommand::PlanCommand(std::string _name, double x, double y, double z, double rx, double ry, double rz): name(_name)
{
    pose[0] = x;
    pose[1] = y;
    pose[2] = z;
    pose[3] = rx;
    pose[4] = ry;
    pose[5] = rz;
}

PlanCommand::PlanCommand(std::string _name, double _argument): name(_name), argument(_argument)
{

}

PlanCommand::PlanCommand(std::string _name, std::vector<std::vector<double>> _spline_pose): name(_name), spline_pose(_spline_pose)
{

}

PlanCommand::PlanCommand(std::string _name, std::vector<double> _spline_pose): name(_name)
{
    spline_pose.push_back(_spline_pose);
}

std::string PlanCommand::get_name()
{
    return name;
}

double* PlanCommand::get_pose()
{
    return pose;
}

int PlanCommand::get_argument()
{
    return argument;
}

std::vector<std::vector<double>> PlanCommand::get_spline_pose()
{
    return spline_pose;
}

void PlanCommand::add_spline_pose(std::vector<double> _spline_pose)
{
    spline_pose.push_back(_spline_pose);
}

#pragma once