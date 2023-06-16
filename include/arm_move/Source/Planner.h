#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <iterator>
#include <sstream>

#include <arm_move/Commands/Command.h>
#include <arm_move/Commands/PlanCommand.h>

#include <arm_move/Exceptions/ArmException.h>

#define IGNORE_CHARACTER "@"

class Planner
{
private:
    // Path and stream of the command syntax file 
    std::string command_syntax_path;
    std::ifstream command_syntax_file;
    // Path and stream of the commands file
    std::string command_path;
    std::ifstream command_file;

    // Buffer of the command syntax
    std::vector<Command> command_syntax;    
    // Buffer of the plan buffer
    std::vector<PlanCommand> plan_buffer;

    bool spline_start = false;
    bool circle_start = false;

    int spline_command_number;
    int loop_index = -1;

    // Check if the command has a valid syntax
    bool check_syntax(std::string command_name, std::string command_first_argument, std::string command_second_argument);

    // Search if the command name is in the command syntax buffer
    bool check_command_name(std::string command_name);
    // Search if the first argument is in the command syntax buffer
    bool check_command_first_argument(std::string command_name, std::string command_first_argument);
    // Search if the second argument is in the command syntax buffer
    bool check_command_second_argument(std::string command_name, std::string command_first_argument, std::string command_second_argument);

    // Add the command in the command buffer
    void add_plan_command(std::string name, std::string argument, int line_index);
    
    // Extract the data from a string data type
    void extract_command_data(std::string argument, double &simple_arg);
    void extract_command_data(std::string argument, double *pose);

public:
    // onstruct a new Planner:: Planner object and gets the parameters from the param server
    Planner(ros::NodeHandle *nh);

    // Read the command syntax from the file 
    void read_command_syntax();
    // Read the actual commands from the file
    bool read_command();

    // Prints the command in the console
    void print_command_syntax();
    void print_plan_buffer();

    std::vector<PlanCommand> get_plan_buffer();
    int get_loop_index();
};

#pragma once