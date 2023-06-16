#include "arm_move/Source/Planner.h"

/**
 * @brief Construct a new Planner:: Planner object and gets the parameters from the param server
 * 
 * @param nh 
 */
Planner::Planner(ros::NodeHandle *nh)
{
    ROS_INFO_STREAM("PL: Planner initialized.");

    std::string pkg_name;
    nh->getParam("pkg_name", pkg_name);

    std::string pkg_path = ros::package::getPath(pkg_name);

    std::string path;
    nh->getParam("command_syntax_path", path);
    command_syntax_path = pkg_path + path;

    nh->getParam("command_path", path);
    command_path = pkg_path + path;

    command_syntax_file.open(command_syntax_path);
    command_file.open(command_path);
}

/**
 * @brief Prints the command in the console
 * 
 */
void Planner::print_command_syntax()
{
    for(int i=0;i<command_syntax.size();i++)
        ROS_INFO_STREAM("PL:" << command_syntax[i].get_name() << ":" << command_syntax[i].get_first_argument() << ":" << command_syntax[i].get_second_argument());
}

/**
 * @brief Read the command syntax from the file 
 * 
 */
void Planner::read_command_syntax()
{
    std::string line;
    int index, count;
    while(getline(command_syntax_file, line))
    {
        std::istringstream iss(line);
        std::vector<std::string> result(std::istream_iterator<std::string>{iss},std::istream_iterator<std::string>());

        std::string first_argument = "";
        std::string second_argument = "";

        if(result.size() >= 2)
            first_argument = result[1];

        if(result.size() >= 3)
            second_argument = result[2] + " " + std::to_string(result.size() - 3);

        Command command(result[0], first_argument, second_argument);
        command_syntax.push_back(command);
    }
}

/**
 * @brief Read the actual commands from the file
 * 
 * @return true all the commands have been read 
 * @return false a command has an invalid syntax
 */
bool Planner::read_command()
{
    int line_index = 1;
    std::string line;
    bool corect;
    while(getline(command_file, line))
    {
        std::istringstream iss(line);
        std::vector<std::string> result(std::istream_iterator<std::string>{iss},std::istream_iterator<std::string>());

        std::string first_argument = "";
        std::string second_argument = "";

        if(result.size() >= 2)
            first_argument = result[1];

        for(int i=2;i<result.size();i++)
            second_argument += result[i] + " ";
        
        try
        {
            corect = check_syntax(result[0], first_argument, second_argument);
            
            if(spline_start && spline_command_number == 0)  
                spline_start = false;
            
            if(corect && spline_start)
                spline_command_number--;

            if(corect)
                add_plan_command(first_argument, second_argument, line_index);

        }
        catch(ArmException e)
        {
            std::string error = e.what() + " At line " + std::to_string(line_index);
            ROS_ERROR_STREAM(error.c_str());
            return false;
        }

        line_index++; 
    }

    print_plan_buffer();
    return true;
}

/**
 * @brief Check if the command has a valid syntax
 * 
 * @param[in] command_name 
 * @param[in] command_first_argument 
 * @param[in] command_second_argument 
 * @return true command has a coret syntax
 * @return false invalid syntax
 */
bool Planner::check_syntax(std::string command_name, std::string command_first_argument, std::string command_second_argument)
{
    for(int i=0;i<command_syntax.size();i++)
    {
        if(check_command_name(command_name))
        {
            if(command_name == "LOOP" && circle_start)
                throw ArmException("Syntax Error: Circle intermidiat pose not specified.");

            if(command_name == "LOOP" && command_first_argument == "" && command_second_argument == "")
                return true;

            if(check_command_first_argument(command_name, command_first_argument))    
            {
                if((command_first_argument == "OPEN" || command_first_argument == "CLOSE") && command_second_argument == "")
                    return true;

                std::istringstream iss(command_second_argument);
                std::vector<std::string> result(std::istream_iterator<std::string>{iss},std::istream_iterator<std::string>());

                std::string second_argument = result[0] + " " + std::to_string(result.size()-1);

                if(check_command_second_argument(command_name, command_first_argument, second_argument))
                {
                    if(command_first_argument != "CIRCLE" && circle_start)
                        throw ArmException("Syntax error: End pose of circle not specified.");

                    if(command_first_argument == "CIRCLE" && result[0] == "POSE_END" && !circle_start)
                        throw ArmException("Syntax Error: Circle intermidiat pose not specified.");

                    if(command_first_argument == "CIRCLE" && result[0] == "POSE_INTERMEDIARY" && !circle_start)
                        circle_start = true;

                    if(command_first_argument == "CIRCLE" && result[0] == "POSE_END" && circle_start)
                        circle_start = false;

                    if(command_first_argument == "SPLINE" && result[0] == "POSE" && !spline_start)
                        throw ArmException("Syntax Error: Too many spline commands.");

                    if(spline_start && command_first_argument == "PAUSE")
                        return true;

                    if(spline_start && (command_first_argument != "SPLINE" || result[0] != "POSE"))
                        throw ArmException("Syntax Error: Different command found in spline command.");

                    if(command_first_argument == "SPLINE" && result[0] == "COMMAND")
                    {
                        spline_start = true;
                        spline_command_number = atoi(result[1].c_str()) + 1;
                    }

                    return true;
                }
                else
                    throw ArmException("Syntax Error: Invalid argument.");
            }
            else
                throw ArmException("Syntax Error: Argument not found.");
        }
        else
            throw ArmException("Syntax Error: Command not found.");
    }
    throw ArmException("Syntax Error: Could not read syntax checker.");
}

/**
 * @brief Search if the command name is in the command syntax buffer
 * 
 * @param[in] command_name 
 * @return true name in the syntax buffer
 * @return false name not found in the syntax buffer
 */
bool Planner::check_command_name(std::string command_name)
{
    for(int i=0;i<command_syntax.size();i++)
    {
        if(command_name == command_syntax[i].get_name())
            return true;
    }
    return false;
}

/**
 * @brief Search if the first argument is in the command syntax buffer
 * 
 * @param command_name 
 * @param command_first_argument 
 * @return true is in the syntax buffer
 * @return false NOT in the syntax buffer
 */
bool Planner::check_command_first_argument(std::string command_name, std::string command_first_argument)
{
    for(int i=0;i<command_syntax.size();i++)
        if(command_name == command_syntax[i].get_name() && command_first_argument == command_syntax[i].get_first_argument())
            return true;
    return false;
}

/**
 * @brief Search if the second argument is in the command syntax buffer
 * 
 * @param command_name 
 * @param command_first_argument 
 * @return true is in the syntax buffer
 * @return false NOT in the syntax buffer
 */
bool Planner::check_command_second_argument(std::string command_name, std::string command_first_argument, std::string command_second_argument)
{
    for(int i=0;i<command_syntax.size();i++)
        if(command_name == command_syntax[i].get_name() && command_first_argument == command_syntax[i].get_first_argument(), command_second_argument == command_syntax[i].get_second_argument())
            return true;
    return false;
}

/**
 * @brief Add the command in the command buffer
 * 
 * @param name name of the arm command
 * @param argument argument of the command
 * @param line_index in case a loop command is read
 */
void Planner::add_plan_command(std::string name, std::string argument, int line_index)
{
    if(name == "SPLINE" && argument.find("COMMAND") != std::string::npos)
        return;

    if(name == "SPLINE" && argument.find("POSE") != std::string::npos)
    {
        double pose[6];
        extract_command_data(argument, pose);
        std::vector<double> vec_pose(pose, pose + sizeof(pose)/sizeof(pose[0]));

        if(plan_buffer.size() == 0 || plan_buffer[plan_buffer.size()-1].get_name() != "SPLINE")
            plan_buffer.push_back(PlanCommand("SPLINE", vec_pose));
        else
            plan_buffer[plan_buffer.size()-1].add_spline_pose(vec_pose);

        return;
    }

    if(name == "CIRCLE")
    {
        double pose[6];
        extract_command_data(argument, pose);
        std::vector<double> vec_pose(pose, pose + sizeof(pose)/sizeof(pose[0]));

        if(plan_buffer[plan_buffer.size()-1].get_name() != "CIRCLE")
            plan_buffer.push_back(PlanCommand("CIRCLE", vec_pose));
        else
            plan_buffer[plan_buffer.size()-1].add_spline_pose(vec_pose);

        return;
    }

    if(name == "CLOSE")
    {
        plan_buffer.push_back(PlanCommand("GRIPPER", 0.0));

        return;    
    }

    if(name == "OPEN")
    {
        plan_buffer.push_back(PlanCommand("GRIPPER", 1.0));

        return;    
    }

    if(name == "PAUSE")
    {
        double simple_arg;
        extract_command_data(argument, simple_arg);

        plan_buffer.push_back(PlanCommand(name, simple_arg));

        return;
    }

    if(name == "" && argument == "")
    {
        loop_index = line_index-1;

        return;
    }

    double pose[6];
    extract_command_data(argument, pose);

    plan_buffer.push_back(PlanCommand(name, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]));

    return;
}

/**
 * @brief Extract the data from a string data type
 * 
 * @param[in] argument string with the data
 * @param[out] simple_arg argument extracted, one variable
 */
void Planner::extract_command_data(std::string argument, double &simple_arg)
{
    std::istringstream iss(argument);
    std::vector<std::string> result(std::istream_iterator<std::string>{iss},std::istream_iterator<std::string>());

    simple_arg = std::stod(result[1]);
}

/**
 * @brief Extract the data from a string data type
 * 
 * @param[in] argument string with the data
 * @param[out] simple_arg argument extracted, array
 */
void Planner::extract_command_data(std::string argument, double *pose)
{
    std::istringstream iss(argument);
    std::vector<std::string> result(std::istream_iterator<std::string>{iss},std::istream_iterator<std::string>());

    for(int i=1; i<result.size(); i++)
        pose[i-1] = std::stod(result[i]);
}

/**
 * @brief Print the plan buffer for debug
 * 
 */
void Planner::print_plan_buffer()
{
    ROS_INFO_STREAM("PL: Plan buffer:");
    for(int i=0;i<plan_buffer.size();i++)
    {
        if(plan_buffer[i].get_name() == "SPLINE" || plan_buffer[i].get_name() == "CIRCLE")
        {
            ROS_INFO_STREAM("PL: " << plan_buffer[i].get_name());

            std::vector<std::vector<double>> spline_pose = plan_buffer[i].get_spline_pose();

            for(int j=0;j<spline_pose.size();j++)
                ROS_INFO_STREAM("PL: " << "\t" << spline_pose[j][0] << " " << spline_pose[j][1] << " " << spline_pose[j][2] << " " << spline_pose[j][3] << " " << spline_pose[j][4] << " " << spline_pose[j][5]);
        }

        if(plan_buffer[i].get_name() == "PTP_JOINT" || plan_buffer[i].get_name() == "PTP_COORD" || plan_buffer[i].get_name() == "LINE")
        {
            ROS_INFO_STREAM("PL: " << plan_buffer[i].get_name());

            double* pose;
            pose = plan_buffer[i].get_pose();

            ROS_INFO_STREAM("PL: " << "\t" << pose[0] << " " << pose[1] << " " << pose[2] << " " << pose[3] << " " << pose[4] << " " << pose[5]);
        }

        if(plan_buffer[i].get_name() == "GRIPPER" || plan_buffer[i].get_name() == "PAUSE")
        {
            ROS_INFO_STREAM("PL: " << plan_buffer[i].get_name());

            ROS_INFO_STREAM("PL: " << "\t" << plan_buffer[i].get_argument());
        }
    }
}

int Planner::get_loop_index()
{
    return loop_index;
}

std::vector<PlanCommand> Planner::get_plan_buffer()
{
    return plan_buffer;
}