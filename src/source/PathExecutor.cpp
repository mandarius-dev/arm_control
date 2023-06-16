#include <arm_move/Source/PathExecutor.h>

/**
 * @brief  Get the parameters from the param server and initializes the topics
 * 
 * @param[in] nh node handle
 */
PathExecutor::PathExecutor(ros::NodeHandle *nh)
{
    ROS_INFO_STREAM("PE: Path executor initialized.");
    std::string topic_name;

    nh->getParam("sleep_time", sleep_time);

    std::vector<double> dim;
    nh->getParam("dimensions", dim);
    copy_state(dimensions, dim);

    nh->getParam("gripper_actuation_time", gripper_actuation);

    nh->getParam("joint_1_comand", topic_name);
    joint_1_pub = nh->advertise<std_msgs::Float64>(topic_name, 100);
    nh->getParam("joint_2_comand", topic_name);
    joint_2_pub = nh->advertise<std_msgs::Float64>(topic_name, 100);
    nh->getParam("joint_3_comand", topic_name);
    joint_3_pub = nh->advertise<std_msgs::Float64>(topic_name, 100);
    nh->getParam("joint_4_comand", topic_name);
    joint_4_pub = nh->advertise<std_msgs::Float64>(topic_name, 100);
    nh->getParam("joint_5_comand", topic_name);
    joint_5_pub = nh->advertise<std_msgs::Float64>(topic_name, 100);
    nh->getParam("joint_6_comand", topic_name);
    joint_6_pub = nh->advertise<std_msgs::Float64>(topic_name, 100);

    pub_pose = nh->advertise<geometry_msgs::Pose>("current_pose", 100);

    nh->getParam("gripper_actuator", topic_name);
    gripper_actuator = nh->advertise<std_msgs::Float64>(topic_name, 100);
}

/**
 * @brief Start the execution of the path, if a loop index exists start an infinite loop
 * 
 * @param[in] instruction_buffer arm instructions buffer
 * @param[in] full_path_points all the points of the path to be followed
 * @param[in] loop_index where in the instruction buffer the loop starts, if it exists
 */
void PathExecutor::execute_instructions(std::vector<InstructionCommand> instruction_buffer, std::vector<std::vector<double>> full_path_points, int loop_index)
{
    ROS_INFO_STREAM("PE: Starting movement:");
    if(loop_index != -1)
    {
        execute_path(instruction_buffer, full_path_points, 0, loop_index);
        while(ros::ok())
            execute_path(instruction_buffer, full_path_points, loop_index, instruction_buffer.size());
    }
    else
        execute_path(instruction_buffer, full_path_points, 0, instruction_buffer.size());
}

/**
 * @brief Executes the instructions form the buffer
 * 
 * @param[in] instruction_buffer arm instructions buffer
 * @param[in] full_path_points all the points of the path to be followed
 * @param[in] start_index where in the instruction buffer to START executing
 * @param[in] end_index where in the instruction buffer to END the execution
 */
void PathExecutor::execute_path(std::vector<InstructionCommand> instruction_buffer, std::vector<std::vector<double>> full_path_points, int start_index, int end_index)
{
    int index = start_index;

    while(ros::ok() && index < end_index)
    {
        if(instruction_buffer[index].get_name() == "PAUSE")
        {
            ROS_INFO_STREAM("PE: Pausing.");
            ros::Duration(instruction_buffer[index].get_arg()).sleep();
        }

        if(instruction_buffer[index].get_name() == "GRIPPER")
        {
            ROS_INFO_STREAM("PE: Gripping.");
            actuate_gripper(instruction_buffer[index].get_arg());
        }
        
        if(instruction_buffer[index].get_name() == "MOVE")
        {
            ROS_INFO_STREAM("PE: Moving.");
            double joints[6], tcp[6], pose[6];
            for (int i = instruction_buffer[index].get_from(); ros::ok() && i <= instruction_buffer[index].get_to(); i++)
            {
                std::copy(full_path_points[i].begin(), full_path_points[i].end(), tcp);

                if(!instruction_buffer[index].is_joint_space())
                {
                    //publish_pose(tcp);
                    Kinematics::ik(dimensions, tcp, joints);
                    publish_joints(joints);
                }
                else
                    publish_joints(tcp);
                    //Kinematics::dk(dimensions, tcp, pose);
                    //publish_pose(pose);

        
                geometry_msgs::Pose pose;

                pose.position.x = full_path_points[i][0];
                pose.position.y = full_path_points[i][1];
                pose.position.z = full_path_points[i][2];
                pose.orientation.x = full_path_points[i][3];
                pose.orientation.y = full_path_points[i][4];
                pose.orientation.z = full_path_points[i][5];
                pose.orientation.w = 0;

                pub_pose.publish(pose);

                ros::Duration(sleep_time).sleep();
            }
        }

        index++;
    }
}

void PathExecutor::publish_pose(double *tcp)
{
    geometry_msgs::Pose pose;

    pose.position.x = tcp[0];
    pose.position.y = tcp[1];
    pose.position.z = tcp[2];
    pose.orientation.x = tcp[3];
    pose.orientation.y = tcp[4];
    pose.orientation.z = tcp[5];
    pose.orientation.w = 0;

    pub_pose.publish(pose);
}

/**
 * @brief Close or open the gripper to a certain angle
 * 
 * @param[in] angle how open or should the gripper be
 */
void PathExecutor::actuate_gripper(double angle)
{
    std_msgs::Float64 float_msg;
    float_msg.data = angle;

    gripper_actuator.publish(float_msg);
    ros::Duration(gripper_actuation).sleep();
}

/**
 * @brief Publish the rotations for the joints
 * 
 * @param[in] joints rotations of the joints
 */
void PathExecutor::publish_joints(double *joints)
{
    std_msgs::Float64 float_msg;

    float_msg.data = -joints[0];
    joint_1_pub.publish(float_msg);
    float_msg.data = joints[1];
    joint_2_pub.publish(float_msg);
    float_msg.data = -joints[2];
    joint_3_pub.publish(float_msg);
    float_msg.data = -joints[3];
    joint_4_pub.publish(float_msg);
    float_msg.data = joints[4];
    joint_5_pub.publish(float_msg);  
    float_msg.data = -joints[5];
    joint_6_pub.publish(float_msg);
}

/**
 * @brief Copy elements from a std vector to a classic one 
 * 
 * @param[in] from std vector
 * @param[out] where classic vector
 */
void PathExecutor::copy_state(double *where, std::vector<double> from)
{
    where[0] = from[0];
    where[1] = from[1];
    where[2] = from[2];
    where[3] = from[3];
    where[4] = from[4];
    where[5] = from[5];
}