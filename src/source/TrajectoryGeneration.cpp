#include <arm_move/Source/TrajectoryGeneration.h>

/**
 * @brief Construct a new Trajectory Generation:: Trajectory Generation object and gets the paramters from the param server
 * 
 * @param nh node handle
 */
TrajectoryGeneration::TrajectoryGeneration(ros::NodeHandle *nh)
{
    ROS_INFO_STREAM("TG: Trajectory Generation initialized.");

    std::string topic_name;

    data_pub = nh->advertise<geometry_msgs::Pose>("/data", 100);

    nh->getParam("tcp_pose", topic_name);
    //tcp_pose = nh->subscribe(topic_name, 1000, &TrajectoryGeneration::tcp_pose_callback, this);

    nh->getParam("step", step);

    std::vector<double> dim;
    nh->getParam("dimensions", dim);
    copy_state(dimensions, dim);

    nh->getParam("spline_distance_ratio", spline_distance_ratio);

    std::string path = ros::package::getPath("arm_move");
    std::string file = "/data/line_debug.txt";
    path += file;

    debug_file.open(path, std::ios_base::app);
}

#pragma region Callback

/**
 * @brief Get the curent pose of the manipulator
 * 
 * @param[in] pose currnet pose of the manipulator
 */
/*void TrajectoryGeneration::tcp_pose_callback(const gazebo_msgs::LinkStates &pose)
{
    tcp_current_pose = pose.pose[pose.pose.size()-1];
}*/

#pragma endregion 

/**
 * @brief Linear interpolation between two sets of rotations
 * 
 * @param[in] state_start start rotation of the joints
 * @param[in] state_final end rotation of the joints
 * @param[out] full_path_points buffer with the point or rotation
 */
void TrajectoryGeneration::ptp_joints(double *state_start,double *state_final, std::vector<std::vector<double>> &full_path_points)
{
    double t = 0, new_state[6], tcp[6];
    std_msgs::Float64 float_msg;

    while(t <= 1 + step) 
    {
        new_state[0] = state_start[0]*(1-t) + t * state_final[0];
        new_state[1] = state_start[1]*(1-t) + t * state_final[1];
        new_state[2] = state_start[2]*(1-t) + t * state_final[2];
        new_state[3] = state_start[3]*(1-t) + t * state_final[3];
        new_state[4] = state_start[4]*(1-t) + t * state_final[4];
        new_state[5] = state_start[5]*(1-t) + t * state_final[5];  

        std::vector<double> pose(new_state, new_state + sizeof(new_state)/sizeof(new_state[0]));
        full_path_points.push_back(pose);

        t += step;
    }
}

/**
 * @brief Linera interpolation between two points 
 * 
 * @param[in] pose_start start point
 * @param[in] pose_end end point
 * @param[out] full_path_points buffer with points
 */
void TrajectoryGeneration::ptp_coord(double *pose_start, double *pose_end, std::vector<std::vector<double>> &full_path_points)
{
    double joints_final[6], joints_start[6];
    Kinematics::ik(dimensions, pose_start, joints_start);
    Kinematics::ik(dimensions, pose_end, joints_final);

    ptp_joints(joints_start, joints_final, full_path_points);
}

/**
 * @brief Copy values from one array to another
 * 
 * @param[in] from classic array
 * @param[out] where classic array
 */
void TrajectoryGeneration::copy_state(double *where, double *from)
{
    where[0] = from[0];
    where[1] = from[1];
    where[2] = from[2];
    where[3] = from[3];
    where[4] = from[4];
    where[5] = from[5];
}

/**
 * @brief Copy values from one array to another
 * 
 * @param[in] from classic array
 * @param[out] where std array array
 */
void TrajectoryGeneration::copy_state(double *where, std::vector<double> from)
{
    where[0] = from[0];
    where[1] = from[1];
    where[2] = from[2];
    where[3] = from[3];
    where[4] = from[4];
    where[5] = from[5];
}

/**
 * @brief Generate a line trajectory 
 * 
 * @param[in] pose_start 
 * @param[in] pose_end  
 * @param[out] full_path_points buffer of points
 */
void TrajectoryGeneration::line(double *pose_start, double *pose_end, std::vector<std::vector<double>> &full_path_points)
{
    Eigen::Matrix3d rot_mat_start;
    Eigen::Matrix3d rot_mat_end;
    Eigen::Matrix3d rot_mat_t;

    double qs[4], qe[4], q[4], tcp_t[6], joints[6];
    double t = 0;

    Kinematics::euler_to_quaternion(pose_start[3], pose_start[4], pose_start[5], qs);
    Kinematics::euler_to_quaternion(pose_end[3], pose_end[4], pose_end[5], qe);

    while(t <= 1 + step)
    {
        Kinematics::slerp(qs, qe, q, t);
        Kinematics::quaternion_to_euler(q, &tcp_t[3], &tcp_t[4], &tcp_t[5]);

        tcp_t[0] = (1-t)*pose_start[0] + t*pose_end[0];
        tcp_t[1] = (1-t)*pose_start[1] + t*pose_end[1];
        tcp_t[2] = (1-t)*pose_start[2] + t*pose_end[2];

        Kinematics::ik(dimensions, tcp_t, joints);

        std::vector<double> pose(tcp_t, tcp_t + sizeof(tcp_t)/sizeof(tcp_t[0]));
        full_path_points.push_back(pose);

        t += step;

        //debug_file << tcp_current_pose.position.x - tcp_t[0] << ";" << tcp_current_pose.position.y - tcp_t[1] << ";" << tcp_current_pose.position.z - tcp_t[2] << std::endl;
    }
}

/**
 * @brief Generates a circle trajectory
 * 
 * @param[in] pose_start 
 * @param[in] pose_intermediate 
 * @param[in] pose_end  
 * @param[out] full_path_points buffer of points
 */
void TrajectoryGeneration::circle(double *pose_start, double *pose_intermediate, double *pose_end, std::vector<std::vector<double>> &full_path_points)
{
    Eigen::Matrix3d rot_mat_start;
    Eigen::Matrix3d rot_mat_end;
    Eigen::Matrix3d rot_mat_t;

    double qs[4], qe[4], q[4], tcp_t[6], joints[6];
    double t = 0;
    double r = 0;

    Kinematics::euler_to_quaternion(pose_start[3], pose_start[4], pose_start[5], qs);
    Kinematics::euler_to_quaternion(pose_end[3], pose_end[4], pose_end[5], qe);

    Eigen::Vector3d v_start(pose_start[0], pose_start[1], pose_start[2]);
    Eigen::Vector3d v_intermediate(pose_intermediate[0], pose_intermediate[1], pose_intermediate[2]);
    Eigen::Vector3d v_end(pose_end[0], pose_end[1], pose_end[2]);

    Eigen::Vector3d N = (v_intermediate-v_start).cross(v_end-v_intermediate);
    N /= N.norm();

    Eigen::Vector3d v_si = v_start - v_intermediate;
    Eigen::Vector3d v_is = v_intermediate - v_start;
    Eigen::Vector3d v_ie = v_intermediate - v_end;
    Eigen::Vector3d v_ei = v_end - v_intermediate;
    Eigen::Vector3d v_se = v_start - v_end;
    Eigen::Vector3d v_es = v_end - v_start;

    double center_a = (v_ie.norm()*v_ie.norm())/(2*(v_si.cross(v_ie).norm()*v_si.cross(v_ie).norm()))*v_si.dot(v_se);
    double center_b = (v_se.norm()*v_se.norm())/(2*(v_si.cross(v_ie).norm()*v_si.cross(v_ie).norm()))*v_is.dot(v_ie);
    double center_c = (v_si.norm()*v_si.norm())/(2*(v_si.cross(v_ie).norm()*v_si.cross(v_ie).norm()))*v_es.dot(v_ei);

    Eigen::Vector3d C = center_a*v_start + center_b*v_intermediate + center_c*v_end;
    
    r = (v_start-C).norm();

    double start_end_angle = acos(((C-v_start).dot(C-v_intermediate))/((C-v_start).norm()*(C-v_intermediate).norm())) + acos(((C-v_intermediate).dot(C-v_end))/((C-v_intermediate).norm()*(C-v_end).norm()));
    
    Eigen::Vector3d S = C-v_start;
    S /= S.norm();

    Eigen::Vector3d V = N.cross(S);

    t = M_PI;

    while (t < M_PI + start_end_angle)
    {
        Kinematics::slerp(qs, qe, q, (t-M_PI)/start_end_angle);
        Kinematics::quaternion_to_euler(q, &tcp_t[3], &tcp_t[4], &tcp_t[5]);

        Eigen::Vector3d P = C + r*cos(t)*S + r*sin(t)*V;

        tcp_t[0] = P(0);
        tcp_t[1] = P(1);
        tcp_t[2] = P(2);

        std::vector<double> pose(tcp_t, tcp_t + sizeof(tcp_t)/sizeof(tcp_t[0]));
        full_path_points.push_back(pose);

        t += step;
    }

}

/**
 * @brief Recursive De Casteljau algorithm
 * 
 * @param[in] control_points  
 * @param[in] t 
 * @param[out] spline_point point on the spline
 */
void TrajectoryGeneration::de_casteljaus(std::vector<Eigen::Vector3d> control_points, double t, Eigen::Vector3d &spline_point) 
{
    if(control_points.size() == 2)
    {
        spline_point = (1-t)*control_points[0] + t*control_points[1];
        return;
    }

    std::vector<Eigen::Vector3d> new_control_points;
    for(int i = 0; i < control_points.size()-1; i++)
    {
        Eigen::Vector3d point = (1-t)*control_points[i] + t*control_points[i+1];
        new_control_points.push_back(point);
    }

    de_casteljaus(new_control_points, t, spline_point);
}

/**
 * @brief Using the control points the point on the spline are generated using the 
 * De Casteljau algorithm 
 * 
 * @param[in] control_points 
 * @param[out] spline_points spline buffer
 */
void TrajectoryGeneration::generate_spline_points(std::vector<Eigen::Vector3d> control_points, std::vector<Eigen::Vector3d> &spline_points)
{
    double t = 0;
    while(t <= 1+step)
    {
        Eigen::Vector3d point;
        de_casteljaus(control_points, t, point);
        spline_points.push_back(point);

        t += step;
    }
}

/**
 * @brief Generates the control points of a given segment taking into account 
 * the geometry of the previous segment, if there are intermediate point and 
 * the geometry of the next segment
 * 
 * @param[in] start_point 
 * @param[in] has_start_geometry exists a previous segment 
 * @param[in] start_geometry previous segment geometry
 * @param[in] end_point 
 * @param[in] has_end_geometry exists a next segment 
 * @param[in] end_geometry next segment geometry
 * @param[out] control_points 
 */
void TrajectoryGeneration::generate_control_points(Eigen::Vector3d start_point, bool has_start_geometry, Eigen::Vector3d start_geometry, Eigen::Vector3d end_point, bool has_end_geometry, Eigen::Vector3d end_geometry, std::vector<Eigen::Vector3d> &control_points)
{
    control_points.push_back(start_point);

    Eigen::Vector3d start_end_geometry = end_point - start_point;
    start_end_geometry = start_end_geometry/start_end_geometry.norm();

    double distance_start_end = (end_point - start_point).norm()/spline_distance_ratio;

    if(has_start_geometry)
    {
        start_geometry = start_geometry/start_geometry.norm();
        Eigen::Vector3d point = start_point + distance_start_end*start_geometry;
        control_points.push_back(point);
    }
    else
    {
        Eigen::Vector3d point = start_point + distance_start_end*start_end_geometry;
        control_points.push_back(point);
    }
    
    if(has_end_geometry)
    {
        end_geometry = end_geometry/end_geometry.norm();
        Eigen::Vector3d point = end_point - distance_start_end*end_geometry;
        control_points.push_back(point);
    }
    else
    {
        Eigen::Vector3d point = end_point - distance_start_end*start_end_geometry;
        control_points.push_back(point);
    }
    
    control_points.push_back(end_point);
}

/**
 * @brief Generates points on the spline that passes through CERTAIN points
 * 
 * @param[in] path_points points through which the spline to pass through 
 * @param[out] spline_points spline points buffer
 */
void TrajectoryGeneration::generate_path_length(std::vector<Eigen::Vector3d> path_points, std::vector<Eigen::Vector3d> &spline_points)
{
    Eigen::Vector3d out_geometry;
    Eigen::Vector3d in_geometry;

    for(int i=0; i<path_points.size()-2; i++)
    {
        std::vector<Eigen::Vector3d> control_points;
        in_geometry = (path_points[i+2] - path_points[i+1])/(path_points[i+2] - path_points[i+1]).norm();

        if(i == 0)
            generate_control_points(path_points[i], false, out_geometry, path_points[i+1], true, in_geometry, control_points);
        else
            generate_control_points(path_points[i], true, out_geometry, path_points[i+1], true, in_geometry, control_points);

        out_geometry = in_geometry;

        generate_spline_points(control_points, spline_points);
    }
}

/**
 * @brief Generates points on the spline that passes through the ALL the given points
 * 
 * @param[in] path_points points through which the spline to pass through 
 * @param[out] spline_points spline points buffer
 */
void TrajectoryGeneration::generate_spline(std::vector<Eigen::Vector3d> path_points, std::vector<Eigen::Vector3d> &spline_points)
{
    if(path_points.size()%2 != 0)
    {
        Eigen::Vector3d middle_point = (path_points[path_points.size()/2+1]+path_points[path_points.size()/2])/2;
        path_points.insert(path_points.begin() + path_points.size()/2+1, middle_point);
    }

    std::vector<Eigen::Vector3d> first_half(path_points.begin(), path_points.begin() + (path_points.size()/2+1));
    std::vector<Eigen::Vector3d> second_half(path_points.begin() + (path_points.size()/2-1), path_points.end());
    std::vector<Eigen::Vector3d> second_half_spline_points;

    std::vector<Eigen::Vector3d> second_half_aux(second_half.rbegin(), second_half.rend());
    second_half.swap(second_half_aux);

    generate_path_length(first_half, spline_points);
    generate_path_length(second_half, second_half_spline_points);

    std::vector<Eigen::Vector3d> control_points;
    generate_control_points(first_half[first_half.size()-2], false, first_half[0], first_half[first_half.size()-1],  false, first_half[0], control_points);
    generate_spline_points(control_points, spline_points);

    for(int i = second_half_spline_points.size()-1; i>=0; i--)
        spline_points.push_back(second_half_spline_points[i]);
}

/**
 * @brief Generates the orientation on the spline that passes through ALL of the given orientation
 * 
 * @param[in] path_rotations orintation of the TCP along the spline
 * @param[out] spline_rotations orientation between the given ones
 */
void TrajectoryGeneration::generate_rotations(std::vector<Eigen::Vector3d> path_rotations, std::vector<Eigen::Vector3d> &spline_rotations)
{
    for(int i = 0; i < path_rotations.size()-1; i++)
        generate_rotation_length(path_rotations[i], path_rotations[i+1], spline_rotations);
}

/**
 * @brief Generates the orientation on the spline that passes through CERTAIN, 
 * of the given orientation
 * 
 * @param[in] path_rotations orintation of the TCP along the spline
 * @param[out] spline_rotations orientation between the given ones
 */
void TrajectoryGeneration::generate_rotation_length(Eigen::Vector3d start_rotation, Eigen::Vector3d end_rotation, std::vector<Eigen::Vector3d> &spline_rotations)
{
    Eigen::Matrix3d rot_mat_start;
    Eigen::Matrix3d rot_mat_end;
    Eigen::Matrix3d rot_mat_t;
 
    double rotation[3];

    double qs[4], qe[4], q[4];
    double t = 0;

    Kinematics::euler_to_quaternion(start_rotation(0), start_rotation(1), start_rotation(2), qs);
    Kinematics::euler_to_quaternion(end_rotation(0), end_rotation(1), end_rotation(2), qe);

    while(t <= 1 + step)
    {
        Kinematics::slerp(qs, qe, q, t);
        Kinematics::quaternion_to_euler(q, &rotation[0], &rotation[1], &rotation[2]);

        Eigen::Vector3d rotation_v(rotation[0], rotation[1], rotation[2]);

        spline_rotations.push_back(rotation_v);

        t += step;
    }
}

/**
 * @brief Generates a spline through the given points and with the given rotations
 * 
 * @param[in] path_points 
 * @param[in] path_rotations 
 * @param[out] full_path_points 
 */
void TrajectoryGeneration::spline(std::vector<Eigen::Vector3d> path_points, std::vector<Eigen::Vector3d> path_rotations, std::vector<std::vector<double>> &full_path_points)
{
    double joints[6], tcp[6];

    std::vector<Eigen::Vector3d> spline_points;
    std::vector<Eigen::Vector3d> spline_rotations;

    generate_spline(path_points, spline_points);
    generate_rotations(path_rotations, spline_rotations);

    for(int i=0; i< spline_points.size(); i++)
    {
        tcp[0] = spline_points[i](0);
        tcp[1] = spline_points[i](1);
        tcp[2] = spline_points[i](2);
        tcp[3] = spline_rotations[i](0);
        tcp[4] = spline_rotations[i](1);
        tcp[5] = spline_rotations[i](2);

        std::vector<double> pose(tcp, tcp + sizeof(tcp)/sizeof(tcp[0]));
        full_path_points.push_back(pose);
    }
}

/**
 * @brief Generate the instruction buffer depending on the plan buffer
 * 
 * @param[in] plan_buffer 
 * @return true finish the generation
 * @return false 
 */
bool TrajectoryGeneration::generate_instruction_buffer(std::vector<PlanCommand> plan_buffer)
{
    double current_pose[6] = {0, 0, 8.2, 0, 0, 0};
    double* current_pose_ptr = current_pose;
    int number_of_points = 0;
    for(int i=0;i<plan_buffer.size();i++)
    {
        if(plan_buffer[i].get_name() == "LINE")
        {
            line(current_pose_ptr, plan_buffer[i].get_pose(), full_path_points);
            instruction_buffer.push_back(InstructionCommand("MOVE", number_of_points, full_path_points.size()-1, false));
            number_of_points = full_path_points.size();

            copy_state(current_pose, plan_buffer[i].get_pose());
            current_pose_ptr = current_pose;
        }

        if(plan_buffer[i].get_name() == "CIRCLE")
        {
            double intermediat_pose[6] = {0};
            double end_pose[6] = {0};

            copy_state(intermediat_pose, (plan_buffer[i].get_spline_pose())[0]);

            copy_state(end_pose, (plan_buffer[i].get_spline_pose())[1]);

            circle(current_pose, intermediat_pose, end_pose, full_path_points);
            instruction_buffer.push_back(InstructionCommand("MOVE", number_of_points, full_path_points.size()-1, false));
            number_of_points = full_path_points.size();

            copy_state(current_pose, end_pose);
            current_pose_ptr = current_pose;
        }

        if(plan_buffer[i].get_name() == "SPLINE")
        {
            std::vector<Eigen::Vector3d> path_points;
            std::vector<Eigen::Vector3d> path_rotations;

            std::vector<std::vector<double>> pose = plan_buffer[i].get_spline_pose();

            path_points.push_back(Eigen::Vector3d(current_pose_ptr[0],current_pose_ptr[1],current_pose_ptr[2]));
            path_rotations.push_back(Eigen::Vector3d(current_pose_ptr[3],current_pose_ptr[4],current_pose_ptr[5]));

            for(int i=0;i<pose.size();i++)
            {
                path_points.push_back(Eigen::Vector3d(pose[i][0],pose[i][1],pose[i][2]));
                path_rotations.push_back(Eigen::Vector3d(pose[i][3],pose[i][4],pose[i][5]));
            }

            spline(path_points, path_rotations, full_path_points);
            instruction_buffer.push_back(InstructionCommand("MOVE", number_of_points, full_path_points.size()-1, false));
            number_of_points = full_path_points.size();

            copy_state(current_pose, pose[pose.size()-1]);
            current_pose_ptr = current_pose;
        }

        if(plan_buffer[i].get_name() == "GRIPPER")
            instruction_buffer.push_back(InstructionCommand("GRIPPER", plan_buffer[i].get_argument()));

        if(plan_buffer[i].get_name() == "PAUSE")
            instruction_buffer.push_back(InstructionCommand("PAUSE", plan_buffer[i].get_argument()));

        if(plan_buffer[i].get_name() == "PTP_COORD")
        {
            ptp_coord(current_pose, plan_buffer[i].get_pose(), full_path_points);

            instruction_buffer.push_back(InstructionCommand("MOVE", number_of_points, full_path_points.size()-1, true));
            number_of_points = full_path_points.size();

            current_pose_ptr = plan_buffer[i].get_pose();
        }

        if(plan_buffer[i].get_name() == "PTP_JOINT")
        {
            double joints[6];
            Kinematics::ik(dimensions, current_pose, joints);

            ptp_joints(joints, plan_buffer[i].get_pose(), full_path_points);

            instruction_buffer.push_back(InstructionCommand("MOVE", number_of_points, full_path_points.size()-1, true));
            number_of_points = full_path_points.size();

            Kinematics::dk(dimensions, plan_buffer[i].get_pose(), current_pose);
        }
    }

    return true;
}

/**
 * @brief Print the path buffer
 * 
 */
void TrajectoryGeneration::print_full_path_points()
{
    ROS_INFO_STREAM("TG: Full path:");
    for(int i=0;i<full_path_points.size();i++)
        ROS_INFO_STREAM("TG: " << i << " " << full_path_points[i][0] << " " << full_path_points[i][1] << " " << full_path_points[i][2] << " " << full_path_points[i][3] << " " << full_path_points[i][4] << " " << full_path_points[i][5]);
}

/**
 * @brief Print the instruction buffer
 * 
 */
void TrajectoryGeneration::print_instruction_buffer()
{
    ROS_INFO_STREAM("TG: Instruction buffer:");
    for(int i=0;i<instruction_buffer.size();i++)
    {
        if(instruction_buffer[i].get_name() == "MOVE")
            ROS_INFO_STREAM(instruction_buffer[i].get_name() << ": " << instruction_buffer[i].get_from() << " - " << instruction_buffer[i].get_to());
        else
            ROS_INFO_STREAM(instruction_buffer[i].get_name() << ": " << instruction_buffer[i].get_arg());
    }
}

/**
 * @brief Print the values of an array
 * 
 * @param[in] msg start message to print 
 * @param[in] values values to print
 * @param[in] n number of galues to print
 */
void TrajectoryGeneration::print_values(std::string msg, double *values, int n)
{
    std::cout << msg << " ";
    for(int i=0;i<n;i++)
        std::cout << values[i] << " ";
    std::cout << std::endl;
}

std::vector<InstructionCommand> TrajectoryGeneration::get_instruction_buffer()
{
    return instruction_buffer;
}

std::vector<std::vector<double>> TrajectoryGeneration::get_full_path_points()
{
    return full_path_points;
}
