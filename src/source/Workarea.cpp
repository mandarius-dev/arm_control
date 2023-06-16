#include <arm_move/Source/Workarea.h>

/**
 * @brief Construct a new Workarea:: Workarea object and get the parameters
 * 
 * @param nh node handle
 */
Workarea::Workarea(ros::NodeHandle *nh)
{
    std::vector<double> dim;
    nh->getParam("dimensions", dim);
    copy_state(dimensions, dim);

    nh->getParam("z_offcet", z_offcet);

    nh->getParam("link_radius", link_radius);
}

/**
 * @brief Check if the points are NOT in he forbidden zones and if they are in the workarea
 * 
 * @return true all points are valid
 * @return false one point is in the forbidden zone or out side workarea
 */
bool Workarea::check_workarea()
{
    if(!check_workspace_limits())
    {
        ROS_ERROR_STREAM("Point outside of workarea!");
        return false;
    }
    
    if(!check_self_forbidden_colide())
        return false;

    return true;
}

/**
 * @brief Check if the points are inside the work area
 * 
 * @return true 
 * @return false 
 */
bool Workarea::check_workspace_limits()
{
    double r = dimensions[2] + dimensions[3] + dimensions[4] + dimensions[5];
    double tcp[6], joints[6];

    double c_z = dimensions[0] + dimensions[1] + z_offcet;

    if(plan_buffer[plan_buffer.size() - 1].get_name() == "PAUSE" || plan_buffer[plan_buffer.size() - 1].get_name() == "OPEN" || plan_buffer[plan_buffer.size() - 1].get_name() == "CLOSE")
        return true;
    
    if(plan_buffer[plan_buffer.size() - 1].get_name() == "PTP_JOINTS")
    {
        copy_state(joints,plan_buffer[plan_buffer.size() - 1].get_pose());
        Kinematics::dk(dimensions,joints,tcp);

        if(tcp[0]*tcp[0]+tcp[1]*tcp[1]+(tcp[2]-c_z)*(tcp[2]-c_z) > r*r)
        {
            ROS_ERROR_STREAM("Point outside of workarea!");
            return false;
        }
    }

    if(plan_buffer[plan_buffer.size() - 1].get_name() == "PTP_COORD" || plan_buffer[plan_buffer.size() - 1].get_name() == "LINE" || plan_buffer[plan_buffer.size() - 1].get_name() == "CIRCLE")
    {
        copy_state(tcp,plan_buffer[plan_buffer.size() - 1].get_pose());

        if(tcp[0]*tcp[0]+tcp[1]*tcp[1]+(tcp[2]-c_z)*(tcp[2]-c_z) > r*r)
        {
            ROS_ERROR_STREAM("Point outside of workarea!");
            return false;
        }
    }

    if(plan_buffer[plan_buffer.size() - 1].get_name() == "SPLINE")
    {
        std::vector<double> pose;

        for(int i=0; i<plan_buffer[plan_buffer.size()-1].get_spline_pose().size(); i++)
        {
            pose = plan_buffer[plan_buffer.size()-1].get_spline_pose()[i];
            if(pose[0]*pose[0]+pose[1]*pose[1]+(pose[2]-c_z)*(pose[2]-c_z) > r*r)
            {
                ROS_ERROR_STREAM("Point outside of workarea!");
                return false;
            }
        }
    }

    return true;
}

/**
 * @brief Read the coordinates of the forbidden zones from the file
 * 
 * @return true point are in valid
 * @return false invalid points
 */
bool Workarea::read_forbidden_zones()
{
    forbidden_zones_path = ros::package::getPath("arm_move") + "/config/forbidden_zones.txt";
    forbidden_zones_file.open(forbidden_zones_path);

    double x_min, y_min, z_min, x_max, y_max, z_max;

    while(forbidden_zones_file.peek()!=EOF)
    {
        forbidden_zones_file >> x_min >> y_min >> z_min >> x_max >> y_max >> z_max;
        if(abs(x_min) > abs(x_max) || abs(y_min) > abs(y_max) || abs(z_min) > abs(z_max))
        {
            ROS_ERROR_STREAM("Invalid forbizzed zone configuration.");
            return false;
        }
        
        Cube cube(x_min, y_min, z_min, x_max, y_max, z_max);
        forbidden_zones.push_back(cube);
    }

    ROS_INFO_STREAM("WO: " + std::to_string(forbidden_zones.size()) + " forbizzen zone/zones found to initizlized.");

    return true;   
}

/**
 * @brief Check if a point is inside a capsule 
 * 
 * @param start_point 
 * @param end_point 
 * @param check_point point to be checked
 * @param radius radius of capsule
 * @return true point inside the capsule
 * @return false point outside the capsule
 */
bool Workarea::check_point_inside_capsule(Eigen::Vector3d start_point, Eigen::Vector3d end_point, Eigen::Vector3d check_point, double radius)
{
    double distance = -1;
    double t;

    Eigen::Vector3d n = end_point - start_point;
    Eigen::Vector3d v = check_point - start_point;

    t = n.dot(v)/(n.norm()*n.norm());

    if(t<0)
        distance = (check_point - start_point).norm();
    else    
        if(t>0)
            distance = (check_point - end_point).norm();

    if(distance != -1 && distance > radius)
        return false;
    else   
        if(distance != -1 && distance <= radius)
            return true;

    Eigen::Vector3d intermediat_point = start_point + t*n;

    distance = (check_point - intermediat_point).norm();

    if(distance > radius)
        return false;
    else
        return true;    
}

/**
 * @brief Self collision test
 * 
 * @return true 
 * @return false 
 */
bool Workarea::check_self_forbidden_colide()
{
    Eigen::Vector3d tcp_point;
    
    Eigen::Vector3d start_point(0, 0,dimensions[0] + z_offcet);
    Eigen::Vector3d end_point(0, 0, dimensions[0] + dimensions[1] + z_offcet);

    double tcp[6], joints[6];
    for(int i=0;i<instruction_buffer.size();i++)
    {
        if(instruction_buffer[i].get_name()=="MOVE")
        {
            for(int j=instruction_buffer[i].get_from(); j<instruction_buffer[i].get_to(); j++)
            {
                if(instruction_buffer[i].is_joint_space())
                {
                    copy_state(joints, full_path_points[j]);
                    Kinematics::dk(dimensions, joints, tcp);

                    tcp_point << tcp[0], tcp[1], tcp[2];
                }
                else
                {
                    copy_state(tcp, full_path_points[j]);
                    tcp_point << tcp[0], tcp[1], tcp[2];
                }

                if(check_point_inside_capsule(start_point, end_point, tcp_point, link_radius))
                {
                    ROS_ERROR_STREAM("Manipulator will self colide.");
                    return false;
                }

                for(int j=0;j<forbidden_zones.size();j++)
                    if(forbidden_zones[j].check_point_inside(tcp[0], tcp[1], tcp[2]))
                    {
                        ROS_ERROR_STREAM("Trajectory is inside a forbidden zone!");
                        return false;
                    }
            }    
        }
    }

    return true;
}

/**
 * @brief Copy elements from a classic vector to a classic one 
 * 
 * @param[in] from classic vector
 * @param[out] where classic vector
 */
void Workarea::copy_state(double *where, double *from)
{
    where[0] = from[0];
    where[1] = from[1];
    where[2] = from[2];
    where[3] = from[3];
    where[4] = from[4];
    where[5] = from[5];
}

/**
 * @brief Copy elements from a std vector to a std one 
 * 
 * @param[in] from std vector
 * @param[out] where classic vector
 */
void Workarea::copy_state(double *where, std::vector<double> from)
{
    where[0] = from[0];
    where[1] = from[1];
    where[2] = from[2];
    where[3] = from[3];
    where[4] = from[4];
    where[5] = from[5];
}


void Workarea::set_buffers( std::vector<PlanCommand> _plan_buffer, std::vector<InstructionCommand> _instruction_buffer, std::vector<std::vector<double>> _full_path_points)
{
    plan_buffer = _plan_buffer;
    instruction_buffer = _instruction_buffer;
    full_path_points = _full_path_points;
}