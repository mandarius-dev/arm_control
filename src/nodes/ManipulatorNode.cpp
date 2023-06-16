#include <ros/ros.h>

#include <arm_move/Source/Manipulator.h>

int main(int argc,char **argv)  
{
    // Initializaion of the main node
    ros::init(argc, argv, "manipulator_node");

    // Node handle of the node
    ros::NodeHandle nh;

    Manipulator m(&nh);
    m.arm_initialize();

    return 0;
}