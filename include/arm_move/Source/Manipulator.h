#include <ros/ros.h>

#include <arm_move/Source/Planner.h>
#include <arm_move/Source/TrajectoryGeneration.h>
#include <arm_move/Source/PathExecutor.h>

#include <arm_move/Source/Workarea.h>

/**
 * Main class that defines the manipulator
 * Uses all the other classes in order to move the arm
 */
class Manipulator
{
private:
    Planner *pl;
    TrajectoryGeneration *tg;
    PathExecutor *ph;
    Workarea *wo;
public:
    Manipulator(ros::NodeHandle *nh);
    
    // Starts the manipulator
    void arm_initialize();
};

#pragma once
