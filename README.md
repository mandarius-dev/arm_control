# Industrial manipulator - Source code Package
## Foreword
This is a repository that is a part of five that are meant to help in designing a six-degree-of-freedom manipulator.
Repositories:
 1. **Source Code Package**
 2. 3D Model Spawner Package [[Link][spawner]]
 3. MatLab Scripts [[Link][script]]
 4. Model of the arm [[Link][model]]
 5. User Interface in LabView [[Link][UI]]
## Content 
This repository contains the necessary code that will run a 6-DOF manipulator. It contains a `Kinematic` class that provides methods to aid in the generation process of the trajectories as well as methods that solve the direct and inverse kinematics.
## Installation and run 
This repository is a simple ROS package. All that is needed to install it is to clone it in a ROS workspace and build it. There are two dependencies, a mathematical library used to ease the implementation of the direct and inverse kinematics, and some other methods used throughout the code. The second dependency is two plugins used to improve the gripping capabilities of the actual gripper.
### Dependencies
 Compile dependencies:
 - [Eigen][eigen_install] template library for linear algebra 
 - The [`ros_control`][ros_control] package with the controllers
 - `ros-noetic-object-recognition-msgs` for the next packages
 	```
	sudo apt-get install ros-noetic-object-recognition-msgs
	```
 - JenniferBuehler [gazebo-pkgs][g_pkgs] and some plugins used for the [gripper][gripper]
	 - The commands for the needed packages are as follows:
		```
		git clone https://github.com/JenniferBuehler/general-message-pkgs.git
		git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
		git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
		```
- After the packages are in the local workspace simply run `catkin_make`

> **Note:** A small change needs to be made in the `mimic_joint` plugin. In the `roboticsgroup_gazebo_plugins/src` folder, file `mimic_joint_plugin.cpp`. line 179 needs to be replace with `mimic_joint_->SetPosition(0, -angle, true);`. Be sure to build again after the change has been made
### Run
To start the package there is a launch file provided. 
`roslaunch arm_move start_control.launch`

Some information will be provided in the console about the commands read and the full list of points and orientations. If any errors occur during run time or a syntax error is  detected in the command folder this will also be printed in the console.

After all the points of the trajectory are computed the program will publish on a specific topic the correct rotation for each joint. From those topics, the angels can be further processed to suit the need of the manipulator. 
## Structure 
The following diagram presents a basic structure of the code.
![](https://mermaid.ink/img/pako:eNqNVNtKw0AQ_ZUwT15SaatNbZ4VH0QQFAQJhHEzbdduNmGzwdbQf3dyad2WFJ2X7O6Zc87MJJsKRJYQhCAUFsWdxIXBNNIexxNqmZcKbWYuBoNnhVqT6YNeDX6S4PXmgTgFrcx0rwTa5f2aRMnbPvwtMys0hFHHdvMZfpSaUhYXxUtWGkFtUp95b3Kb3rTpHaNe1aJ1XCars3NnK_fb7aHIrtwDsliSWMVfHXSC6bTtkncz9nLlHPY16NmFS3Pm5OXk1q6llajkN52opHPcFzFQsrCeyNIUdRIXG21xfYjlzPgo5_Pdp8Au3Cjn1qm_g-sG0So45r_WfX1Vf3m151IX1pSiZsRHtSh-s_FiL-gWJI1QJ7AiP8E7qNidc_XviqhhUJwz-0h2F-BDSiZFmfBFbJQjsEtKKYKQl5pKa1BFEOktpyL7v2y0gJAtyYcyT9BSd3UhnKMq-DRHDWEFawhHwfBqFMyCyXh4M7m9GU992EA4GE2vZnVcB-NpMBnOtj58ZxkLjHygRHKPT92foX40gu8N3hhsfwCf4lss)

**Manipulator**
A central class will start all the processes needed to fully run the manipulator.

**Planner**
Reads the syntax of the commands and stores it in a special buffer in order to be later used to check the syntax provided by the user. A plan buffer will be created if the syntax of all commands is correct.

**TrajectoryGeneration**
With the buffer of commands from the **Planner** a continuous trajectory will be generated and stored in another buffer. Besides the trajectory, a set of simple commands will be also generated in order to know whether the arm should pause or activate the gripper or move to a new pose.

**PathExecutor**
The trajectory buffer will be taken and looped through it in order to move the arm to the new poses. Inverse kinematics will be used in order to obtain the angles for the joints and published them to specific topics.

**Workspace**
Check if the points generated are valid, not in any forbidden zones, and in the work envelope. 

> **Note:** The presented structure is a simplified version of the actual structure. Other classes are used, especially for the buffers.

## Configuration
Different configurations can be made to the program in order to better suit the build manipulator.

**Config file: `topics.yaml`**
Contains a list of the topic's names on which should the angle of each joint to be published.
There is also a list of topics names from which should the program read the current state of the joint.
The topic of the gripper actuation is also present.

>**Note:** The names are specifically chosen in order to correctly interface with the `ros_control` controllers. It is recommended to leave the names as is and adapt accordantly.

**Config file: `variables.yaml`** 
A set of variables that are used throughout the code for different purposes. 
 - `pkg_name` - the name of the package in which the code is placed, this name is used to generate the paths of other files needed by the program
 - `command_syntax_path` - the path to the file in which the syntax of the commands is stored
 - `command_path` - file location of where the commands provided are stored
 - `step` - determines how many points will be generated for each segment of the trajectory
 - `sleep_time` - the time between publishes of the angels
 - `dimensions` - array with the dimensions of the manipulator 
 - `gripper_actuation_time` - time needed for the actuator to open and close
 - `z_offcet` - this is the distance to the bottom of the manipulator, mainly used in the simulation 
 - `link_radius` - radius of the links, used for self-collision

>**Note:** This set of variables can be changed accordingly to the real build of the manipulator.

**Config file: `forbiddedn_zones.txt`**
Definitions of where the forbidden zones are in 3D space. The file should contain how many forbidden zones are on the first line and on the following lines the opposite points coordinates.

[eigen_install]: https://eigen.tuxfamily.org/dox/GettingStarted.html
[g_pkgs]: https://github.com/JenniferBuehler/gazebo-pkgs/wiki/Installation
[gripper]: https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins
[ros_control]: http://wiki.ros.org/ros_control

[UI]: https://github.com/mandarius-dev/arm_user_interface
[spawner]: https://github.com/mandarius-dev/arm_model_spawner
[script]: https://github.com/mandarius-dev/arm_matlab_scripts
[control]: https://github.com/mandarius-dev/arm_control
[model]: https://github.com/mandarius-dev/arm_3d_model
