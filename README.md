# Industrial manipulator - Source code Package
## Foreword
This is a repository that is a part of five that are meant to help in designing a six degree-of-freedom manipulator.
Repositories:
 - **Source Code Package**
 - 3D Model Spawner Package 
 - MatLab Scripts
 - Model of the arm
 - User Interface in LabView
## Content 
This repository contains the necessary code that will run a 6-DOF manipulator. It contains a `Kinematic` class that provides methods to aid in the generation process of the trajectories as well as methods that solve the direct and invers kinematics.
### Dependencies
 Compile dependencies:
 - [Eigen][eigen_install] template library for linear algebra 
 
Run dependencies:
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

> **Note:** A small change needs to be made in the `mimic_joint` plugin. In the `roboticsgroup_gazebo_plugins/src` folder file `mimic_joint_plugin.cpp` line 179 needs to be replace with `mimic_joint_->SetPosition(0, -angle, true);`. Be sure to build again after the change has been made

[eigen_install]: https://eigen.tuxfamily.org/dox/GettingStarted.html
[g_pkgs]: https://github.com/JenniferBuehler/gazebo-pkgs/wiki/Installation
[gripper]: https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins
