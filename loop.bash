while true 
do 
    rosservice call /gazebo/reset_world
    roslaunch arm_move start_control.launch 
done