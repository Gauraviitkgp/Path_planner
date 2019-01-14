#!/bin/bash
source ~/Path_planner/devel/setup.bash
source ~/src/Firmware/Tools/setup_gazebo.bash ~/src/Firmware/ ~/src/Firmware/build/posix_sitl_default 
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/src/Firmware/ 
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/src/Firmware/Tools/sitl_gazebo 
export GAZEBO_MODEL_PATH=~/Path_planner/ROS/src/iarc_px4_gazebo/models:$GAZEBO_MODEL_PATH

