#!/bin/bash
source ~/Path_planner/devel/setup.bash
source ~/src/Firmware/Tools/setup_gazebo.bash ~/src/Firmware/ ~/src/Firmware/build/posix_sitl_default
# source ~/src/Firmware/Tools/setup_gazebo.bash ~/src/Firmware/ ~/src/Firmware/build_gazebo/
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/src/Firmware/ 
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/src/Firmware/Tools/sitl_gazebo 
export GAZEBO_MODEL_PATH=~/Path_planner/src/px4_gaz/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/src/sitl_gazebo/Build
# Set the model path so Gazebo finds the airframes
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/src/sitl_gazebo/models
# Disable online model lookup since this is quite experimental and unstable
export GAZEBO_MODEL_DATABASE_URI=""
export SITL_GAZEBO_PATH=~/src/sitl_gazebo