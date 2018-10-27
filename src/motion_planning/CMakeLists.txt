cmake_minimum_required(VERSION 2.8.3)
project(motion_planning)

find_package(gazebo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gazebo_msgs
  gazebo_plugins 
  gazebo_ros
  message_generation
  rospy
  sensor_msgs
  std_msgs
)


catkin_package()

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

