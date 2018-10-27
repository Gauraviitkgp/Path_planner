cmake_minimum_required(VERSION 2.8.3)
project(px4_gaz)
add_compile_options(-std=c++11)
find_package(gazebo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gazebo_msgs
  gazebo_plugins 
  gazebo_ros
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS px4 mavros geometry_msgs message_generation roscpp rospy sensor_msgs std_msgs gazebo_msgs gazebo_plugins gazebo_ros
)

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

