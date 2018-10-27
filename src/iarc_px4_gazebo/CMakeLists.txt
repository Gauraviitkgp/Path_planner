cmake_minimum_required(VERSION 2.8.3)
project(iarc_px4_gazebo)
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

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(activate_node src/activate.cpp)
target_link_libraries(activate_node ${catkin_LIBRARIES})

add_executable(joy_controller src/joy_controller.cc)
target_link_libraries(joy_controller ${catkin_LIBRARIES})
add_dependencies(joy_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lidar_spinner src/lidar_spinner.cc)
target_link_libraries(lidar_spinner ${catkin_LIBRARIES})
add_dependencies(lidar_spinner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(groundbot src/bot.cpp)
target_link_libraries(groundbot ${catkin_LIBRARIES})

add_library(contact SHARED src/ContactPlugin.cc)
target_link_libraries(contact ${GAZEBO_libraries})

