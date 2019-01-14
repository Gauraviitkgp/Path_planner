cmake_minimum_required(VERSION 2.8.3)
project(trajectory_optimsation)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# find_package()
# find_package()


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
find_package(
	Eigen3 3.3 REQUIRED NO_MODULE
	)

find_package(gazebo)


find_package(catkin_simple REQUIRED)


catkin_simple()

catkin_package(
	INCLUDE_DIRS include
	)

cs_add_library(geometry_eigen_conversions
	src/geometry_eigen_conversions.cpp
	)

target_link_libraries(geometry_eigen_conversions
	Eigen3::Eigen)

# target_include_directories(geometry_eigen_conversions PUBLIC
# 	include)

cs_add_library(mpc_solver
	externals/qpoases/SRC/Bounds.cpp
  	externals/qpoases/SRC/Constraints.cpp
	externals/qpoases/SRC/CyclingManager.cpp
	externals/qpoases/SRC/Indexlist.cpp
	externals/qpoases/SRC/MessageHandling.cpp
	externals/qpoases/SRC/QProblem.cpp
	externals/qpoases/SRC/QProblemB.cpp
	externals/qpoases/SRC/SubjectTo.cpp
	externals/qpoases/SRC/Utils.cpp
	externals/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
	model/hedwig_mpc_codegen/acado_qpoases_interface.cpp
	model/hedwig_mpc_codegen/acado_integrator.c
	model/hedwig_mpc_codegen/acado_solver.c
	model/hedwig_mpc_codegen/acado_auxiliary_functions.c
	)

target_include_directories(mpc_solver PUBLIC
	model/hedwig_mpc_codegen
	externals/qpoases
	externals/qpoases/INCLUDE
	externals/qpoases/SRC
	)

target_link_libraries(mpc_solver
	geometry_eigen_conversions
	)

# cs_add_library(mpc_wrapper
# 	src/hedwig_mpc_wrapper.cpp)

# target_link_libraries(mpc_wrapper
# 	mpc_solver)

# cs_add_library(mpc_controller
# 	src/hedwig_mpc_controller.cpp)

# target_link_libraries(mpc_controller 
# 	mpc_wrapper)

# make an executable
# cs_install()
# cs_export()

cs_add_executable(mpc_controller_node
  src/hedwig_controller.cpp)
target_link_libraries(mpc_controller_node
  mpc_solver)

# cs_add_executable(hedwig_mpc_instance
#   src/hedwig_mpc_instance.cpp)
# target_link_libraries(hedwig_mpc_instance
#   mpc_controller
#   mpc_wrapper
#   mpc_solver)


install(DIRECTORY models
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	)

cs_install()

cs_export()
