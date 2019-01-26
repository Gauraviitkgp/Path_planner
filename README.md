# Path_planner
This is a ROS package for Motion planning consisting of two parts:
1) More important Model Predictive Control
2) Python implemention to use control inputs for quadrotor movements

##Dependencies Required for installation of package:

1. 	ROS-Gazebo installed
2.	Pixhawk ROS interface installed in /home/src
3.	Mavros Mavlink installed in /catkin_ws/src 
if not use https://dev.px4.io/en/ros/mavros_installation.html to install.
4. 	Automatic Control and Dynamic Optimization(ACADO) installed from http://acado.github.io/install_linux.html
5.	Qpoases install https://projects.coin-or.org/qpOASES/wiki/QpoasesInstallation
6.	

##Steps for first build

1.	Develop a ROS simulation enviroment in gazebo or can directly used the simulation enviroment used in the package. (px4_gaz is the simulation package being used)
2.	Directly copy catkin_simple package into your source dir
3.	Make a package for mpc of your choice. Let's name is trajectory_optimsation
4.	Make a directory named "externals" in your package. Make a subdirectory name qpoases in it
5.	Copy the contents of /qpOASES-version/include/qpOASES to /trajectory_optimsation/externals/qpoases/INCLUDE similarly do for /qpOASES-version/src/ and /qpOASES-version/examples.
6.	Make a directory called model in your package.
7.  Copy the codegen into it. Generation of codegen is in a different section
8.	Make directories called include and src and copy all the files in this repository to them

###Generating Codegen
1.	copy /Path_planner/Resources/ACADO_FILES to ~ /ACADOtoolkit/examples/my_example
2.	``` 
cd ~/ACADOtoolkit
make 
```
3.	```
cd examples/my_examples
./simple_ocp_2.cpp
```

4.	```cd ark_mpc``` This folder contains the codegen.
5.	Modify acado_auxilary_functions.c to file in this repository
6.	Run
``` make
./test
```
to test whether code is correct or not






do
```
	source start.sh

	roslaunch px4_gaz Simulate.launch 
	rosrun motion_planning motionplanner.py
```

motion_planning_1 is the prelimanry version for testing
trajectory optimsation is the 2nd version

If you face problems in connecting to mavlink in your pc, you should check that fcu_url is correct. https://dev.px4.io/en/simulation/ros_interface.html
