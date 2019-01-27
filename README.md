# Path_planner
This is a ROS package for Motion planning consisting of two parts:
1) More important Model Predictive Control
2) Python implemention to use control inputs for quadrotor movements

## Dependencies Required for installation of package:

1. 	ROS-Gazebo installed
2.	Pixhawk ROS interface installed in /home/src
3.	Mavros Mavlink installed in /catkin_ws/src 
if not use https://dev.px4.io/en/ros/mavros_installation.html to install.
4. 	Automatic Control and Dynamic Optimization(ACADO) installed from http://acado.github.io/install_linux.html
5.	Qpoases install https://projects.coin-or.org/qpOASES/wiki/QpoasesInstallation
6.	

## Steps for first build

1.	Develop a ROS simulation enviroment in gazebo or can directly used the simulation enviroment used in the package. (px4_gaz is the simulation package being used)
2.	Directly copy catkin_simple package into your source dir
3.	Make a package for mpc of your choice. Let's name is trajectory_optimsation
4.	Make a directory named "externals" in your package. Make a subdirectory name qpoases in it
5.	Copy the contents of /qpOASES-version/include/qpOASES to /trajectory_optimsation/externals/qpoases/INCLUDE similarly do for /qpOASES-version/src/ and /qpOASES-version/examples.
6.	Make a directory called model in your package.
7.  Copy the codegen into it. Generation of codegen is in a different section
8.	Make directories called include and src and copy all the files in this repository to them

### Generating Codegen
1.	copy /Path_planner/Resources/ACADO_FILES to ~ /ACADOtoolkit/examples/my_example
2.	
``` 
	cd ~/ACADOtoolkit
	make 
	cd examples/my_examples
	./simple_ocp_2.cpp
	cd ark_mpc
``` 
This folder contains the codegen.

3.	Modify acado_auxilary_functions.c to that file in this repository
4.	Run
``` 
	make
	./test
```
to test whether code is correct or not

## Basics and Mathematical concepts

###	Trajectory Optimization 
Trajectory optimization is the process of designing a trajectory that minimizes (or maximizes) some measure of performance while satisfying a set of constraints. Trajectory can be understood as a velocity profile, or in simpler terms a time-based space state of a robot

https://arxiv.org/pdf/1707.00284.pdf?fbclid=IwAR14hNSRHO5aydoc0abLcX0KLFYKgNcee_F6Qfz_O2mw89TCFSpvDaBQ3RM paper is a good way to begin Trajectory optimzation

Further deep knowledge on collocation can be found on https://epubs.siam.org/doi/pdf/10.1137/16M1062569

### Model Predictive Control
Model predictive control (MPC) is an advanced control strategy which allows to determine
inputs of a given process that optimise the forecasted process behaviour.MPC is an non-linear optimsation problem and a specific case of Trajectory Optimisation. It gives us the optimal input controls to achieve a specific state under quadrotor constraints. It is a cost function minimization problem. 

This paper is a good way to understand MPC,(ignore the preception part) https://arxiv.org/pdf/1804.04811.pdf?fbclid=IwAR2SBQ6v0qqDQsPWLyUfMYM5nyR6wxp7mfWfFEFJ4a3CWVCeVS1pbHGhJSc

###	Acado
Acado stands for Automatic Control and Dynamic Optimization. ACADO Toolkit is a software environment and algorithm collection for automatic control and dynamic optimization. It provides a general framework for using a great variety of algorithms for direct optimal control, including model predictive control, state and parameter estimation and robust optimization. ACADO Toolkit is implemented as self-contained C++ code.

https://www.youtube.com/watch?v=JC6PNjndQ_c&t=2724s link is a good implemention of ACADO if you're well versed with MPC.


#### Our Model
Implemention of MPC was done to improvise quad localisation and counter high speed winds. Winds were modelled in form of differential equations and were added it to the states wherever it affected. Simple_ocp.cpp and Simple_ocp_2 is the implemention of the following model.(can be found on Resources/ACADO_FILES/simple_ocp.cpp)

State of a Quadrotor is defined by a vector
```
	x=[positionx, py,pz, velx,vy,vz,quaternionw,qx,qy,qz]
```
Controls of our quadrotor are defined by vector
```
	u=[thrust_c, angular_vel_x, avy,avz]
```
Let the reference trajectory(A path which you want the quad to take. Say let, we want our robot to go from Delhi to Bombay, a reference trajectory can be a direct euclidean distance between them, however the mpc would create a dynamially feasible path, taking account of high cost of path where mettalic road isn't present, turns, crossings, traffic etc to give us the optimal trajectory to follow. It will give us output as which controls to take such that such a path is followed) be 
```
	y=[positionx_ref, py_r,pz_r, velx_r,vy_r,vz_r,quaternionw_r,qx_r,qy_r,qz_r]
```
Hence, our non linear cost function is https://latex.codecogs.com/gif.latex?\int_{t=0}^{t=t} A_1\times(px-pxr)^2+A_2 \times (py-pyr)^2+\ldots+A_9 \times(qz-qzr)^2 dt

![equation](https://latex.codecogs.com/gif.latex?%5Cint_%7Bt%3D0%7D%5E%7Bt%3Dt%7D%20A_1%5Ctimes%28px-pxr%29%5E2&plus;A_2%20%5Ctimes%20%28py-pyr%29%5E2&plus;%5Cldots&plus;A_9%20%5Ctimes%28qz-qzr%29%5E2%20dt)

When we optimize this cost function we're actually trying to minimize the eculidian distance between the reference trajectory and the path. By adding higher order terms for specific states in the cost function we can redirect the robot to take a new path. Here the Ai's are the coefficients of the associated state variable in the cost function which is represented by a Q matrix which takes value from Qd Vector
```
	Qd=[200, 200, 500, 10, 10, 10, 50, 50, 50, 50]
	Q[10][10]=Qd.asDiagonal()
```
One more parameter which is always included in the mpc is the time. We need to optimize our model with respect to time. Which is represented as (in sec)
```
	t_start							=0.0
	t_end							=5.0
	number of discretized intervals	=20
	Discretization time 			=(5.0-0.0)/20=0.25
```
Note: Discretization time is the time interval after which the mpc again optimizes. Lower the number more accurate is the path, but also increases the computation time.

Differential Element of states wrt time are where c=upward accn and g=gravity
```
	d(px)/dt = vx;                         
    d(py)/dt = vy;
    d(pz)/dt = vz;
    d(vx)/dt = 2.0*(qx*qz - qw*qy)*c;
    d(vy)/dt = 2.0*(qy*qz + qw*qx)*c;
    d(vz)/dt = (1.0 - 2.0*qx*qx - 2.0*qy*qy)*c+g;
    d(qw)/dt = (-wx*qx-wy*qy-wz*qz)/2.0;
    d(qx)/dt = (wx*qw+wz*qy-wy*qz)/2.0;
    d(qy)/dt = (wy*qw-wz*qx+wx*qz)/2.0;
    d(qz)/dt = (wz*qw+wy*qx-wx*qy)/2.0;
```
Other important factors in the model(RK stands for Runge-kutta) (RK methods are included in NsOPDE course of mathematics)
```
	INTEGRATOR_TYPE			= 		INT_RK4 
    NUM_INTEGRATOR_STEPS	= 		30 
    HESSIAN_APPROXIMATION	= 		GAUSS_NEWTON 
```

v1.0 Consists of the basic mathematical model of Quadcoptor without integration of ROS 

### Qpoases
Read https://www.coin-or.org/qpOASES/doc/3.0/manual.pdf

## ROS interfacing 
To start with an interface with ROS, create an src folder and include controller.cpp. Include a controller.hpp as a header file. 

#### hedwig_controller.h
##### Hedwig_command: Hedwig class for command
```
	hgeometry:	Public Variable for storing the angular velocity commands(geometry_msgs::TwistStamped )
	hposition:	Extra Variable to store current position(geometry_msgs::PoseStamped)
	lift:		Variable for storing Upward accn(thrust) value(mavros_msgs::Thrust)
	timestamp:	Store the timestamp of the command(ros::Time)
	armed:		Status Arming. MPC won't give commands unless its armed(bool)
```
Various different Constructers are used to initialize the object, if the given inputs are in the form of Ros::messages. Other functions include
```
	set_command:	Used to change the command after initialization and inputs are available in ROS form
	zero:			Used to set everything to 0
	set_thrust:		Used to set thrust if thrust is available as float
	set_angle:		Used to set angular velocity commands if available as float
```
##### Hedwig_state:	Hedwig class for state
```
	hstate:		Used to store the position and quaternion values in state(geometry_msgs::PoseStamped)
	hvel:		Used to store the velocity values in state(geometry_msgs::TwistStamped)
	timestamp:	Store the timestamp of the command(ros::Time)
	hPOSX:		Used to store x_corr of quad position(float)
	hPOSY:		Used to store y_corr of quad position(float)		
	hPOSZ:		Used to store z_corr of quad position(float)
	hOriW:		Used to store qw_corr of quad orientation(float)
	hOriX:		Used to store qx_corr of quad orientation(float)
	hOriY:		Used to store qy_corr of quad orientation(float)
	hOriZ:		Used to store qz_corr of quad orientation(float)
	hVelX:		Used to store vx_corr of quad velocity(float)
	hVelY:		Used to store vy_corr of quad velocity(float)
	hVelZ:		Used to store vz_corr of quad velocity(float) 
```
Various different Constructers are used to initialize the object, if the given inputs are in the form of Ros::messages. Other functions include
```
	set_state:	Used to change the state after initialization and inputs are available in ROS form
```
#### hedwig_controller.cpp
##### Macros
These are various acado definations (see adjacent comments for more details )
##### Htrajectory:	Hedwig class for trajectory
Class use to store both command and state variable at a particular time
```
	command:	Used to store commands from mpc(hcommand)
	state:		Used to store states from mpc(hstate)
```
Various different Constructers are used to initialize the object, if the given inputs are in the form of Ros::messages. Other functions include
```
	set_hover_command:	Used to set hover command at the current state. Modify the th value according the the quad
	set_commandnstate:	Used to set both command and state simultenously.
	initialize:			Used to initialize the controller.
	run_mpc:			runs the mpc and its steps
```
Various elements present in initialize are explained as follows
```
	memset:						Sets the memory and clears it for acado workspace
	acado_initializeSolver():	Initialize the solver
	acadoVariables.x[i]:	Initializes the state variables. Prefer a value close to ref_trajectory, to get the global optimum.
	acadoVariables.u[i]:	Initializes the control Variables. 
	acadoVariables.y[i]:	Initialize the reference trajectory
	acadoVariables.x0[i]:	Initializes the 0th state or state at t=0
```
#### Acado codegen
This is a set of portable c++ code generated directory which helps acado interfacing with different languages and softwares(ROS in this Case). Codegen is dependent upon the model you specify.
```
	acado_auxiliary_functions:	Auxiliary functions which help us in first line of interfacing with ACADO. Comments are added for detail understanding
	acado_common:		Header file for various declarations of variables and functions
	acado_integrator:	File for numerical integrations
	acado_qpoases_interface:	Acado file to intereface with the qpoases solver.
	acado_solver:		Solves the given mpc problem.
	test.c:				A test file to test acado without ros.
```
Run make to make the testfile.


## Starting MPC
Run the following commands to run the mpc
```
	source start.sh
	roslaunch px4_gaz Simulate.launch
	commander takeoff
	rosrun trajectory_optimsation mpc_controller_node
```
For python script for non-mpc controls testing run
do
```
	source start.sh
	roslaunch px4_gaz Simulate.launch 
	commander takeoff
	rosrun motion_planning motionplanner.py
```
motion_planning_1 package is the prelimanry version for testing.

## Known Issues
### ROS
1.	If you face problems in connecting to mavlink in your pc, you should check that fcu_url is correct. https://dev.px4.io/en/simulation/ros_interface.html
2.	If you face an error in roslaunch 
```
	terminate called after throwing an instance of 'std::runtime_error'
  	what():  Time is out of dual 32-bit range
```
ignore it an run again and again until the gazebo is launched (however if somebody manages to figure out a permanent solution do mail at gauravs123456789@gmail.com or send a PR)
3.	Very low fps in gazebo: Disable the lidar sensor from the gazebo model.
4.	If you're facing to integrate codegen, qpoases and the controller, maybe your CMakeLists.txt in your package may be incorrect and look at the CMakeLists.txt present in this repository.


### Logical Errors
1.	Ignore the .yN reference that acado generates in the test.c and comment it out, instead change 
```
	for (i = 0; i < NY * (N); ++i)
		{
			acadoVariables.y[ i ] = 0.0;
			if (i%NY==6)
			{
		  		acadoVariables.y[ i ] = 1.0;
			}
			if (i%NY==2)
			{
		  		acadoVariables.y[ i ] = 0.0+(float)i/100;
		  		printf("%d,%f\n", (i-2)/10,acadoVariables.y[ i ]);
		  	}
		} 
```
to (Change NY\*N->NY\*(N+1))
```
	for (i = 0; i < NY * (N+1); ++i)
		{
			acadoVariables.y[ i ] = 0.0;
			if (i%NY==6)
			{
		  		acadoVariables.y[ i ] = 1.0;
			}
			if (i%NY==2)
			{
		  		acadoVariables.y[ i ] = 0.0+(float)i/100;
		  		printf("%d,%f\n", (i-2)/10,acadoVariables.y[ i ]);
		  	}
		}
```
2.	Check whether that in your model constraints are correctly defined, incorrect contraints may lead to a never feasible path

3.	Ensure that you're giving sufficient time to execute the trajectory. t_end(in simple_ocp_2.cpp) should have sufficient value. 

4.	If you're facing problem in integrating qpOASES blindly copy the qpOASES present in this repository. However, suggestion is to try it yourself

5.	If you're facing problem in integrating ACADO MPC CODEGEN blindly copy the ACADO MPC CODEGEN present in this repository

### ACADO Changes
1.	For small 2-3 decimal numbers you can change line number 66 and 81 from %e to %f for better viewing. 

## New issues
If you face any issue kindly create a new issue in github and mail me specifically at gauravs123456789@gmail.com 

Thankyou
