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
```

3.	

```
cd examples/my_examples
./simple_ocp_2.cpp
```

4.	
```
	cd ark_mpc
``` 
This folder contains the codegen.

5.	Modify acado_auxilary_functions.c to file in this repository
6.	Run

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
MPC is an non-linear optimsation problem and a specific case of Trajectory Optimisation. It gives us the optimal input controls to achieve a specific state under quadrotor constraints. It is a cost function minimization problem. 

This paper is a good way to understand MPC,(ignore the preception part) https://arxiv.org/pdf/1804.04811.pdf?fbclid=IwAR2SBQ6v0qqDQsPWLyUfMYM5nyR6wxp7mfWfFEFJ4a3CWVCeVS1pbHGhJSc

#### Our Model
Implemention of MPC was done to improvise quad localisation and counter high speed winds. Winds were modelled in form of differential equations and were added it to the states wherever it affected. 

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
Hence, our non linear cost function is
<img src="https://latex.codecogs.com/gif.latex?\int_{t=0}^{t=t} A_1\times(px-pxr)^2+A_2 \times (py-pyr)^2+\ldots+A_9 \times(qz-qzr)^2 dt" /> 

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
	INTEGRATOR_TYPE 		= 		INT_RK4 
    NUM_INTEGRATOR_STEPS 	= 		30 
    HESSIAN_APPROXIMATION	= 		GAUSS_NEWTON 
```

v1.0 Consists of the basic mathematical model of Quadcoptor without integration of ROS 

###	Acado
Acado stands for Automatic Control and Dynamic Optimization. ACADO Toolkit is a software environment and algorithm collection for automatic control and dynamic optimization. It provides a general framework for using a great variety of algorithms for direct optimal control, including model predictive control, state and parameter estimation and robust optimization. ACADO Toolkit is implemented as self-contained C++ code.

https://www.youtube.com/watch?v=JC6PNjndQ_c&t=2724s link is a good implemention of ACADO if you're well versed with MPC.




do
```
	source start.sh

	roslaunch px4_gaz Simulate.launch 
	rosrun motion_planning motionplanner.py
```

motion_planning_1 is the prelimanry version for testing
trajectory optimsation is the 2nd version

If you face problems in connecting to mavlink in your pc, you should check that fcu_url is correct. https://dev.px4.io/en/simulation/ros_interface.html
