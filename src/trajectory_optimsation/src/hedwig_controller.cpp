#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include <hedwig_controller.h>

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"

#include <std_msgs/Float64.h>
#include <sstream>
#include <math.h>

/* Some convenient definitions. */
#define NX		 ACADO_NX  /* Number of differential state variables.  */
#define NXA		 ACADO_NXA /* Number of algebraic variables. */
#define NU		 ACADO_NU  /* Number of control inputs. */
#define NP		 ACADO_NP  /* Number of parameters. */

#define NY		 ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN		 ACADO_NYN /* Number of measurements/references on node N. */

#define N		 ACADO_N		/* Number of intervals in the horizon. */

#define NUM_STEPS   10		 /* Number of real-time iterations. */
#define VERBOSE	 	0		 /* Show iterations: 1, silent: 0.  */

using namespace std;
using namespace hedwig_mpc;

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

class htrajectory
{
public:
	int i,iter;
	acado_timer t;
	hedwig_command command;
	hedwig_state state;


	htrajectory(geometry_msgs::PoseStamped hstat, geometry_msgs::TwistStamped velocit,geometry_msgs::Vector3 hbody_rates1, float hthrust1)
	{
		command.set_command(hbody_rates1,hthrust1);	//Initial Command
		state.set_state(hstat,velocit);	//Inital State
	}

	htrajectory(geometry_msgs::PoseStamped hstat, geometry_msgs::TwistStamped velocit)
	{
		state.set_state(hstat,velocit);
		float t[3]={0,0,0};
		float th=0.3;
		command.set_thrust(th);
		command.set_angle(t);
	}
	htrajectory(){};

	void set_hover_state(geometry_msgs::PoseStamped hstat, geometry_msgs::TwistStamped velocit)
	{
		state.set_state(hstat,velocit);
		float t[3]={0,0,0};
		float th=0.3;	//Modify this thrust value to hover the quad
		command.set_thrust(th);
		command.set_angle(t);
	}

	void set_hover_state()
	{
		float t[3]={0,0,0};
		float th=0.3;	//Modify this thrust value to hover the quad
		command.set_thrust(th);
		command.set_angle(t);
	}


	void set_commandnstate(hedwig_command comm, hedwig_state stat)
	{
		command=comm;					//if avaliable in this format
		state=stat;
	}

	void set_commandnstate(geometry_msgs::PoseStamped hstat, geometry_msgs::TwistStamped velocit,geometry_msgs::Vector3 hbody_rates1, float hthrust1)
	{
		command.set_command(hbody_rates1,hthrust1);	
		state.set_state(hstat,velocit);	
	}

	float get_thrust()
	{
		return command.hthrust;
	}

	geometry_msgs::Vector3 get_bodyrates()
	{
		return command.hbody_rates;
	}

	geometry_msgs::Quaternion get_orientation()
	{
		return command.horientation;
	}

	void initialize()
	{
		/* Clear solver memory. */
		memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
		memset(&acadoVariables, 0, sizeof( acadoVariables ));
	
		/* Initialize the solver. */
		acado_initializeSolver();

		/* Initialize the states and controls. Can be random */
		//px, py, pz, vx, vy, vz, qw, qx, qy, qz; 

		for (i = 0; i < NX * (N + 1); ++i)
		{
			acadoVariables.x[ i ] = 0.0; 		//States
			if (i%NX==6)
			{
				acadoVariables.x[ i ] = 1.0;
		  	}
		}
		for (i = 0; i < NU * N; ++i)  
			acadoVariables.u[ i ] = 1.0;	//Controls
		
		/* Initialize the measurements/reference. */
		// printf("ACADO_NY %d,%d,%d\n",NY,NYN,N );
		for (i = 0; i < NY * (N); ++i)
		{
			switch(i%NY)
			{
				case 0: //px
						acadoVariables.y[i] = 0;
						break;
				case 1:	//py 
						acadoVariables.y[i] = 0;
						break;
				case 2:	//pz 
						acadoVariables.y[i] = 5;
						break;
				case 3:	//vx 
						acadoVariables.y[i] = 0;
						break;
				case 4:	//vy
						acadoVariables.y[i] = 0;
						break;
				case 5:	//vz 
						acadoVariables.y[i] = 0;
						break;
				case 6:	//qw 
						acadoVariables.y[i] = 1;
						break;
				case 7:	//qx 
						acadoVariables.y[i] = 0;
						break;
				case 8:	//qy
						acadoVariables.y[i] = 0;
						break;
				case 9:	//qz
						acadoVariables.y[i] = 0;
						break;
					
			}
		}

		for (i = 0; i < NYN; ++i)  
		{
			switch(i%NYN)
			{
				case 0: //px
						acadoVariables.yN[i] = 0;
						break;
				case 1:	//py 
						acadoVariables.yN[i] = 0;
						break;
				case 2:	//pz 
						acadoVariables.yN[i] = 5;
						break;
				case 3:	//vx 
						acadoVariables.yN[i] = 0;
						break;
				case 4:	//vy
						acadoVariables.yN[i] = 0;
						break;
				case 5:	//vz 
						acadoVariables.yN[i] = 0;
						break;
				case 6:	//qw 
						acadoVariables.yN[i] = 1;
						break;
				case 7:	//qx 
						acadoVariables.yN[i] = 0;
						break;
				case 8:	//qy
						acadoVariables.yN[i] = 0;
						break;
				case 9:	//qz
						acadoVariables.yN[i] = 0;
						break;
					
			}
		}

		/* MPC: initialize the current state feedback. */
		for (i = 0; i < NX; i++)
		{ 
			switch(i%NX)
			{
				case 0: 
						acadoVariables.x0[i] = state.hPosX;
						break;
				case 1: 
						acadoVariables.x0[i] = state.hPosY;
						break;
				case 2: 
						acadoVariables.x0[i] = state.hPosZ;
						break;
				case 3: 
						acadoVariables.x0[i] = state.hOriW;
						break;
				case 4: 
						acadoVariables.x0[i] = state.hOriX;
						break;
				case 5: 
						acadoVariables.x0[i] = state.hOriY;
						break;
				case 6: 
						acadoVariables.x0[i] = state.hOriZ;
						break;
				case 7: 
						acadoVariables.x0[i] = state.hVelX;
						break;
				case 8: 
						acadoVariables.x0[i] = state.hVelY;
						break;
				case 9: 
						acadoVariables.x0[i] = state.hVelZ;
						break;
					
			}
		}


	}

	void run_mpc()
	{
		/* Prepare first step */
		acado_preparationStep();

		/* Get the time before start of the loop. */
		acado_tic( &t );

		/* The "real-time iterations" loop. */
		for(iter = 0; iter < NUM_STEPS; ++iter)
		{
			/* Perform the feedback step. */
			acado_feedbackStep();

			/* Apply the new control immediately to the process, first NU components. */

			if( VERBOSE ) printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT() );

			/* Optional: shift the initialization (look at acado_common.h). */
			/* acado_shiftStates(2, 0, 0); */
			/* acado_shiftControls( 0 ); */

			/* Prepare for the next step. */
			acado_preparationStep();
		}
		/* Read the elapsed time. */
		real_t te = acado_toc( &t );

		if( VERBOSE ) 
		{
			printf("\n\nEnd of the RTI loop. \n\n\n");
			printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te / NUM_STEPS);
		}

		acado_printDifferentialVariables();
		acado_printControlVariables();
		
		// float THRUST=acadoVariables.u[0];
		float THRUST=0.5;
		float ANGLE[3]={acadoVariables.u[1],acadoVariables.u[2],acadoVariables.u[3]};
		command.set_thrust(THRUST);
		command.set_angle(ANGLE);
	}

};

void Find_bot(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	hpos=msg;
	FIRST_RUN1=true;
}
void Find_bot2(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	hvel=msg;
	FIRST_RUN2=true;
}	

void get_imu_data(const sensor_msgs::Imu::ConstPtr& msg)
{
	FIRST_RUN3=true;
	himu=msg;
}

void get_status(const mavros_msgs::State::ConstPtr& msg)
{
	FIRST_RUN4=true;
	hstatus=msg;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "MPC");
	ros::NodeHandle nh;

	ros::Subscriber sub1= nh.subscribe("/mavros/local_position/pose",1,Find_bot);
	ros::Subscriber sub2= nh.subscribe("/mavros/local_position/velocity",1,Find_bot2);
	ros::Subscriber sub3= nh.subscribe("/mavros/imu/data",1,get_imu_data);
	ros::Subscriber sub4= nh.subscribe("/mavros/state",1,get_status);


	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    pub_controls = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);

	



	// image_transport::ImageTransport it(nh);
	// image_transport::Subscriber sub = it.subscribe("/iarc/camera/right/image_raw",1,input_cam/*,ros::VoidPtr(),image_transport::TransportHints("compressed")*/);
	// image_transport::Subscriber sub = it.subscribe("/bebop/image_raw",1,input_cam/*,ros::VoidPtr(),image_transport::TransportHints("compressed")*/);
	// pub_thrust = nh.advertise<mavros_msgs::Thrust>("/mavros/setpoint_attitude/thrust", 1);
	// pub_angles = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude/cmd_vel", 1);
	
	ros::Rate rate(60.0);

	bool LOOP_START=false;
	FIRST_RUN1=FIRST_RUN2=FIRST_RUN3=FIRST_RUN4=false;


	htrajectory HTraj;
	hthnatt_estimator HESTIMATOR;
	hedwig_state HESTATE; //Hedwig Estimator State

	while (ros::ok())
	{
		if(FIRST_RUN1 && FIRST_RUN2 && HESTIMATOR.success==false)
		{
			HESTATE.set_state(*hpos,*hvel);
			HESTIMATOR.hestate=HESTATE;
			HESTIMATOR.status=*hstatus;
			HESTIMATOR.himu=*himu;
			HESTIMATOR.set_mode=set_mode_client;
			HESTIMATOR.set_arm= arming_client;

			if(HESTIMATOR.preflightcheck==false)
				HESTIMATOR.init();
			if(HESTIMATOR.preflightcheck==true)
			{
				HESTIMATOR.estimatecm();
				pub_controls.publish(HESTIMATOR.COMMAND);
			}
			
		}
		if(HESTIMATOR.success==true)
		{
			if (FIRST_RUN1 && FIRST_RUN2)
			{
				HTraj.set_hover_state(*hpos,*hvel);
				LOOP_START=true;
			}
			if(LOOP_START)
			{
				HTraj.initialize();
				HTraj.run_mpc();

				THRUST 		=HTraj.get_thrust();
				BODY_RATES	=HTraj.get_bodyrates();
				ORIENT 		=HTraj.get_orientation();
				
				COMMAND.thrust=THRUST;
				COMMAND.body_rate=BODY_RATES;
				COMMAND.orientation=ORIENT;

				COMMAND.header.frame_id="";

				pub_controls.publish(COMMAND);
			}
		}
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}