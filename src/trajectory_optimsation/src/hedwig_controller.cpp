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
#include "mavros_msgs/Thrust.h"
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
#define VERBOSE	 	1		 /* Show iterations: 1, silent: 0.  */

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
	/* Global variables used by the solver. */

	htrajectory(geometry_msgs::PoseStamped hstat, geometry_msgs::TwistStamped velocit,geometry_msgs::TwistStamped geo, mavros_msgs::Thrust Th)
	{
		command.set_command(geo,Th);	//Initial Command
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

	void set_command(geometry_msgs::PoseStamped hstat, geometry_msgs::TwistStamped velocit)
	{
		state.set_state(hstat,velocit);
		float t[3]={0,0,0};
		float th=0.3;
		command.set_thrust(th);
		command.set_angle(t);
	}
	htrajectory(){};
	void set_commandnstate(hedwig_command comm, hedwig_state stat)
	{
		command=comm;					//if avaliable in this format
		state=stat;
	}

	void set_commandnstate(geometry_msgs::PoseStamped hstat, geometry_msgs::TwistStamped velocit,geometry_msgs::TwistStamped geo, mavros_msgs::Thrust Th)
	{
		command.set_command(geo,Th);	//Updated Command
		state.set_state(hstat,velocit);	//Updated State
	}

	void initialize()
	{
		/* Clear solver memory. */
		memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
		memset(&acadoVariables, 0, sizeof( acadoVariables ));
	
		/* Initialize the solver. */
		acado_initializeSolver();

		/* Initialize the states and controls. */
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
		{
			acadoVariables.u[ i ] = 1.0;	//Controls
		}

		/* Initialize the measurements/reference. */
		// printf("ACADO_NY %d,%d,%d\n",NY,NYN,N );
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
		// for (i = 0; i < NYN; ++i)  
		// {
		// 	acadoVariables.y[ i ] = 0.0;
		// 	if (i%NY==6)
		// 	{
		// 		acadoVariables.y[ i ] = 1.0;
		// 	}
		// 	if (i%NY==2)
		// 	{
		// 		acadoVariables.y[ i ] = 2.0;
		// 	}
		// }

		/* MPC: initialize the current state feedback. */
		#if 1
			for (i = 0; i < NX; i++)
			{ 
				acadoVariables.x0[ i ] = 0.0; 		//States
		  		if (i%NX==6)
		  		{
		  			acadoVariables.x0[ i ] = 1.0;
		  		}
			}
		#endif
		if( VERBOSE ) acado_printHeader();

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

		if( VERBOSE ) printf("\n\nEnd of the RTI loop. \n\n\n");

		/* Eye-candy. */

		if( !VERBOSE )
		printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te / NUM_STEPS);

		acado_printDifferentialVariables();
		acado_printControlVariables();
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


int main(int argc, char** argv)
{
	ros::init(argc, argv, "MPC");
	ros::NodeHandle nh;

	ros::Subscriber sub1= nh.subscribe("/mavros/local_position/pose",1,Find_bot);
	ros::Subscriber sub2= nh.subscribe("/mavros/local_position/velocity",1,Find_bot2);


	FIRST_RUN1=false;
	FIRST_RUN2=false;

	htrajectory HTraj;

	// image_transport::ImageTransport it(nh);
	// image_transport::Subscriber sub = it.subscribe("/iarc/camera/right/image_raw",1,input_cam/*,ros::VoidPtr(),image_transport::TransportHints("compressed")*/);
	// image_transport::Subscriber sub = it.subscribe("/bebop/image_raw",1,input_cam/*,ros::VoidPtr(),image_transport::TransportHints("compressed")*/);
	// pub_thrust = nh.advertise<mavros_msgs::Thrust>("/mavros/setpoint_attitude/thrust", 1);
	// pub_angles = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude/cmd_vel", 1);

	ros::Rate rate(10.0);
	bool LOOP_START=false;

	while (ros::ok())
	{
		if (FIRST_RUN1 && FIRST_RUN2)
		{
			HTraj.set_command(*hpos,*hvel);
			LOOP_START=true;
		}
		if(LOOP_START)
		{
			HTraj.initialize();
			HTraj.run_mpc();
		}


		// geometry_msgs::PoseArray Bots;
		// geometry_msgs::Pose Bot;
		// Bots.header.stamp = ros::Time::now();
		// /*
		// std_msgs::Float64MultiArray botx;
		// std_msgs::Float64MultiArray boty;
		// std_msgs::Float64MultiArray botmode;
		// std_msgs::Float64MultiArray botr;
		// std_msgs::Float64MultiArray bottheta;
		// */
		// std_msgs::Int32 total_bots;

		// total_bots.data=total;
		// //botx.data.clear();
		// //boty.data.clear();
		// //botmode.data.clear();
		// //botr.data.clear();
		// //bottheta.data.clear();
		// Bots.poses.clear();

		// for(int i=0;i<total;i++)
		// {
		//   //cout<<endl<<x[i]<<" "<<y[i]<<" "<<mode[i]<<" "<<r[i]<<" "<<xo[i]<<" "<<yo[i]<<" "<<total<<endl;
		//   Bot.position.x=x[i];
		//   Bot.position.y=y[i];
		//   if(check==1)
		//   {
		// 	Bot.position.z=mode[i];
		// 	Bot.orientation.x=xo[i];
		// 	Bot.orientation.y=yo[i];
		// 	Bot.orientation.z=0;
		// 	Bot.orientation.w=r[i];
		//   }
		//   Bots.poses.push_back(Bot);
		//  // botx.data.push_back(x[i]);
		//  // boty.data.push_back(y[i]);
		//  // botmode.data.push_back(mode[i]);
		// //  botr.data.push_back(r[i]);
		//  // bottheta.data.push_back(theta[i]);
		// }

		// bot_detect.publish(Bots);
		// bot_total.publish(total_bots);


		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}