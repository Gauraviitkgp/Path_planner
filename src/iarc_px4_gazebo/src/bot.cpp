#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include <string>
#include <iterator>
#include <random>
#include <thread>
#include "tf/transform_datatypes.h"
#include "ros/time.h"
#include <angles/angles.h>
#include <thread>
#include <vector>
#include <stdlib.h>

using namespace std;

#define PI 3.14159
enum State {MOVE,TURN};

class iRobot{
	ros::NodeHandle nh;
	std::default_random_engine generator;
	std::normal_distribution<double> distribution;

public:
	
	double velocity;			// Forward Moving Velocity
	double flipDuration;		// Time to headRotation degree turn
	double noiseDuration;		// Time to random noise turn
	double rotationTime;		// Time taken to turn 
	double maxRotation;			// Max noise angle possible
	int robotID;				// Robot ID Number
	bool turning;				// Flag to tell the current state of bot 
	int time;					// Timer clock of bot
	double topRotation;			// Rotation angle by tap
	double headRotation;		// Rotation angle by head on collision
	double yaw;					// Current Yaw value of the bot
	ros::Publisher cmdVel;		// cmd_vel topic publisher
	ros::Subscriber odom;		// odom subscriber
	ros::Subscriber contactSub;	// cotact subscriber
	State state;				// Current state of bot
	int startTime;				// Start time of bot wrt ROS clock
	bool firstCallBack;			// First odom callback monitor to intialise startTime
	geometry_msgs::Twist Vel;	// cmd_vel msg type
	int prevTime;				// Last time turn was called
	int prevContactTime;		// Last time a contact was made
	int contact;				// Contact type , 1 - head-on 2 - tap 


	// Generates a random number based on gaussian distribution 
	double getRotationAngle(){
		double num = distribution(generator);
		// num += 0.3;	//Bias for low rotation angles
		return (num > 1.0)?maxRotation:(num < -1.0)?-maxRotation:num*maxRotation;
	}

	// Fuction to switch states and publish forward command otherwise
	void start(){
		state = MOVE; // Mode set

		while(nh.ok() and !contact){ // Checks if the bot is in contact of any bot or being tapped

			ros::Rate loop_rate(10);

			// Check for headRotation time
			if(time != prevTime and time % (int)flipDuration == 0 and not turning){
				state = TURN;
				prevTime = time;
				// ROS_INFO("Flip T: %d %d",time,startTime);
				turning = true;
				turnBy(headRotation);
				return;
			}
			else
			//Check for Noise duration
				if(time != prevTime and (time)%(int)noiseDuration == 0 and state == MOVE and not turning){
					state = TURN;
					prevTime = time;
					// ROS_INFO("Noise");
					turning = true;
					turnBy(getRotationAngle(),true);
					return;
				}
			// State shift 
			state = (turning)?TURN:MOVE;
			// Straight moving
			if(state == MOVE){
				Vel.linear.x = velocity;
				Vel.linear.y = 0;
				Vel.linear.z = 0;
				Vel.angular.x = 0;
				Vel.angular.y = 0;
				Vel.angular.z = 0;
				cmdVel.publish(Vel);
			}
			ros::spinOnce();
			loop_rate.sleep();
		}
	}

	// Turn the bot
	void turnBy(double turn_angle, bool noise = false)
	{
	    turning = true;
		
		// Completes any angle rotation in rotationTime but overshoots. Apply PID ??
	    // Vel.angular.z = (turn_angle*PI/180.)/rotationTime ;
	    
		Vel.angular.z = 1.46;	// Around 1-2 degree of rotation error.
	    Vel.linear.x = (noise)?velocity:0.;
		Vel.linear.y = 0;
		Vel.linear.z = 0;
		Vel.angular.x = 0;
		Vel.angular.y = 0;
		turn_angle *= PI/180.;
	    ros::Rate rate(10.0);
	    double last_angle = yaw;
	    double angle = 0;

	    while ((angle < turn_angle) && nh.ok())
	    {
	        cmdVel.publish(Vel);
	        ros::spinOnce();
	        rate.sleep();

	        // Compute the amount of rotation since the last loop
	        angle += angles::normalize_angle(yaw - last_angle);
	        last_angle = yaw;
	    }

	    Vel.angular.z = 0;
	    cmdVel.publish(Vel);

	    turning = false;
	    start(); 				// Hand back control to Move 
	}


	// updates bot timer
	void timer(){
		while(nh.ok()){
			ros::Rate loop_rate(10);

			if(firstCallBack) // If the startTime is not initialised skip
				continue;

			if(!turning)	  // Update timer of the bot
				time = ros::Time::now().toSec() - startTime;
			else			// If turning , stop clock update and update startTime to maintain the timer at steady state once turning is done
				startTime = ros::Time::now().toSec() - time ;

			loop_rate.sleep();
		}
	}


	// update current yaw value of the bot
	void OdomCallBack(const nav_msgs::Odometry::ConstPtr& msg){
		
		if(firstCallBack){		// If first callback of the yaw state
			firstCallBack = false;
			startTime = ros::Time::now().toSec();		// start the bot and initialise startTime
		}	

		tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
	    tf::Matrix3x3 m(q);
	    double roll, pitch;

	    m.getRPY(roll, pitch, yaw);					// Get yaw value
	}

	// Contact string callback
	void ContactCallBack(const std_msgs::String::ConstPtr& msg){
		
		// Read string with contact condition of all bots and pick the robotID status
		contact = (msg->data)[robotID] - '0';

		if(contact and prevContactTime != time){
			prevContactTime = time;
			if(contact == 1)
				turnBy(headRotation);		// head collision
			else if(contact == 2)
				turnBy(topRotation);		// Tapping
		}
	}

	iRobot(string ID,ros::NodeHandle n){
		nh = n;
		distribution = std::normal_distribution<double>(0.0,1.0);	// Normal distribution with 0 mean 1 variance
		nh.param("x_velocity",velocity,0.33);						// Initialise forward velocity with 0.33 default if parameter is not passed
		nh.param("straight_duration",flipDuration,20.);				// Initialise 180 turn duration with 20 default if parameter is not passed
		nh.param("noise_duration",noiseDuration,5.);				// Initialise noise duration with 5 default if parameter is not passed
		nh.param("rotation_time",rotationTime,2.15);				// Initialise turn duration with 2.15s default if parameter is not passed
		nh.param("rotation_time",maxRotation,10.);					// Initialise maximum noise angle with 20 default if parameter is not passed
		state = MOVE;
		robotID = ID.back() - '0';									// string to integer
		topRotation = 45.;
		headRotation = 180;
		yaw = 0.;
		turning = false;
		time = 0;
		startTime = 0;
		contact = 0;
		prevContactTime = -1;										
		firstCallBack = true;
		prevTime = 0;

		// Publisher and Subscriber Initialisations
		cmdVel = nh.advertise<geometry_msgs::Twist>("/"+ID+"/cmd_vel", 1);
		odom = nh.subscribe("/"+ID+"/odom", 10, &iRobot::OdomCallBack,this);
		contactSub = nh.subscribe("/contact", 10, &iRobot::ContactCallBack,this);

		// Start Timer
		std::thread timerThread(&iRobot::timer,this);
		timerThread.detach();		// Deattach thread
	}
};

bool isActive = false;

void ActivateCallBack(const std_msgs::Bool::ConstPtr& activate){
	isActive = activate->data;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "groundbot_node");
	ros::NodeHandle n("~");
	string ID,def = "robot5";
	ros::Subscriber active;
	active = n.subscribe("/activate", 10, ActivateCallBack);
	n.param("robotID",ID,def);		// Read robot ID (robotX : x being the robot number) defaults to 5
	iRobot bot(ID,n);
	bool running = false;
	while(n.ok()){
		if(isActive and not running){
			bot.start();
		}
		ros::spinOnce();
	}
	return 0;
}