#include <thread>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/Thrust.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <list>
#include "trajectory_optimsation/mpc_wrapper.h"
#include "trajectory_optimsation/hedwig_mpc_params.h"
#include "trajectory_optimsation/geometry_eigen_conversions.h"

using std::list;

namespace hedwig_mpc 
{

	enum STATE 
	{
		kPosX = 0,
		kPosY = 1,
		kPosZ = 2,
		kOriW = 3,
		kOriX = 4,
		kOriY = 5,
		kOriZ = 6,
		kVelX = 7,
		kVelY = 8,
		kVelZ = 9
	};

	enum INPUT
	{
		kThrust = 0,
		kRateX = 1,
		kRateY = 2,
		kRateZ = 3
	};

	class hedwig_command
	{
	public:
		geometry_msgs::TwistStamped geometry;
		geometry_msgs::PoseStamped hposition;
		mavros_msgs::Thrust lift;
		ros::Time timestamp;
		bool armed;

		hedwig_command(){}

		hedwig_command(geometry_msgs::TwistStamped geo, mavros_msgs::Thrust Th)
		{
			geometry=geo;
			lift=Th;
			armed=false;
		}
		hedwig_command(geometry_msgs::TwistStamped geo, mavros_msgs::Thrust Th, geometry_msgs::PoseStamped posit)
		{
			geometry=geo;
			lift=Th;
			hposition=posit;
			armed=false;
		}

		void zero()
		{
			lift.thrust=0;
			geometry.twist.angular.x=0;
			geometry.twist.angular.y=0;
			geometry.twist.angular.z=0;
			geometry.linear.angular.x=0;
			geometry.linear.angular.y=0;
			geometry.linear.angular.z=0;
		}
	}

	class hedwig_state
	{
	public:
		geometry_msgs::PoseStamped hstate;
		geometry_msgs::TwistStamped hvel;
		hedwig_state(geometry_msgs::PoseStamped hstat, geometry_msgs::TwistStamped velocit)
		{
			hstate=hstat;
			hvel=velocit;
		}
	}

	class hedwig_Trajectory_point
	{
	public:
		ros::duration time_from_start;

		geometry_msgs::Pose GGtrajpt;//Geometry message form
		geometry_msgs::Twist Gvelocity;
		geometry_msgs::Twist Gacceleration;
		float heading;
		float heading_rate;
		float heading_acceleration;

		//pose
		Eigen::Vector3d position;//normal form
		Eigen::Quaterniond orientation;

		// Linear derivatives
		Eigen::Vector3d velocity;
		Eigen::Vector3d acceleration;
		Eigen::Vector3d jerk;
		Eigen::Vector3d snap;

		// Angular derivatives
		Eigen::Vector3d bodyrates;
		Eigen::Vector3d angular_acceleration;
		Eigen::Vector3d angular_jerk;
		Eigen::Vector3d angular_snap;

		hedwig_Trajectory_point()
		{}
		hedwig_Trajectory_point(ros::duration a,geometry_msgs::Pose b, geometry_msgs::Twist c,geometry_msgs::Twist d,float g[3])
		{
			time_from_start=a;
			Gtrajpt=b;
			Gvelocity=c;
			Gacceleration=d;
			heading=g[0];
			heading_rate=g[1];
			heading_acceleration=g[2];
			this.convert();
		}
		void convert()
		{
			position = geometryToEigen(Gtrajpt.position);
			orientation = geometryToEigen(Gtrajpt.pose.orientation);

			velocity = geometryToEigen(Gvelocity.linear);
			acceleration = geometryToEigen(Gacceleration.linear);
			// jerk = geometryToEigen(trajectory_point_msg.jerk.linear);
			// snap = geometryToEigen(trajectory_point_msg.snap.linear);

			bodyrates = geometryToEigen(Gvelocity.angular);
			angular_acceleration = geometryToEigen(Gacceleration.angular);
			// angular_jerk = geometryToEigen(trajectory_point_msg.jerk.angular);
			// angular_snap = geometryToEigen(trajectory_point_msg.snap.angular);

			// heading = trajectory_point_msg.heading;
			// heading_rate = trajectory_point_msg.heading_rate;
			// heading_acceleration = trajectory_point_msg.heading_acceleration;
		}
	}	

	class hedwig_Trajectory
	{
	public:
		int points;
		// hedwig_Trajectory_point *Traj;
		// hedwig_Trajectory(int a)
		// {
		// 	points=a;
		// 	traj=new hedwig_Trajectory_point(a);
		// }
		// void set_point(int index,ros::duration a,geometry_msgs::Pose b, geometry_msgs::Twist c,geometry_msgs::Twist d,float g[3])
		// {
		// 	traj[index]=new hedwig_Trajectory_point(a,b,c,d,g)
		// }
		list<hedwig_Trajectory_point> Traj;
		hedwig_Trajectory(int a)
		{
			points=0;
		} 
		void set_points(int index,ros::duration a,geometry_msgs::Pose b, geometry_msgs::Twist c,geometry_msgs::Twist d,float g[3])
		{
			points++;
			hedwig_Trajectory_point temp=new hedwig_Trajectory_point(a,b,c,d,g)
			Traj.push_back(temp);
		}
	}

template <typename T>
	class MpcController 
	{
	public:

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		static_assert(kStateSize == 10,
			"MpcController: Wrong model size. Number of states does not match.");
		static_assert(kInputSize == 4,
			"MpcController: Wrong model size. Number of inputs does not match.");

		MpcController(const ros::NodeHandle & nh, const ros::NodeHandle & pnh);
		MpcController() : MpcController(ros::NodeHandle(), ros::NodeHandle("~")) {}

    	// attitude set an off command
		hedwig_command off();

    	//Running the command
		hedwig_command run(
      const quadrotor_common::QuadStateEstimate& state_estimate, //Change it in accordance with hedwig
      const quadrotor_common::Trajectory& reference_Trajectory,//Change it in accordance with hedwig
      const MpcParams<T>& params); //Define a parameter file


	private:


	    //Not Required
		void pointOfInterestCallback(
			const geometry_msgs::PointStamped::ConstPtr& msg);

    	//Given by hedwig
		bool setStateEstimate(
      const quadrotor_common::QuadStateEstimate& state_estimate); //Change it in accordance with hedwig

    	//Given by hedwig
    	bool setReference(const quadrotor_common::Trajectory& reference_Trajectory);//change quadrotor common in accordance with hedwig

    	//Command which we have to give to ros
	    hedwig_command updateControlCommand(
	    	const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
	    	const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
	      ros::Time& time);//input state and input and give out geometry msgs

	    //Publisher
	    bool publishPrediction(
	    	const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples+1>> states,
	    	const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs,
	    	ros::Time& time);

	    //Started Preparation_thread
	    void preparationThread();

	    //A function of set new params, which sets the mpc params from the file. (mpcparams.h)
    	bool setNewParams(MpcParams<T>& params);

	    // Handles
	    ros::NodeHandle nh_;
	    ros::NodeHandle pnh_;

	    // Subscribers and publisher.
	    ros::Subscriber sub_point_of_interest_;
	    ros::Publisher pub_predicted_Trajectory_;

	    // Parameters
	    MpcParams<T> params_;

	    // MPC
		MpcWrapper<T> mpc_wrapper_;

	    // Preparation Thread
	    std::thread preparation_thread_;

	    // Variables
	    T timing_feedback_, timing_preparation_;
	    Eigen::Matrix<T, kStateSize, 1> est_state_;
	    Eigen::Matrix<T, kStateSize, kSamples+1> reference_states_;
	    Eigen::Matrix<T, kInputSize, kSamples+1> reference_inputs_;
	    Eigen::Matrix<T, kStateSize, kSamples+1> predicted_states_;
	    Eigen::Matrix<T, kInputSize, kSamples> predicted_inputs_;
	    Eigen::Matrix<T, 3, 1> point_of_interest_;
};
