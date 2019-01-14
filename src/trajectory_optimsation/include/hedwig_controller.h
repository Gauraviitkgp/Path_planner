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
#include <ros/ros.h>
// #include "trajectory_optimsation/mpc_wrapper.h"
// #include "trajectory_optimsation/hedwig_mpc_params.h"

#include "geometry_eigen_conversions.h"

using std::list;

namespace hedwig_mpc 
{
	geometry_msgs::TwistStamped::ConstPtr hvel;
	geometry_msgs::PoseStamped::ConstPtr hpos;
	bool FIRST_RUN1;
	bool FIRST_RUN2;

	ros::Publisher pub_thrust;
	ros::Publisher pub_angles;
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
		geometry_msgs::TwistStamped hgeometry;
		geometry_msgs::PoseStamped hposition;
		mavros_msgs::Thrust lift;
		ros::Time timestamp;
		bool armed;

		hedwig_command(){}

		hedwig_command(geometry_msgs::TwistStamped geo, mavros_msgs::Thrust Th)
		{
			hgeometry=geo;
			lift=Th;
			armed=false;
		}
		hedwig_command(geometry_msgs::TwistStamped geo, mavros_msgs::Thrust Th, geometry_msgs::PoseStamped posit)
		{
			hgeometry=geo;
			lift=Th;
			hposition=posit;
			armed=false;
		}

		void set_command(geometry_msgs::TwistStamped geo, mavros_msgs::Thrust Th)
		{
			hgeometry=geo;
			lift=Th;
			armed=false;
		}

		void zero()
		{
			lift.thrust=0;
			hgeometry.twist.angular.x=0;
			hgeometry.twist.angular.y=0;
			hgeometry.twist.angular.z=0;
			hgeometry.twist.linear.x=0;
			hgeometry.twist.linear.y=0;
			hgeometry.twist.linear.z=0;
		}
		void set_thrust(float th)
		{
			lift.thrust=th;
			lift.header.frame_id="map";
			lift.header.stamp=ros::Time::now();
			timestamp=ros::Time::now();
		}
		void set_angle(float th[3])
		{
			hgeometry.header.frame_id="map";
			lift.header.stamp=ros::Time::now();
			timestamp=ros::Time::now();
			hgeometry.twist.angular.x=th[0];
			hgeometry.twist.angular.y=th[1];
			hgeometry.twist.angular.z=th[2];
			hgeometry.twist.linear.x=0;
			hgeometry.twist.linear.y=0;
			hgeometry.twist.linear.z=0;
		}
	};

	class hedwig_state
	{
	public:
		geometry_msgs::PoseStamped hstate;
		geometry_msgs::TwistStamped hvel;
		ros::Time timestamp;
		float hPOSX,hPOSY,hPOSZ,hOriW,hOriX,hOriY,hOriZ,hVelX,hVelY,hVelZ;

		hedwig_state(geometry_msgs::PoseStamped hstat, geometry_msgs::TwistStamped velocit)
		{
			hstate=hstat;
			hvel=velocit;
			
			timestamp=hstate.header.stamp;

			hPOSX=hstate.pose.position.x;
			hPOSY=hstate.pose.position.y;
			hPOSZ=hstate.pose.position.z;
			
			hOriW=hstate.pose.orientation.w;
			hOriX=hstate.pose.orientation.x;
			hOriY=hstate.pose.orientation.y;
			hOriZ=hstate.pose.orientation.z;

			hVelX=hvel.twist.linear.x;
			hVelY=hvel.twist.linear.y;
			hVelZ=hvel.twist.linear.z;
		}

		hedwig_state(){};

		void set_state(geometry_msgs::PoseStamped hstat, geometry_msgs::TwistStamped velocit)
		{
			hstate=hstat;
			hvel=velocit;
			
			timestamp=hstate.header.stamp;

			hPOSX=hstate.pose.position.x;
			hPOSY=hstate.pose.position.y;
			hPOSZ=hstate.pose.position.z;
			
			hOriW=hstate.pose.orientation.w;
			hOriX=hstate.pose.orientation.x;
			hOriY=hstate.pose.orientation.y;
			hOriZ=hstate.pose.orientation.z;

			hVelX=hvel.twist.linear.x;
			hVelY=hvel.twist.linear.y;
			hVelZ=hvel.twist.linear.z;
		}

	};

	// class hedwig_Trajectory_point
	// {
	// public:
	// 	ros::duration time_from_start;

	// 	geometry_msgs::Pose GGtrajpt;//Geometry message form
	// 	geometry_msgs::Twist Gvelocity;
	// 	geometry_msgs::Twist Gacceleration;
	// 	float heading;
	// 	float heading_rate;
	// 	float heading_acceleration;

	// 	//pose
	// 	Eigen::Vector3d position;//normal form
	// 	Eigen::Quaterniond orientation;

	// 	// Linear derivatives
	// 	Eigen::Vector3d velocity;
	// 	Eigen::Vector3d acceleration;
	// 	Eigen::Vector3d jerk;
	// 	Eigen::Vector3d snap;

	// 	// Angular derivatives
	// 	Eigen::Vector3d bodyrates;
	// 	Eigen::Vector3d angular_acceleration;
	// 	Eigen::Vector3d angular_jerk;
	// 	Eigen::Vector3d angular_snap;

	// 	hedwig_Trajectory_point()
	// 	{}
	// 	hedwig_Trajectory_point(ros::duration a,geometry_msgs::Pose b, geometry_msgs::Twist c,geometry_msgs::Twist d,float g[3])
	// 	{
	// 		time_from_start=a;
	// 		Gtrajpt=b;
	// 		Gvelocity=c;
	// 		Gacceleration=d;
	// 		heading=g[0];
	// 		heading_rate=g[1];
	// 		heading_acceleration=g[2];
	// 		this.convert();
	// 	}
	// 	void convert()
	// 	{
	// 		position = geometryToEigen(Gtrajpt.position);
	// 		orientation = geometryToEigen(Gtrajpt.pose.orientation);

	// 		velocity = geometryToEigen(Gvelocity.linear);
	// 		acceleration = geometryToEigen(Gacceleration.linear);
	// 		// jerk = geometryToEigen(trajectory_point_msg.jerk.linear);
	// 		// snap = geometryToEigen(trajectory_point_msg.snap.linear);

	// 		bodyrates = geometryToEigen(Gvelocity.angular);
	// 		angular_acceleration = geometryToEigen(Gacceleration.angular);
	// 		// angular_jerk = geometryToEigen(trajectory_point_msg.jerk.angular);
	// 		// angular_snap = geometryToEigen(trajectory_point_msg.snap.angular);

	// 		// heading = trajectory_point_msg.heading;
	// 		// heading_rate = trajectory_point_msg.heading_rate;
	// 		// heading_acceleration = trajectory_point_msg.heading_acceleration;
	// 	}
	// };	

	// class hedwig_Trajectory
	// {
	// public:
	// 	int points;
	// 	// hedwig_Trajectory_point *Traj;
	// 	// hedwig_Trajectory(int a)
	// 	// {
	// 	// 	points=a;
	// 	// 	traj=new hedwig_Trajectory_point(a);
	// 	// }
	// 	// void set_point(int index,ros::duration a,geometry_msgs::Pose b, geometry_msgs::Twist c,geometry_msgs::Twist d,float g[3])
	// 	// {
	// 	// 	traj[index]=new hedwig_Trajectory_point(a,b,c,d,g)
	// 	// }
	// 	list<hedwig_Trajectory_point> Traj;
	// 	hedwig_Trajectory(int a)
	// 	{
	// 		points=0;
	// 	} 
	// 	void set_points(int index,ros::duration a,geometry_msgs::Pose b, geometry_msgs::Twist c,geometry_msgs::Twist d,float g[3])
	// 	{
	// 		points++;
	// 		hedwig_Trajectory_point temp=new hedwig_Trajectory_point(a,b,c,d,g)
	// 		Traj.push_back(temp);
	// 	}
	// }
}