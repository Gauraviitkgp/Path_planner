/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
sensor_msgs::Joy joyvar;
mavros_msgs::State current_state;
sensor_msgs::Imu imuvar;
mavros_msgs::AttitudeTarget att_tgt;
bool fl_takeoff=false;
bool fl_manual=false ;
bool fl_auto=false ;
bool fl_no_sig=true ;
bool fl_land=false ;
bool fl_kill=false;

float q[4], q_tgt[4];
float yaw_ang, yaw_tgt, pitch_ang, roll_ang;
float dt;
float rad = 3.0;            // radius of circle
float cent_x = 0.0;         // x coordinate of centre of circle
float cent_y = 0.0;         // y coordinate of entre of circle
float ang =0.0;             // angular position
float step = 0.01;          // angular rate
float x_set = 0.0;          // x coordinate set point
float y_set = 0.0;          // y coordinate set point
float z_set = 3.0;          // z coordinate set point
float eps = 0.2;            // allowed error in x and y coordinate
float eps_z = z_set*0.1;    //allowed error in z coordinate

nav_msgs::Odometry pos_feed;

/*void yawfn(const sensor_msgs::Imu::ConstPtr& imu_data)
{imuvar= *imu_data;
q[0] = imuvar.orientation.w;
    q[1] = imuvar.orientation.x;
    q[2] = imuvar.orientation.y;
    q[3] = imuvar.orientation.z;
yaw_ang=atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
pitch_ang=asin(2*(q[0]*q[2]-q[3]*q[1]));
roll_ang=atan2(2*(q[0]*q[1]+q[3]*q[2]),1-2*(q[2]*q[2]+q[1]*q[1]) );
}
*/

/////////////////////////////////////////////////
//////////////////POSE SUBSCRIBER////////////////
/////////////////////////////////////////////////

void feedbackfn(const nav_msgs::Odometry::ConstPtr& odom_data)
{
    pos_feed= *odom_data;
    q[0] = pos_feed.pose.pose.orientation.w;
    q[1] = pos_feed.pose.pose.orientation.x;
    q[2] = pos_feed.pose.pose.orientation.y;
    q[3] = pos_feed.pose.pose.orientation.z;
    yaw_ang=atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
    pitch_ang=asin(2*(q[0]*q[2]-q[3]*q[1]));
    roll_ang=atan2(2*(q[0]*q[1]+q[3]*q[2]),1-2*(q[2]*q[2]+q[1]*q[1]) );
}

///////////////////////////////////////
//////////JOY STICK SUBSCRIBER////////////
////////////////////////////////////////

void joyfn(const sensor_msgs::Joy::ConstPtr& joy){
    ROS_INFO("JOY");
    joyvar = *joy;
/////////BUTTON 0 FOR TAKEOFF////////////////////////
if (joyvar.buttons[0] == 1 && fl_takeoff==false && fl_land==false)
	{fl_manual=false;
	fl_takeoff=true;
ROS_INFO("TAKE OFF");}
//////////////BUTTON 2 FOR LAND////////////////////////
if (joyvar.buttons[2] == 1 && fl_land==false && fl_takeoff==false)
	{fl_land=true;
    fl_manual=false;	
	ROS_INFO("LAND");}
////////////BUTTON 1 FOR MANUAL CONTROL//////////////
 if (joyvar.buttons[1] == 1 && fl_land==false && fl_takeoff==false)
    {fl_manual=true;  
    ROS_INFO("manual");}   
    //////////BUTTON 3 FOR AUTO///////////////////////////
if (joyvar.buttons[3] == 1 && fl_land==false && fl_takeoff==false)
    {fl_manual=false;  
    ROS_INFO("auto");}   

if (joyvar.buttons[5] == 1)
{
	fl_kill=true;
	ROS_INFO("KILL");
}

}

//////////////////////////////////
///// STATE SUBSCRIBER ////////////////
///////////////////////////////////

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
	float v_xi, v_yi, omg;

    //int rate = 20;

    ros::init(argc, argv, "multi_1");
    ros::NodeHandle nh;

    //ros::Rate r(rate);

    //dt=1/rate;
    ros::Subscriber imu_yaw = nh.subscribe("mavros/local_position/odom", 10, feedbackfn);
    ros::Subscriber joy_ip = nh.subscribe("joy", 10, joyfn);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher att_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/attitude", 10);

    geometry_msgs::PoseStamped pos_cmd;
    geometry_msgs::TwistStamped vel;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    
    pos_cmd.pose.position.x = 0;
    pos_cmd.pose.position.y = 0;
    pos_cmd.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pos_cmd);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
    	if (fl_takeoff==true && fl_land==false)
        {
            if(pos_feed.pose.pose.position.z<eps_z)
            {
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            // if( set_mode_client.call(offb_set_mode) &&
            //     offb_set_mode.response.success){
            //     ROS_INFO("Offboard enabled");
            // }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                // if( arming_client.call(arm_cmd) &&
                //     arm_cmd.response.success){
                //     ROS_INFO("Vehicle armed");
                // }
                last_request = ros::Time::now();
            }
        }
         }
    fl_takeoff=false;
    yaw_tgt=yaw_ang;			}


         ////////////////////////////////////////////
    ///////////////////LAND/////////////////////
    ////////////////////////////////////////////
    
 	if (fl_land==true)
 	   {fl_manual=false;
    
    ros::ServiceClient land_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = 0;
    // srv_land.request.latitude = 0;
    //srv_land.request.longitude = 0;
    srv_land.request.min_pitch = 0;
    //srv_land.request.yaw = 0;
    if(land_cl.call(srv_land)){
        //ROS_INFO("srv_land send ok %d", srv_land.response.success);
    }else{
        ROS_ERROR("Failed Land");
    }
    sleep(5);

    fl_land=false;
 	}

 	/////////////////////////////////////
 	////////KILL////////////////////////
 	/////////////////////////////////

 	if(fl_kill==true)
 	{
 		ROS_INFO("KILLING");
 		arm_cmd.request.value = false;
		// if( arming_client.call(arm_cmd) && arm_cmd.response.success)
		// {
  //       	ROS_INFO("Vehicle disarmed");
  //       }
        fl_kill=false;
 	}

        ////////////////////////////////////////////
    /////////////////MANUAL CONTROL///////////////////
    ////////////////////////////////////////////

    if (fl_manual==true&&fl_kill==false)
    {
   
        v_xi= joyvar.axes[3]*3.5;
        v_yi= joyvar.axes[2]*3.5;

        vel.twist.linear.y = v_yi;//*cos(-ang) - v_yi*sin(-ang);
        vel.twist.linear.x = v_xi;//*cos(-ang) + v_xi*sin(-ang);
        vel.twist.linear.z = joyvar.axes[1]*1.0;
        vel.twist.angular.z = joyvar.axes[0]*1.0;
        /*if(joyvar.buttons[6] != 0 ||  joyvar.buttons[7] != 0)
        {   omg = joyvar.buttons[6]*0.3;
            //else
            omg = -joyvar.buttons[7]*0.3;
            //if(joyvar.buttons[6] != 0)
            yaw_tgt+=omg*dt;  

            double t0 = cos(yaw_tgt * 0.5);
            double t1 = sin(yaw_tgt * 0.5);
            double t2 = cos(roll_ang* 0.5);
            double t3 = sin(roll_ang * 0.5);
            double t4 = cos(pitch_ang * 0.5);
            double t5 = sin(pitch_ang * 0.5);

            q_tgt[0] = t0 * t2 * t4 + t1 * t3 * t5;
            q_tgt[1] = t0 * t3 * t4 - t1 * t2 * t5;
            q_tgt[2] = t0 * t2 * t5 + t1 * t3 * t4;
            q_tgt[3] = t1 * t2 * t4 - t0 * t3 * t5;

            att_tgt.orientation.w=q_tgt[0];
            att_tgt.orientation.x=q_tgt[1];
            att_tgt.orientation.y=q_tgt[2];
            att_tgt.orientation.z=q_tgt[3];
            att_tgt.body_rate.z=omg;

            att_pub.publish(att_tgt);
        }
        else*/
            local_vel_pub.publish(vel);
    }

    ////////////////////////////////////////////
    ///////////////////AUTO FLIGHT/////////////////////
    ////////////////////////////////////////////  
    if(fl_manual==false && fl_takeoff==false && fl_kill==false)
    {   
        ROS_INFO("auto mode");
        //x_set = cent_x + rad*cos(ang); // CIRCLE
        //y_set = cent_y + rad*sin(ang);
        x_set = 2*sin(ang);
        y_set = 2*cos(ang);
	
        ROS_INFO("Going towards %f %f %f", x_set, y_set, z_set);
        ROS_INFO("current position %f   %f  %f angle %f",pos_feed.pose.pose.position.x, pos_feed.pose.pose.position.y, pos_feed.pose.pose.position.z, ang);
        if(fabs(pos_feed.pose.pose.position.z - z_set)<=eps_z)
        {
            ROS_INFO("ALTD REACHED");
            //if(fabs(pos_feed.pose.pose.position.x - x_set)<=eps && fabs(pos_feed.pose.pose.position.y - y_set)<=eps)
            ang = ang + step;
            pos_cmd.pose.position.x = x_set;
            pos_cmd.pose.position.y = y_set;
            pos_cmd.pose.position.z = z_set;
            pos_cmd.pose.orientation.x = 0.0;
            pos_cmd.pose.orientation.y = 0.0;
            pos_cmd.pose.orientation.z = 0.0;
            pos_cmd.pose.orientation.w = 0.0;
		
            /*vel.twist.linear.x = 0.3;
            vel.twist.linear.y = 0.0;
            vel.twist.linear.z = 0.0;
            vel.twist.angular.z = 0.1;*/
        }
        else
        {
            ROS_INFO("REACHING ALTD");
            pos_cmd.pose.position.x = pos_feed.pose.pose.position.x;
            pos_cmd.pose.position.y = pos_feed.pose.pose.position.y;
            pos_cmd.pose.position.z = z_set;
            pos_cmd.pose.orientation.x = 0.0;
            pos_cmd.pose.orientation.y = 0.0;
            pos_cmd.pose.orientation.z = 0.0;
            pos_cmd.pose.orientation.w = 0.0;
        }

        local_pos_pub.publish(pos_cmd);
     }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
