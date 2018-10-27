#include "ros/ros.h"
#include <gazebo_msgs/SetModelConfiguration.h>
#include <std_msgs/Float32.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_spinner");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelConfiguration>("gazebo/set_model_configuration");
  ros::Publisher angle_pub = n.advertise<std_msgs::Float32>("hedwig/lidar_angle", 1);
  float angle = 0;
  float increment = 0.157*0.5;
  ros::Rate r(10); // 10 hz
  while (ros::ok())
  {
    gazebo_msgs::SetModelConfiguration srv;
    srv.request.model_name = "typhoon_h480";
    srv.request.urdf_param_name = "robot_description";
    srv.request.joint_names.push_back("lidar_joint");
    angle += increment;
    if (angle > 3.1428 && increment > 0){
      angle = -3.1428 + (angle - 3.1428);
    }

    else if (angle > 0 && increment < 0){
      angle = 0;
    }
    srv.request.joint_positions.push_back(angle);
    std_msgs::Float32 msg;
    msg.data = angle;
    client.call(srv);
    angle_pub.publish(msg);
    r.sleep();
  }

  return 0;
}