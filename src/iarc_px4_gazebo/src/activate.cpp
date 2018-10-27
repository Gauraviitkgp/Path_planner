#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <stdlib.h>

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "activate_node");
	ros::NodeHandle n;
	ros:: Publisher pub_4; 
	pub_4 = n.advertise<std_msgs::Bool>("/activate", 10);
	while(n.ok())
	{
		std_msgs::Bool d;
		d.data = true;
		pub_4.publish(d);
		ros::spinOnce();
	}
	return 0;
}