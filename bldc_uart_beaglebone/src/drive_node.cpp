// drive_node.cpp

#include <ros/ros.h>
#include <std_msgs/Int64.h>

#include "comm_uart.h"
#include "bldc_interface.h"

int power;

int main(int argc, char **argv)
{
	// initialize the ROS system.
	ros::init(argc, argv, "drive_node");

	// establish this program as an ROS node.
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<std_msgs::Int64>("/drive/status", 1000);

	// indicates the node has started
	ROS_INFO_STREAM("DRIVING");

	comm_uart_init();

	ros::Rate rate(0.5);
	while(ros::ok())
	{
		if (power == 100)
		{
			power = 0;
			ROS_INFO_STREAM("MOTORS OFF");
		}
		else
		{
			power = 100;
			ROS_INFO_STREAM("MOTORS ON");
		}
		std_msgs::Int64 message;
		message.data = power;
		pub.publish(message);
		rate.sleep();
	}

	return 0;
}
