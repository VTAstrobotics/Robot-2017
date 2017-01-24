#include <ros/ros.h>
#include <std_msgs/Int16.h> // for std_msgs/String

int count = 0;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "drive_robot");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::Int16>(
		"drive/status", 1000);

	ros::Rate rate(10);
	while(ros::ok())
	{
		++count;
		
		std_msgs::Int16 msg;
		msg.data = count;

		pub.publish(msg);

		rate.sleep();
	}
}
