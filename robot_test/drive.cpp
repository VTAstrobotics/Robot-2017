#include <ros/ros.h>
#include <std_msgs/Int16.h> // for std_msgs/Int16

// dummy node until I can figure out pwm
// with the erle and flobotics drivers

int count = 0;

void increment()
{
	count = (count + 1) % 16;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "drive_robot");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::Int16>(
		"robot-test/drive", 1000);

	ros::Rate rate(10);
	while(ros::ok())
	{
		increment();
		
		std_msgs::Int16 msg;
		msg.data = count;

		pub.publish(msg);

		rate.sleep();
	}
}
