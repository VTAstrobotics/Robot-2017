// drive_node.cpp
// handles robot driving, including processing teleop commands and managing autonomy

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <string>
#include "robot2017/Teleop.h"
#include "motors.h"

const int refreshRate = 1;

bool onBeagleBone = true;
ros::Publisher pub;

Motor driveLeft(0);
Motor driveRight(1);

void teleopReceived(const robot2017::Teleop& msg)
{
    // byte range should be [-100, 100]
    ROS_INFO("Driving left: %d, right: %d", msg.y_l_thumb, msg.y_r_thumb);
	driveLeft.set(msg.y_l_thumb * 1.0f / 100.0f);
    driveRight.set(msg.y_r_thumb * 1.0f / 100.0f);
}

int main(int argc, char **argv)
{
	for (int i = 0; i < argc; ++i)
	{
		if (strcmp(argv[i], "-pc") == 0)
		{
			onBeagleBone = false;
			ROS_DEBUG_STREAM("Node is not being run on a BeagleBone");
		}
	}


	// initialize the ROS system.
	ros::init(argc, argv, "drive_node");

	// establish this program as an ROS node.
	ros::NodeHandle nh;

	// initialize the motors interface
	if (onBeagleBone)
	{
		Motor::init();
	}

	// create a topic which will contain motor speed
	pub = nh.advertise<std_msgs::Int64>("/robot/rpm", 1000);

	ros::Subscriber sub = nh.subscribe("/robot/teleop", 1000, &teleopReceived);

// 	std::stringstream message;
// 	message << "DRIVING. Max RPM: " << max_erpm << ", duty cycle: " << duty;
// 	message << ", current: " << amps << " amps, brake current: " << brake << " amps.";
// 	ROS_INFO_STREAM(message.str());

	int current_RPM = 0;

	int tick = 0;

    ROS_INFO("Astrobotics 2017 ready");
    ros::spin();
/*	ros::Rate rate(refreshRate);
	while(ros::ok())
	{
		if (tick % (refreshRate * 6) == 0)
		{
			current_RPM = 0;
			ROS_INFO_STREAM("Motors OFF");
		}
		else if (tick % (refreshRate * 3) == 0)
		{
			current_RPM = max_erpm;
			ROS_INFO_STREAM("Motors ON");
		}

		// sets motor speed
		// bldc_interface_set_rpm(current_RPM);

		// publish a message containing motor speed
		std_msgs::Int64 message;
		message.data = current_RPM;
		pub.publish(message);
		rate.sleep();
		ros::spinOnce();
		++tick;
	}*/

	return 0;
}
