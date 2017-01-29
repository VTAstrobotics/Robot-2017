// drive_node.cpp
// alternates turning motors on and off for 3 seconds each.

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <string>

#include <bldc_uart_beaglebone/comm_uart.h>
#include <bldc_uart_beaglebone/bldc_interface.h>


// control variables // ! -- SET THESE VALUES BEFORE RUNNING -- ! //
const int max_erpm = 0; 
const float duty = 0;
const float amps = 0;
const float brake = 0;


int main(int argc, char **argv)
{
	if (max_erpm == 0)
	{
		ROS_WARN_STREAM("Max RPM is 0");
	}


	// initialize the ROS system.
	ros::init(argc, argv, "drive_node");

	// establish this program as an ROS node.
	ros::NodeHandle nh;

	// initialize the UART interface
	comm_uart_init();

	// set the current in amps
	bldc_interface_set_current(amps);

	// set the brake current in amps
	bldc_interface_set_current_brake(brake);

	// set the duty cycle
	bldc_interface_set_duty_cycle(duty);


	// create a topic which will contain motor speed
	ros::Publisher pub = nh.advertise<std_msgs::Int64>("/drive/rpm", 1000);

	std::stringstream message;
	message << "DRIVING. Max RPM: " << max_erpm << ", duty cycle: " << duty;
	message << ", current: " << amps << " amps, brake current: " << brake << " amps.";
	ROS_INFO_STREAM(message.str());


	int current_RPM = 0;

	ros::Rate rate(0.333);
	while(ros::ok())
	{
		if (current_RPM == max_erpm)
		{
			current_RPM = 0;
			ROS_INFO_STREAM("Motors OFF");
		}
		else
		{
			current_RPM = max_erpm;
			ROS_INFO_STREAM("Motors ON");
		}

		// sets motor speed
		bldc_interface_set_rpm(current_RPM);

		// publish a message containing motor speed
		std_msgs::Int64 message;
		message.data = current_RPM;
		pub.publish(message);
		rate.sleep();
	}

	return 0;
}
