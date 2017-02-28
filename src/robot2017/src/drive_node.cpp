// drive_node.cpp
// handles robot driving, including processing teleop commands and managing autonomy

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <string>
#include "robot_msgs/Teleop.h"
#include "motors.h"
#include "robot_exec.h"

const int refreshRate = 1;

bool onBeagleBone = true;
bool autState = false;
ros::Publisher pub;
RobotExec exec;

//Command line arguments - order doesn't matter (other than debug type must follow -debug)
//-pc: Running test on PC instead of beaglebone
//-debug: Hardcodes some values in order to test specific features
//-aut: Tests autonomy - hardcodes autonomyActive to 1 (true), does not subscribe to teleop messages
//TODO: any other specific states we want to test?
int main(int argc, char **argv)
{
    for (int i = 0; i < argc; ++i)
    {
        if (strcmp(argv[i], "-pc") == 0)
        {
            onBeagleBone = false;
            ROS_WARN_STREAM("Node is not being run on a BeagleBone");
        }
		else if (strcmp(argv[i], "-debug") == 0)
		{
			if(strcmp(argv[i+1], "-aut") == 0)
			{
				exec.autonomyActive = true;
			}
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

	if (exec.autonomyActive == false) //autonomy debugging, don't want teleop commands interfering if true
		ros::Subscriber sub_tele = nh.subscribe("/robot/teleop", 1000, &RobotExec::teleopReceived, &exec);
    ros::Subscriber sub_aut = nh.subscribe("/robot/autonomy", 1000, &RobotExec::autonomyReceived, &exec);

    int current_RPM = 0;

    int tick = 0;

    ROS_INFO("Astrobotics 2017 ready");
    ros::spin();
    return 0;

}
