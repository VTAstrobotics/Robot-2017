// drive_node.cpp
// handles robot driving, including processing teleop commands and managing autonomy

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <string>
#include "robot_msgs/Teleop.h"
#include "motors.h"
#include "teleop_exec.h"

const int refreshRate = 1;

bool onBeagleBone = true;
bool autState = false;
ros::Publisher pub;
TeleopExec teleop;

int main(int argc, char **argv)
{
    for (int i = 0; i < argc; ++i)
    {
        if (strcmp(argv[i], "-pc") == 0)
        {
            onBeagleBone = false;
            ROS_WARN_STREAM("Node is not being run on a BeagleBone");
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

    ros::Subscriber sub_aut = nh.subscribe("/robot/teleop", 1000, &autonomyReceived);
    ros::Subscriber sub_tele = nh.subscribe("/robot/teleop", 1000, &teleopReceived);

    int current_RPM = 0;

    int tick = 0;

    ROS_INFO("Astrobotics 2017 ready");
    ros::spin();

    return 0;

}
