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

void teleopReceived(const robot2017::Teleop& cmd)
{
    std::stringstream message;

    message << "Command recieved: ";
    message << " " << +cmd.a << " " << +cmd.b << " " << +cmd.x << " " << +cmd.y;
    message << " " << +cmd.lb << " " << +cmd.rb << " " << +cmd.back;
    message << " " << +cmd.start << " " << +cmd.l_thumb << " " << +cmd.r_thumb;
    message << " (" << +cmd.x_l_thumb << " " << +cmd.y_l_thumb << ")";
    message << " (" << +cmd.x_r_thumb << " " << +cmd.y_r_thumb << ")";
    message << " (" << +cmd.l_trig << " " << +cmd.y_trig << ")";

    ROS_DEBUG_STREAM(message.str());

    // byte range should be [-100, 100]
    driveLeft.set(cmd.y_l_thumb * 1.0f / 100.0f);
    driveRight.set(cmd.y_r_thumb * 1.0f / 100.0f);
}

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

    ros::Subscriber sub = nh.subscribe("/robot/teleop", 1000, &teleopReceived);

    int current_RPM = 0;

    int tick = 0;

    ROS_INFO("Astrobotics 2017 ready");
    ros::spin();

    return 0;
}
