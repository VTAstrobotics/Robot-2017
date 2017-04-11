//watchdog_node.cpp
//receives watchdog msg type from beaglebone serial port (rosserial)
//publishes to /robot/watchdog rostopic

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <string>
#include "robot_msgs/Watchdog.h"

robot_msgs::Watchdog wd;

void watchdogReceived(const robot_msgs::Watchdog& msg)
{
    wd = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "watchdog_node");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/serial/watchdog", 100, &watchdogReceived);
    ros::Publisher pub = nh.advertise<robot_msgs::Watchdog>("/robot/watchdog", 100);

    while (ros::ok())
    {
        ros::spinOnce();

        std::stringstream message;
        message << "Watchdog data: " << +wd.batVoltage << " " << +wd.stateOfCharge << " " << +wd.cell1Temp;
        message << " " << +wd.cell2Temp << " " << +wd.dcConverterTemp << " " << +wd.current << " " << +wd.cpuTemp;
        message << " " << +wd.ssrTemp << " " << +wd.boxTemp << " " << +wd.vcc << " " << +wd.timeFromLastStop << " " << +wd.totalKWH;
        ROS_INFO_STREAM(message.str());

        pub.publish(wd);
    }

    return 0;
}