//watchdog_node.cpp
//receives watchdog msg type from beaglebone serial port (rosserial)
//publishes to /robot/watchdog rostopic

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "watchdog_node");


    return 0;
}