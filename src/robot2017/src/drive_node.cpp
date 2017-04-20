// drive_node.cpp
// handles robot driving, including processing teleop commands and managing autonomy

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include <string>
#include "robot_msgs/Teleop.h"
#include "robot_msgs/Autonomy.h"
#include "robot_msgs/Status.h"
#include "robot_exec.h"

const uint8_t statusRate = 1; // hertz
bool onBeagleBone = true;
bool autState = false;
bool hibernating = true;

ros::Subscriber sub_tele, sub_aut, driver_ping;
ros::Publisher pub_fb, pub_status;
ros::Time lastDriverPing;
robot_msgs::Status status;

//Command line arguments - order doesn't matter (other than debug type must follow -debug)
//-pc: Running test on PC instead of beaglebone
//-debug: Hardcodes some values in order to test specific features
//-aut: Tests autonomy - hardcodes autonomyActive to 1 (true), does not subscribe to teleop messages
//TODO: any other specific states we want to test?

void publishStatus()
{
    ROS_DEBUG_STREAM("Publishing status message.");
    pub_status.publish(status);
}

void recievedPing(const std_msgs::Int8& ping)
{
    ROS_DEBUG_STREAM("Driver ping recieved.");
    lastDriverPing = ros::Time::now();
}

int main(int argc, char **argv)
{
    // initialize the ROS system.
    ros::init(argc, argv, "drive_node");
    ros::NodeHandle nh;
    lastDriverPing = ros::Time::now() - ros::Duration(100);

    // argument processing
    bool onPC = false;
    bool debug = false;
    bool autoEnable = false;
    for (int i = 0; i < argc; ++i)
    {
        if (strcmp(argv[i], "-pc") == 0)
        {
            onPC = true;
            ROS_WARN_STREAM("Node is not being run on a BeagleBone");
        }
        else if (strcmp(argv[i], "-debug") == 0)
        {
            debug = true;
            ROS_WARN_STREAM("Debug mode enabled");
        } else if(strcmp(argv[i], "-aut") == 0)
        {
            autoEnable = true;
            ROS_WARN_STREAM("Starting in autonomy mode");
        }
    }
    
    status.robotCodeActive = true;
    status.autonomyActive = autoEnable;
    status.deadmanPressed = false;

    RobotExec exec(onPC, debug, autoEnable);

    pub_fb = nh.advertise<robot_msgs::MotorFeedback>("/robot/autonomy/feedback", 100);

    pub_status = nh.advertise<robot_msgs::Status>("/robot/status", 100);
    driver_ping = nh.subscribe("/driver/ping", 1000, &recievedPing);
    ros::Publisher fake_ping = nh.advertise<std_msgs::Int8>("/driver/ping", 100);
    
    robot_msgs::MotorFeedback motorFb;

    ROS_INFO("Astrobotics 2017 ready");
    ROS_INFO_STREAM("Awaiting connection to driver station...");
    
    int hertz;
    if (exec.isDebugMode()) // FIXME we probably don't want this in the future
    {
        hertz = 10; //slow rate when in debug mode
    }
    else
    {
        hertz = 100; //100 Hz/10 ms (is this the freq. we want?)
    }

    ros::Rate r(hertz);

    while(ros::ok())
    {
        ros::spinOnce();
        double timeSince = (ros::Time::now() - lastDriverPing).toSec();
        if (timeSince > 2 && !hibernating)
        {
            hibernating = true;
            ROS_ERROR_STREAM("Disconnected from driver station");
            exec.killMotors();
        }
        else if (timeSince <= 2 && hibernating)
        {
            ROS_INFO_STREAM("Established connection with driver station");
            hibernating = false;
        }
        if (!hibernating && exec.isAutonomyActive())
        {
            ROS_DEBUG_STREAM("Executing motor commands");
            motorFb = exec.publishMotors();
            std::stringstream msg;
            msg << motorFb.drumRPM << " " << motorFb.liftPos << " " << motorFb.leftTreadRPM
            << " " << motorFb.rightTreadRPM;
            ROS_DEBUG_STREAM_COND(exec.isDebugMode(), msg.str());
            pub_fb.publish(motorFb);
        }
        publishStatus();
        r.sleep();
    }
    return 0;
}
