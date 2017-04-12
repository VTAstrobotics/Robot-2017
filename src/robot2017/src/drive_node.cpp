// drive_node.cpp
// handles robot driving, including processing teleop commands and managing autonomy

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <string>
#include "robot_msgs/Teleop.h"
#include "robot_msgs/Autonomy.h"
#include "robot_msgs/Ping.h"
#include "robot_exec.h"

const int refreshRate = 1;
const double pingRate = 4; // hertz
bool onBeagleBone = true;
bool autState = false;
// 0 - connected, 1 - disconnected, 2 - pending response
int connection = 0;

RobotExec exec;

ros::Subscriber sub_tele, sub_aut, ping_in;
ros::Publisher pub_fb, ping_out;

//Command line arguments - order doesn't matter (other than debug type must follow -debug)
//-pc: Running test on PC instead of beaglebone
//-debug: Hardcodes some values in order to test specific features
//-aut: Tests autonomy - hardcodes autonomyActive to 1 (true), does not subscribe to teleop messages
//TODO: any other specific states we want to test?

void publishPing(const ros::TimerEvent& event)
{
    ROS_DEBUG_STREAM("Pinging driver station.");
    robot_msgs::Ping msg;
    msg.data = 0;
    ping_out.publish(msg);
    connection = 2;
}

void recievedPing(const robot_msgs::Ping& ping)
{
    if (ping.data == 1)
    {
        ROS_DEBUG_STREAM("Confirmed connection with driver station.");
        connection = 0;
    }
    else connection = 1;
}

int main(int argc, char **argv)
{
    // initialize the ROS system.
    ros::init(argc, argv, "drive_node");
    ros::NodeHandle nh;

    for (int i = 0; i < argc; ++i)
    {
        if (strcmp(argv[i], "-pc") == 0)
        {
            onBeagleBone = false;
            ROS_WARN_STREAM("Node is not being run on a BeagleBone");
        }
        else if (strcmp(argv[i], "-debug") == 0)
        {
            exec.setDebugMode(true);
        }
    }

    //temporarily hardcoding this
    exec.setDebugMode(true);
    exec.setAutonomyActive(true);
    
    sub_tele = nh.subscribe("/robot/teleop", 1000, &RobotExec::teleopReceived, &exec);
    sub_aut = nh.subscribe("/robot/autonomy", 1000, &RobotExec::autonomyReceived, &exec);

    pub_fb = nh.advertise<robot_msgs::MotorFeedback>("/robot/autonomy/feedback", 100);

    ping_out = nh.advertise<robot_msgs::Ping>("/robot/ping", 100);
    ping_in = nh.subscribe("/robot/ping", 1000, &recievedPing);
    
    ros::Timer timer = nh.createTimer(ros::Duration(1/pingRate), publishPing);
    
    robot_msgs::MotorFeedback motorFb;

    ROS_INFO("Astrobotics 2017 ready");
    
    int hertz;
    if (exec.isDebugMode())
    {
        hertz = 1; //slow rate when in debug mode
    }
    else
    {
        hertz = 100; //100 Hz/10 ms (is this the freq. we want?)
    }

    ros::Rate r(hertz);

    while(ros::ok())
    {
        ros::spinOnce();
        
        ROS_WARN_STREAM(connection);
        if (connection == 1)
        {
            ROS_WARN_STREAM("DISCONNECTED FROM CONTROLLER.");
            exec.killMotors();
            ros::Duration(5).sleep();
        }
        else if (exec.isAutonomyActive())
        {
            motorFb = exec.publishMotors();
            std::stringstream msg;
            msg << motorFb.drumRPM << " " << motorFb.liftPos << " " << motorFb.leftTreadRPM
            << " " << motorFb.rightTreadRPM;
            // ROS_INFO_STREAM(msg.str());
            pub_fb.publish(motorFb);
        }
        
        r.sleep();
    }

    return 0;

}
