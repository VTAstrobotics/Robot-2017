// drive_node.cpp
// handles robot driving, including processing teleop commands and managing autonomy

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
#include <string>
#include "robot_msgs/Teleop.h"
#include "robot_msgs/Autonomy.h"
#include "robot_msgs/Status.h"
#include "robot_exec.h"
#include "motor_receive.h"

const uint8_t statusRate = 1; // hertz
const uint8_t maxPing = 2; // max secs between pings before shutdown
bool hibernating = true;

ros::Publisher pub_status; // declared globally for publishStatus()
ros::Time lastDriverPing;
robot_msgs::Status status;

//Command line arguments - order doesn't matter (other than debug type must follow -debug)
//-pc: Running test on PC instead of beaglebone
//-debug: Hardcodes some values in order to test specific features
//-aut: Tests autonomy - hardcodes autonomyActive to 1 (true), does not subscribe to teleop messages
//TODO: any other specific states we want to test?

void publishStatus(RobotExec& exec)
{
    pub_status.publish(exec.getStatus());
}

void recievedPing(const std_msgs::Bool& ping)
{
    ROS_DEBUG_STREAM("Driver ping recieved.");
    lastDriverPing = ros::Time::now();
}

int main(int argc, char **argv)
{
    // initialize the ROS system.
    ros::init(argc, argv, "drive_node");

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
            if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
                ros::console::notifyLoggerLevelsChanged();
            }
            ROS_WARN_STREAM("Debug mode enabled");
        } else if(strcmp(argv[i], "-aut") == 0)
        {
            autoEnable = true;
            ROS_WARN_STREAM("Starting in autonomy mode");
        }
    }

    RobotExec exec(onPC, debug, autoEnable);
    MotorsReceive motors_update(exec);

    // establish this program as an ROS node.
    ros::NodeHandle nh;
    lastDriverPing = ros::Time::now() - ros::Duration(100);

    pub_status = nh.advertise<robot_msgs::Status>("/robot/status", 100);

    ros::Subscriber driver_ping = nh.subscribe("/driver/ping", 1000, &recievedPing);

    ros::Subscriber sub_tele = nh.subscribe("/robot/teleop", 1000, &RobotExec::teleopReceived, &exec);

    ros::Subscriber sub_aut = nh.subscribe("/robot/autonomy", 1000, &RobotExec::autonomyReceived, &exec);

    ros::Publisher pub_fb = nh.advertise<robot_msgs::MotorFeedback>("/robot/motor/feedback", 100);

    ros::Publisher pub_en = nh.advertise<std_msgs::Bool>("/robot/autonomy/enable", 100);

    robot_msgs::MotorFeedback motorFb;

    exec.setPingDisabled(true);
    bool autonomyEnabled = false;

    ROS_INFO("Astrobotics 2017 ready");
    ROS_INFO("Awaiting connection to driver station...");

    int hertz = 100;
    ros::Rate r(hertz);

    while(ros::ok())
    {
        // TODO add status led code for setting HIBER when ping disconnects
        ros::spinOnce();
        exec.motorHeartbeat();
        exec.enforceLimits();

        // Safety ping auto-disable
        double timeSince = (ros::Time::now() - lastDriverPing).toSec();
        if (timeSince > maxPing && !hibernating)
        {
            hibernating = true;
            ROS_ERROR_STREAM("Disconnected from driver station");
            exec.killMotors();
            exec.setPingDisabled(true);
        }
        else if (timeSince <= maxPing && hibernating)
        {
            ROS_INFO_STREAM("Established connection with driver station");
            hibernating = false;
            exec.setPingDisabled(false);
        }

        publishStatus(exec);

        // Update autonomy enable status
        if (autonomyEnabled != exec.getEnMsg().data)
        {
            autonomyEnabled = exec.getEnMsg().data;
            pub_en.publish(exec.getEnMsg());
        }

        // Publishing motor data every time
        motorFb = exec.getMotorFeedback();
        pub_fb.publish(motorFb);

        motors_update.update();
        r.sleep();
    }
    return 0;
}
