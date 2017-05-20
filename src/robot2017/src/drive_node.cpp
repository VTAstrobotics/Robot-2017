// drive_node.cpp
// handles robot driving, including processing teleop commands and managing autonomy

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>
#include <string>
#include "robot_msgs/Teleop.h"
#include "robot_msgs/Autonomy.h"
#include "robot_exec.h"
#include "motor_receive.h"

const int refreshRate = 1;

//Command line arguments - order doesn't matter (other than debug type must follow -debug)
//-pc: Running test on PC instead of beaglebone
//-debug: Hardcodes some values in order to test specific features
//-aut: Tests autonomy - hardcodes autonomyActive to 1 (true), does not subscribe to teleop messages
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

    ros::Subscriber sub_tele = nh.subscribe("/robot/teleop", 1000, &RobotExec::teleopReceived, &exec);

    ros::Subscriber sub_aut = nh.subscribe("/robot/autonomy", 1000, &RobotExec::autonomyReceived, &exec);

    ros::Publisher pub_fb = nh.advertise<robot_msgs::MotorFeedback>("/robot/motor/feedback", 100);

    ros::Publisher pub_en = nh.advertise<std_msgs::Bool>("/robot/autonomy/enable", 100);

    robot_msgs::MotorFeedback motorFb;

    bool autonomyEnabled = false;

    ROS_INFO("Astrobotics 2017 ready");

    int hertz = 100;
    ros::Rate r(hertz);

    while(ros::ok())
    {
        // TODO add status led code for setting HIBER when ping disconnects
        ros::spinOnce();
        exec.motorHeartbeat();
        exec.checkKillButton();

        if (autonomyEnabled != exec.getEnMsg().data) //if enable msg has been updated
        {
            autonomyEnabled = exec.getEnMsg().data;
            pub_en.publish(exec.getEnMsg());
        }

        // Publishing motor data every time
        motorFb = exec.getMotorFeedback();
        ROS_DEBUG_STREAM(motorFb.drumRPM << " " << motorFb.liftPos << " " << motorFb.leftTreadRPM
        << " " << motorFb.rightTreadRPM);
        pub_fb.publish(motorFb);

        motors_update.update();
        r.sleep();
    }

    return 0;
}
