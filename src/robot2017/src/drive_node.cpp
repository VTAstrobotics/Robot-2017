// drive_node.cpp
// handles robot driving, including processing teleop commands and managing autonomy

#include <ros/ros.h>
#include <std_msgs/Int64.h>
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
//TODO: any other specific states we want to test?
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

    ros::Publisher pub_fb = nh.advertise<robot_msgs::MotorFeedback>("/robot/autonomy/feedback", 100);

    robot_msgs::MotorFeedback motorFb;

    ROS_INFO("Astrobotics 2017 ready");

    int hertz = 100;
    ros::Rate r(hertz);

    while(ros::ok())
    {
        ros::spinOnce();

        if (exec.isAutonomyActive())
        {
            motorFb = exec.publishMotors();
            std::stringstream msg;
            msg << motorFb.drumRPM << " " << motorFb.liftPos << " " << motorFb.leftTreadRPM
            << " " << motorFb.rightTreadRPM;
            ROS_DEBUG_STREAM_COND(exec.isDebugMode(), msg.str());
            pub_fb.publish(motorFb);
        }

        motors_update.update();
        r.sleep();
    }

    return 0;
}
