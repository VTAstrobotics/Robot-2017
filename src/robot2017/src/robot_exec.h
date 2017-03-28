//teleop_exec.h
//handles teleop calculations
//calls appropriate motor functions

#include "robot_msgs/Teleop.h"
#include "robot_msgs/Autonomy.h"
#include "robot_msgs/MotorFeedback.h"
#include "motors.h"
#include <ros/ros.h>

#ifndef ROBOT_EXEC_H
#define ROBOT_EXEC_H

class RobotExec
{
    private:
        bool dead;
        bool autonomyActive;
        float leftRatio;
        float rightRatio;
        Motor driveLeft;
        Motor driveRight;
        ros::NodeHandle nh;
        ros::Publisher pub_fb = nh.advertise<robot_msgs::MotorFeedback>("/robot/autonomy/feedback", 100);

    public:
        RobotExec(); //constructor

        void teleopReceived(const robot_msgs::Teleop& cmd);
        void autonomyReceived(const robot_msgs::Autonomy& cmd);

        void teleopExec(const robot_msgs::Teleop& cmd);
        void autonomyExec(const robot_msgs::Autonomy& cmd);
        void killMotors();

        bool isAutonomyActive();
        void setAutonomyActive(bool active);

        void publishMotors();
};

#endif
