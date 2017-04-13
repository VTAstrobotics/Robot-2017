//teleop_exec.h
//handles teleop calculations
//calls appropriate motor functions

#include "robot_msgs/Teleop.h"
#include "robot_msgs/Autonomy.h"
#include "robot_msgs/MotorFeedback.h"
#include <ros/ros.h>
#include <vesc_bbb_uart/bldc.h>

#ifndef ROBOT_EXEC_H
#define ROBOT_EXEC_H

class RobotExec
{
    private:
        bool dead;
        bool onPC;
        bool debug;
        bool autonomyActive;

        float leftRatio;
        float rightRatio;
        BLDC LeftDrive;
        BLDC RightDrive;
        BLDC Lift;
        BLDC Storage;
        BLDC Bucket;

    public:
        RobotExec(bool onPC, bool debug, bool autoActive); //constructor

        void teleopReceived(const robot_msgs::Teleop& cmd);
        void autonomyReceived(const robot_msgs::Autonomy& cmd);

        void teleopExec(const robot_msgs::Teleop& cmd);
        void autonomyExec(const robot_msgs::Autonomy& cmd);
        void killMotors();

        bool isAutonomyActive();
        void setAutonomyActive(bool active);

        bool isDebugMode();
        void setDebugMode(bool active);

        robot_msgs::MotorFeedback publishMotors();
};

#endif
