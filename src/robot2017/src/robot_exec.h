//teleop_exec.h
//handles teleop calculations
//calls appropriate motor functions

#include "sensors.h"
#include "robot_msgs/Teleop.h"
#include "robot_msgs/Autonomy.h"
#include "robot_msgs/MotorFeedback.h"
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <vesc_bbb_uart/bldc.h>

#ifndef ROBOT_EXEC_H
#define ROBOT_EXEC_H

class MotorsReceive;

class RobotExec
{
    friend MotorsReceive;
    private:
        bool dead;  // Killed by an estop or by deadman
        bool onPC;
        bool debug;
        bool autonomyActive;
        bool prevState;

        float leftRatio;
        float rightRatio;
        BLDC LeftDrive;
        BLDC RightDrive;
        BLDC Lift;
        BLDC Storage;
        BLDC Bucket;
        Sensors sensors;

        std_msgs::Bool enable;

    public:
        RobotExec(bool onPC, bool debug, bool autoActive); //constructor

        void teleopReceived(const robot_msgs::Teleop& cmd);
        void modeTransition(const bool buttonState);

        void autonomyReceived(const robot_msgs::Autonomy& cmd);

        void teleopExec(const robot_msgs::Teleop& cmd);
        void autonomyExec(const robot_msgs::Autonomy& cmd);
        void killMotors();

        bool isOnPC();

        bool isDebugMode();
        void setDebugMode(bool active);

        bool isAutonomyActive();
        void setAutonomyActive(bool active);

        bool isDead();

        void motorHeartbeat();

        robot_msgs::MotorFeedback getMotorFeedback();
        std_msgs::Bool getEnMsg();
};

#endif
