#include "robot_exec.h"
#include "robot_msgs/Teleop.h"
#include "robot_msgs/Autonomy.h"
#include "robot_msgs/MotorFeedback.h"
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <string>

const char* motorPath = "/dev/tty01";  // Connected via UART
// const char* motorPath = "/dev/ttyUSB0" // Connected via USB

RobotExec::RobotExec(bool onPC, bool debug, bool autoActive)
    : dead(true), onPC(onPC), debug(debug), autonomyActive(autoActive),
      leftRatio(0.0f), rightRatio(0.0f),
      LeftDrive(LEFTDRIVE,   Alien_4260),
      RightDrive(RIGHTDRIVE, Alien_4260),
      Lift(LIFT,             Alien_4260),
      Storage(STORAGE,       Alien_4260),
      Bucket(BUCKET,         Alien_4260)
{
    if(!onPC) {
        BLDC::init((char*) motorPath);
    }
}

void RobotExec::teleopReceived(const robot_msgs::Teleop& cmd)
{
    //if debugging, ignore teleop cmds
    if (this->debug) {
        std::stringstream message;

        message << "Teleop Command recieved: ";
        message << " " << +cmd.a << " " << +cmd.b << " " << +cmd.x << " " << +cmd.y;
        message << " " << +cmd.lb << " " << +cmd.rb << " " << +cmd.back;
        message << " " << +cmd.start << " " << +cmd.l_thumb << " " << +cmd.r_thumb;
        message << " (" << +cmd.x_l_thumb << " " << +cmd.y_l_thumb << ")";
        message << " (" << +cmd.x_r_thumb << " " << +cmd.y_r_thumb << ")";
        message << " (" << +cmd.l_trig << " " << +cmd.r_trig << ")";

        ROS_DEBUG_STREAM(message.str());
    }

    //if autonomy mode is toggled (y button is pressed)
    if (cmd.y == 1)
        this->autonomyActive = !(this->autonomyActive); //toggle autonomy state
    if(this->autonomyActive)
        return;
    else
    {
        //function for teleop
        teleopExec(cmd);
    }

    // byte range should be [-100, 100]
    //driveLeft.set(cmd.y_l_thumb * 1.0f / 100.0f);
    //driveRight.set(cmd.y_r_thumb * 1.0f / 100.0f);
}

void RobotExec::autonomyReceived(const robot_msgs::Autonomy& cmd)
{
    std::stringstream message;

    message << "Autonomy Command recieved: " << cmd.leftRatio << " "
    << cmd.rightRatio << " " << cmd.digCmd << " " << cmd.dumpCmd;;

    ROS_DEBUG_STREAM_COND(this->isDebugMode(),message.str());

    if(this->autonomyActive)
    {
        autonomyExec(cmd);
    }
    publishMotors();
}

void RobotExec::teleopExec(const robot_msgs::Teleop& cmd)
{
    dead = !cmd.lb;
    //write LED??? (did this last year)
    ROS_DEBUG_STREAM("Robot dead: " << dead);
    if(dead)
    {
        killMotors();
        return;
    }

    // Note: y axis is given as negative = up, positive = down
    //       x axis also needs to be reversed when going backwards
    if(cmd.y_l_thumb == 0)
    {
        // special case for spins
        rightRatio = -cmd.x_r_thumb;
        leftRatio  =  cmd.x_r_thumb;
    }
    else if(cmd.y_l_thumb < 0)
    {
        if(cmd.x_r_thumb > 0)
        {
            rightRatio = -cmd.y_l_thumb - cmd.x_r_thumb;
            leftRatio = fmax(-cmd.y_l_thumb, cmd.x_r_thumb);
        }
        else
        {
            rightRatio = fmax(-cmd.y_l_thumb, -cmd.x_r_thumb);
            leftRatio = -cmd.y_l_thumb + cmd.x_r_thumb;
        }
    }
    else
    {
        if(cmd.x_r_thumb < 0)
        {
            rightRatio = -fmax(-cmd.y_l_thumb, -cmd.x_r_thumb);
            leftRatio = -cmd.y_l_thumb - cmd.x_r_thumb;
        }
        else
        {
            rightRatio = -cmd.y_l_thumb + cmd.x_r_thumb;
            leftRatio = -fmax(cmd.y_l_thumb, cmd.x_r_thumb);
        }
    }

    // TODO decide if we need set_Speed (rpm) or set_Current (amps) or set_Duty (percent of input voltage)
    if(!this->onPC) {
        LeftDrive.set_Duty(leftRatio);
        RightDrive.set_Duty(rightRatio);
    }
    std::stringstream msg;
    msg << "Left Ratio " << leftRatio << ", Right Ratio " << rightRatio;
    ROS_DEBUG_STREAM(msg.str());

    if(fabs(leftRatio) > 0.1 || fabs(rightRatio))
        return;

    if(cmd.l_trig > 0.0f)
    {
        //TODO: dumping mechanism commands
        ROS_DEBUG_STREAM_COND(this->isDebugMode(), "ENTERED DUMPING STATE");
    }

    if(cmd.r_trig > 0.0f)
    {
        //TODO: dig commands
        ROS_DEBUG_STREAM_COND(this->isDebugMode(), "ENTERED DIG STATE");
    }
}

void RobotExec::autonomyExec(const robot_msgs::Autonomy& cmd)
{
    ROS_DEBUG_STREAM_COND(this->isDebugMode(), "EXECUTING AUTONOMY CMDS");
    LeftDrive.set_Speed(cmd.leftRatio);
    RightDrive.set_Speed(cmd.rightRatio);
}

void RobotExec::killMotors()
{
    ROS_DEBUG_STREAM_COND(this->isDebugMode(), "KILL MOTORS");
    LeftDrive.set_Speed(0);
    RightDrive.set_Speed(0);
    Lift.set_Speed(0);
    Storage.set_Speed(0);
    Bucket.set_Speed(0);
}

bool RobotExec::isOnPC()
{
    return onPC;
}

bool RobotExec::isAutonomyActive()
{
    return autonomyActive;
}

void RobotExec::setAutonomyActive(bool active)
{
    autonomyActive = active;
}

bool RobotExec::isDebugMode()
{
    return debug;
}

void RobotExec::setDebugMode(bool active)
{
    debug = active;
}

robot_msgs::MotorFeedback RobotExec::publishMotors()
{
    robot_msgs::MotorFeedback fb;

    //get RPM methods will be implemented here
    //for now, send dummy values

    ROS_DEBUG_STREAM_COND(this->isDebugMode(), "GETTING MOTOR DATA");

    //testing random dummy values
    fb.drumRPM = 1000;
    fb.liftPos = 77.77;
    fb.leftTreadRPM = 0;
    fb.rightTreadRPM = 16.101;

    return fb;
}
