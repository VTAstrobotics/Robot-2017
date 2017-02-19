#include "teleop_exec.h"
#include "robot_msgs/Teleop.h"
#include "motors.h"
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <string>

dead = true;
autonomyActive = false;

float leftRatio;
float rightRatio;

Motor driveLeft(0);
Motor driveRight(1);

void teleopReceived(const robot_msgs::Teleop& cmd)
{
    std::stringstream message;

    message << "Teleop Command recieved: ";
    message << " " << +cmd.a << " " << +cmd.b << " " << +cmd.x << " " << +cmd.y;
    message << " " << +cmd.lb << " " << +cmd.rb << " " << +cmd.back;
    message << " " << +cmd.start << " " << +cmd.l_thumb << " " << +cmd.r_thumb;
    message << " (" << +cmd.x_l_thumb << " " << +cmd.y_l_thumb << ")";
    message << " (" << +cmd.x_r_thumb << " " << +cmd.y_r_thumb << ")";
    message << " (" << +cmd.l_trig << " " << +cmd.r_trig << ")";

    ROS_INFO_STREAM(message.str());


    //if autonomy mode is toggled (y button is pressed)
    if (cmd.y == 1)
        autonomyActive = !autonomyActive; //toggle autonomy state
    if(autonomyActive)
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

void autonomyReceived(const robot_msgs::Autonomy& cmd)
{
    std::stringstream message;

    message << "Autonomy Command recieved: ";
    message << " " << +cmd.a << " " << +cmd.b << " " << +cmd.x << " " << +cmd.y;
    message << " " << +cmd.lb << " " << +cmd.rb << " " << +cmd.back;

    ROS_INFO_STREAM(message.str());

    if(autonomyActive)
    {
        autonomyExec(cmd);
    }
}

void TeleopExec::teleopExec(const robot_msgs::Teleop& cmd)
{
    dead = !cmd.lb;
    //write LED??? (did this last year)
    if(dead)
    {
        killMotors();
        return;
    }

    if(cmd.y_l_thumb > 0)
    {
        if(cmd.x_r_thumb > 0)
        {
            leftRatio = cmd.y_l_thumb - cmd.x_r_thumb;
            rightRatio = fmax(cmd.y_l_thumb, cmd.x_r_thumb);
        }
        else
        {
            leftRatio = fmax(cmd.y_l_thumb, -cmd.x_r_thumb);
            rightRatio = cmd.y_l_thumb + cmd.x_r_thumb;
        }
    }
    else
    {
        if(cmd.x_r_thumb > 0)
        {
            leftRatio = -fmax(cmd.y_l_thumb, cmd.x_r_thumb);
            rightRatio = cmd.y_l_thumb +cmd.x_r_thumb;
        }
        else
        {
            leftRatio = cmd.y_l_thumb - cmd.x_r_thumb;
            rightRatio = -fmax(-cmd.y_l_thumb, -cmd.x_r_thumb);
        }
    }

    //TODO: set motor speeds using ratios
    //currently just printing ratios for debugging purpose
    std::stringstream msg;
    msg << "Left Ratio " << leftRatio << ", Right Ratio " << rightRatio;
    ROS_INFO_STREAM(msg.str());

    if(fabs(leftRatio) > 0.1 || fabs(rightRatio))
        return;

    if(cmd.l_trig > 1) //not sure what trigger ranges are
    {
        //TODO: dumping mechanism commands
        ROS_INFO_STREAM("ENTERED DUMPING STATE");
    }

    if(cmd.r_trig > 1)
    {
        //TODO: dig commands
        ROS_INFO_STREAM("ENTERED DIG STATE");
    }
}

void TeleopExec::autonomyExec(const robot_msgs::Autonomy& cmd)
{

}

void TeleopExec::killMotors()
{
    //TODO
    ROS_INFO_STREAM("KILL MOTORS");
}


