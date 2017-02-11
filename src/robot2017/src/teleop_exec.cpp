#include "teleop_exec.h"
#include "robot_msgs/Teleop.h"
#include "motors.h"
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <string>

bool dead = true;
float leftRatio;
float rightRatio;

Motor driveLeft(0);
Motor driveRight(1);

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

void TeleopExec::killMotors()
{
    //TODO
    ROS_INFO_STREAM("KILL MOTORS");
}
