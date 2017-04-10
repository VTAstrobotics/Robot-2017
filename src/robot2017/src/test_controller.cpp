// testController.cpp
// publishes random commands to the /robot/teleop topic

#include <ros/ros.h>
#include <string>
#include <robot_msgs/Teleop.h>
#include <robot_msgs/Autonomy.h>
#include <robot_msgs/MotorFeedback.h>

#include <stdlib.h>
#include <time.h>

bool randomBool()
{
  return rand() % 2 == 1;
}

void printFbCmd(const robot_msgs::MotorFeedback& cmd)
{
    //see if callback function is actually called
    ROS_WARN_STREAM("Inside feedback callback");

    std::stringstream message;

    message << "Feedback Command recieved: ";
    message << " " << +cmd.drumRPM << " " << +cmd.liftPos;
    message << " " << +cmd.leftTreadRPM << " " << +cmd.rightTreadRPM;

    ROS_INFO_STREAM(message.str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_controller");

    ros::NodeHandle nh;

    // create a topic for teleop cmds
    ros::Publisher teleopPub = nh.advertise<robot_msgs::Teleop>("/robot/teleop", 1000);

    //create topic for autonomy cmds
    ros::Publisher autonomyPub = nh.advertise<robot_msgs::Autonomy>("/robot/autonomy", 1000);

    //subscribe to MotorFeedback topic
    ros::Subscriber fbSub = nh.subscribe("/robot/autonomy/feedback", 100, &printFbCmd);
    
    int count = 0;
    while(ros::ok())
    {
        ros::spinOnce();
        ros::Duration(2).sleep(); //sleep for 2 sec between messages


        // publish a teleop command message
        robot_msgs::Teleop command;

        command.a = randomBool();
        command.b = randomBool();
        command.x = randomBool();
        command.y = randomBool();
        //command.lb = randomBool();
        command.rb = randomBool();
        command.back = randomBool();
        command.start = randomBool();
        command.l_thumb = randomBool();
        command.r_thumb = randomBool();
        command.x_l_thumb = rand();
        //command.y_l_thumb = rand();
        //command.x_r_thumb = rand();
        command.y_r_thumb = rand();
        //command.l_trig = rand();
        //command.r_trig = rand();

        if (count % 4 == 0) //State 1: deadman switch off (dead)
        {
            command.lb = 0;
            command.y_l_thumb = rand();
            command.x_r_thumb = rand();
            command.l_trig = rand();
            command.r_trig = rand();
        }
        else
        {
            command.lb = 1;
            if (count % 4 == 1) //State 2: driving
            {
                command.y_l_thumb = rand() + 1;
                command.x_r_thumb = rand() + 1;
                command.l_trig = rand();
                command.r_trig = rand();
            }
            else
            {
                command.y_l_thumb = 0;
                command.x_r_thumb = 0;
                if (count % 4 == 2) //State 3: Digging
                {
                    command.r_trig = (rand() % 99) + 1;
                    command.l_trig = 0;
                }
                else //State 4: Dumping
                {
                    command.r_trig = 0;
                    command.l_trig = (rand() % 99) + 1;

                }
            }
        }
        count++;
        teleopPub.publish(command);


        //publish autonomy message
        robot_msgs::Autonomy autonomyCmd;
        autonomyCmd.leftRatio = rand();
        autonomyCmd.rightRatio = rand();
        autonomyCmd.digCmd = rand();
        autonomyCmd.dumpCmd = rand();
        
        autonomyPub.publish(autonomyCmd);
    }

    return 0;
}