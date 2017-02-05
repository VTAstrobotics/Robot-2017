// testController.cpp
// publishes random commands to the /robot/teleop topic

#include <ros/ros.h>
#include <string>
#include <teleop_msg/Teleop.h>

#include <stdlib.h>
#include <time.h>

bool randomBool()
{
  return rand() % 2 == 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_controller");

    ros::NodeHandle nh;

    // create a topic which will contain motor speed
    ros::Publisher teleopPub = nh.advertise<robot2017::Teleop>("/robot/teleop", 1000);
    int count = 0;
    while(ros::ok())
    {
        ros::Duration(2).sleep(); //sleep for 2 sec between messages


        // publish a teleop command message
        robot2017::Teleop command;

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
    }

    return 0;
}
