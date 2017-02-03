// testController.cpp
// publishes random commands to the /robot/teleop topic

#include <ros/ros.h>
#include <string>
#include <robot2017/Teleop.h>

#include <stdlib.h>
#include <time.h>

bool randomBool()
{
  return rand() % 2 == 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "testController");

    ros::NodeHandle nh;

    // create a topic which will contain motor speed
    ros::Publisher teleopPub = nh.advertise<robot2017::Teleop>("/robot/teleop", 1000);

    ros::Rate rate(1);
    while(ros::ok())
    {
        // publish a teleop command message
        robot2017::Teleop command;

        command.a = randomBool();
        command.b = randomBool();
        command.x = randomBool();
        command.y = randomBool();
        command.lb = randomBool();
        command.rb = randomBool();
        command.back = randomBool();
        command.start = randomBool();
        command.l_thumb = randomBool();
        command.r_thumb = randomBool();
        command.x_l_thumb = rand();
        command.y_l_thumb = rand();
        command.x_r_thumb = rand();
        command.y_r_thumb = rand();
        command.l_trig = rand();
        command.y_trig = rand();

        teleopPub.publish(command);
        rate.sleep();
    }

    return 0;
}
