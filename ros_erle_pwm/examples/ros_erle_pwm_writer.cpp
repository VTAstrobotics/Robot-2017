#include "ros/ros.h"
#include "ros_erle_pwm/pwm.h"
#include <cstdlib>
#include <iostream>
#include <string>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_erle_pwm_client");
  if (argc != 3){
    ROS_INFO("usage: ros_erle_pwm_client index PWM");
    return 1;
  }

  std::string topic_name = std::string("PWM_") + std::string(argv[1]);

  std::cout << topic_name << std::endl;

  ros::NodeHandle n;
  ros::Publisher led_pub = n.advertise<ros_erle_pwm::pwm>(topic_name, 1000);
  ros_erle_pwm::pwm msg;

  msg.PWM = atoi(argv[2]);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()){
    led_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }  
  return 0;
}
