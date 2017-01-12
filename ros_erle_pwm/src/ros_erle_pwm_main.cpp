#include "ros/ros.h"
#include "ros_erle_pwm/pwm.h"

#include "RC_Channel.h"

#define NUM_PWM 8

RC_Channel* c[NUM_PWM];

void copy_input_output(void)
{
	for (int i = 0; i < NUM_PWM; i++){
    	c[i]->servo_out = c[i]->control_in;
    	c[i]->calc_pwm();
    	c[i]->output();
  	}
}


void PWM_1_Callback(const ros_erle_pwm::pwm::ConstPtr& msg){
  ROS_INFO("I heard: PWM_1:[%d]", msg->PWM);
  c[0]->set_pwm(msg->PWM);
  copy_input_output();
}
void PWM_2_Callback(const ros_erle_pwm::pwm::ConstPtr& msg){
  ROS_INFO("I heard: PWM_2:[%d]", msg->PWM);
  c[1]->set_pwm(msg->PWM);
  copy_input_output();
}
void PWM_3_Callback(const ros_erle_pwm::pwm::ConstPtr& msg){
  ROS_INFO("I heard: PWM_3:[%d]", msg->PWM);
  c[2]->set_pwm(msg->PWM);
  copy_input_output();
}
void PWM_4_Callback(const ros_erle_pwm::pwm::ConstPtr& msg){
  ROS_INFO("I heard: PWM_4:[%d]", msg->PWM);
  c[3]->set_pwm(msg->PWM);
  copy_input_output();
}
void PWM_5_Callback(const ros_erle_pwm::pwm::ConstPtr& msg){
  ROS_INFO("I heard: PWM_5:[%d]", msg->PWM);
  c[4]->set_pwm(msg->PWM);
  copy_input_output();
}
void PWM_6_Callback(const ros_erle_pwm::pwm::ConstPtr& msg){
  ROS_INFO("I heard: PWM_6:[%d]", msg->PWM);
  c[5]->set_pwm(msg->PWM);
  copy_input_output();
}
void PWM_7_Callback(const ros_erle_pwm::pwm::ConstPtr& msg){
  ROS_INFO("I heard: PWM_7:[%d]", msg->PWM);
  c[6]->set_pwm(msg->PWM);
  copy_input_output();
}
void PWM_8_Callback(const ros_erle_pwm::pwm::ConstPtr& msg){
  ROS_INFO("I heard: PWM_8:[%d]", msg->PWM);
  c[7]->set_pwm(msg->PWM);
  copy_input_output();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pwm");

  for (int i = 0; i < NUM_PWM; i++){
  	c[i] = new RC_Channel(i);
  	c[i]->set_range(0,1000);
    c[i]->set_type(RC_CHANNEL_TYPE_RANGE);
    c[i]->enable_out();
  }

  ros::NodeHandle n;

  ros::Subscriber sub[NUM_PWM];
  sub[0] = n.subscribe("PWM_1", 1000, PWM_1_Callback);
  sub[1] = n.subscribe("PWM_2", 1000, PWM_2_Callback);
  sub[2] = n.subscribe("PWM_3", 1000, PWM_3_Callback);
  sub[3] = n.subscribe("PWM_4", 1000, PWM_4_Callback);
  sub[4] = n.subscribe("PWM_5", 1000, PWM_5_Callback);
  sub[5] = n.subscribe("PWM_6", 1000, PWM_6_Callback);
  sub[6] = n.subscribe("PWM_7", 1000, PWM_7_Callback);
  sub[7] = n.subscribe("PWM_8", 1000, PWM_8_Callback);

  ros::spin();

  return 0;
}

