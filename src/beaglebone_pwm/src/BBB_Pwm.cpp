/*
Virginia Tech Astrobotics 2017
Created by: Ryan Owens
Date: 2/6/2017
***********************************************
Rev 1 : Added position control for servo motors
Modified: 2/9/2017
***********************************************
PWM Library for a BeagleBone Black. 
Device tree is dependent on Linux Distribution.
File paths may need to be modified if used with a different distribution.

** To Do **
// Add ability to determine file paths programatically after file tree has been created
***********
*/
#include "BBB_Pwm.h"
#include <stdio.h>
#include <string.h>
#include <cstdlib>
#include <string>
using namespace std;

//********************************************************************
// Pre: None
// Post: File paths are created for PWM
//********************************************************************
void init_Pwm() {
	system("sudo echo am33xx_pwm > /sys/devices/bone_capemgr.9/slots");
	system("sudo echo bone_pwm_P9_22 > /sys/devices/bone_capemgr.9/slots");
	system("sudo echo bone_pwm_P9_21 > /sys/devices/bone_capemgr.9/slots");
	system("sudo echo bone_pwm_P9_42 > /sys/devices/bone_capemgr.9/slots");
	system("sudo echo bone_pwm_P9_14 > /sys/devices/bone_capemgr.9/slots");
	system("sudo echo bone_pwm_P9_16 > /sys/devices/bone_capemgr.9/slots");
	system("sudo echo bone_pwm_P8_19 > /sys/devices/bone_capemgr.9/slots");
	system("sudo echo bone_pwm_P8_13 > /sys/devices/bone_capemgr.9/slots");
}

//********************************************************************
// Pre: init_Pwm() has been called at least once
// Post: PWM object initialized with ocpFile and pinFile determined programatically
//********************************************************************
PWM::PWM(string pinSelect)
{
	pin = pinSelect;
	period_ns = PERIOD; // frequency error correction
	duty_ns = 0;
	dutyPercent = 0;
	ocpFilePathNum = find_OcpFilePathNum();
	pinFilePathNum = find_PinFilePathNum();
	set_Period(period_ns);
}

//********************************************************************
// Pre: init_Pwm() has been called at least once
// Post: PWM object initialized with ocpFile and pinFile set by caller
//********************************************************************
PWM::PWM(string pinSelect, int ocpFileNum, int pinFileNum)
{
	pin = pinSelect;
	period_ns = PERIOD;

	duty_ns = 0;
	dutyPercent = 0;
	ocpFilePathNum = ocpFileNum;
	pinFilePathNum = pinFileNum;
	set_Period(period_ns);
}

PWM::~PWM()
{
}

// NOT FINISHED
int PWM::find_OcpFilePathNum()
{
	return OCP;
}
// NOT FINISHED
int PWM::find_PinFilePathNum()
{
	if (pin == P9_22)
		return 15;
	else if (pin == P9_21)
		return 16;
	else if (pin == P9_42)
		return 17;
	else if (pin == P9_14)
		return 18;
	else if (pin == P9_16)
		return 19;
	else if (pin == P8_19)
		return 20;
	else if (pin == P8_13)
		return 21;
	else 
		return 0;
}

//********************************************************************
// Pre: PWM file paths have been created
// Post: Period has been set in nano seconds
//********************************************************************
void PWM::set_Period(int period) 
{
	char buf[255];
	period_ns = period; 
	sprintf(buf, "echo %d > /sys/devices/ocp.%d/pwm_test_%s.%d/period", period_ns, ocpFilePathNum, pin.c_str(), pinFilePathNum);
	system(buf);
}

//********************************************************************
// Pre: PWM file paths have been created and Period has been set
// Post: Duty cycle has been set in nano seconds
//********************************************************************
void PWM::set_Duty(float duty) 
{
	char buf[255];
	int duty_ns;
	dutyPercent = duty;
	duty_ns = duty*period_ns;
	sprintf(buf, "echo %d > /sys/devices/ocp.%d/pwm_test_%s.%d/duty", duty_ns, ocpFilePathNum, pin.c_str(), pinFilePathNum);
	system(buf);
}
//********************************************************************
// Pre: PWM file paths have been created and Period has been set
// Post: Servo position has been set
//********************************************************************
void PWM::set_Position(int position)
{
	float duty;
	duty = ((MAXDUTY-MINDUTY)/(MAXPOS - MINPOS) * position) + MINDUTY;
	if (duty > MAXDUTY)
		duty = MAXDUTY;
	else if (duty < MINDUTY)
		duty = MINDUTY;
	set_Duty(duty);
}
//********************************************************************
// Pre: PWM file paths have been created
// Post: Pin polarity has been set
//********************************************************************
void PWM::set_Polarity(int polarity) 
{
	char buf[255];
	sprintf(buf, "echo %d > /sys/devices/ocp.%d/pwm_test_%s.%d/polarity", polarity, ocpFilePathNum, pin.c_str(), pinFilePathNum);
	system(buf);
}
//********************************************************************
// Pre: PWM file paths have been created
// Post: PWM output signal is enabled
//********************************************************************
void PWM::enable_Pwm() 
{
	char buf[255];
	sprintf(buf, "echo 1 > /sys/devices/ocp.%d/pwm_test_%s.%d/run", ocpFilePathNum, pin.c_str(), pinFilePathNum);
	system(buf);
}
//********************************************************************
// Pre: PWM file paths have been created
// Post: PWM output signal is disabled
//********************************************************************
void PWM::disable_Pwm() 
{
	char buf[255];
	sprintf(buf, "echo 0 > /sys/devices/ocp.%d/pwm_test_%s.%d/run", ocpFilePathNum, pin.c_str(), pinFilePathNum);
	system(buf);
}