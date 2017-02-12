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

// #includes
#include <stdio.h>
#include <string.h>
#include <cstdlib>
#include <string>

// constant definitions
#define PERIOD 20000000
#define MINDUTY 0.025
#define MAXDUTY 0.118
#define MAXPOS 180
#define MINPOS 0
#define OCP 3
#define P9_22 "P9_22" // num = 15
#define P9_21 "P9_21" // num = 16
#define P9_42 "P9_42" // num = 17
#define P9_14 "P9_14" // num = 18
#define P9_16 "P9_16" // num = 19
#define P8_19 "P8_19" // num = 20
#define P8_13 "P8_13" // num = 21

using namespace std;

// Create file paths for PWM pins
// Must be called once before initializing any PWM object
void init_Pwm();

class PWM
{
public:
	
	// Constructors
	PWM(string pinSelect); // uses pre-defined file numbers
	PWM(string pinSelect, int ocp, int pinFile); // caller can specify file numbers manually
	
	// Destructor
	~PWM();

	// Set period 
	// in: PERIOD value in nano seconds. Default value is 20 nano seconds (50Hz).
	void set_Period(int period);

	// Set duty cycle 
	// in: duty cycle as a decimal percentage.  50% = 0.5
	void set_Duty(float duty);

	// Start PWM output
	void enable_Pwm();
	// Stop PWM output
	void disable_Pwm();

	// Set polarity of PWM pin
	// 0 = reversed
	// 1 = not reversed
	void set_Polarity(int polarity);
	
	// Set servo position
	// in: servo position in degrees
	//     0-180 degrees by default
	//     can modify range by changing MAXPOS and MINPOS
	void set_Position(int position);
	
private:
	
	int find_OcpFilePathNum();
	int find_PinFilePathNum();
	
	string pin;
	int pinFilePathNum;
	int ocpFilePathNum;
	
	int period_ns;
	int duty_ns;
	float dutyPercent;
	
};
