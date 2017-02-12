// test code for PWM library
#include "BBB_Pwm.h"
#include <unistd.h> // for sleep() function
#include <iostream>


using namespace std;

int main(void) {

	bool loop = 1;
	float duty;
	int pos = 0;
	// initialize file tree for PWM pins
	init_Pwm(); 
	
	// initalize servo object
	PWM servo(P9_14);
	// setters
	servo.set_Duty(0.0);
	servo.set_Polarity(0);
	// begin PWM output
	servo.enable_Pwm();
	
	float i = 0.0;
	
	for(i = MINDUTY ; i <= MAXDUTY; i+= 0.0001) {
		usleep(1000);
		servo.set_Duty(i);
	};
	for(i = MAXDUTY; i >= MINDUTY; i-= 0.0001) {
		usleep(1000);
		servo.set_Duty(i);
	};
	// Main loop 
	while (loop)
	{
		cout << "Enter Position 0-180 degrees: ";
		cin >> pos;
		servo.set_Position(pos);
		cout << endl << "Continue?" << endl << "1 = y, 0 = n" << endl;
		cin >> loop;
		cout << endl;
	}
	// stop PWM output
	servo.disable_Pwm();
	
	return 0;
};

