// motors.h
// handles communicaton with motor controllers

#ifndef MOTORS_H
#define MOTORS_H

//#define MOTORS_VESC_UART
#define MOTORS_TALON_PWM

class Motor {
public:
    static void init();

    Motor(int num);
    // speed is in range [-1.0, 1.0]
    void set(float speed);

private:
    int num;
};

#endif
