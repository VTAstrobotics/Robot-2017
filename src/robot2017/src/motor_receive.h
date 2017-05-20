// motor_receive.h: handles the VESC receive data state machine
// temporary solution until generic state machine is implemented upstream

#ifndef MOTOR_RECEIVE
#define MOTOR_RECEIVE

#include "robot_exec.h"
#include <vesc_bbb_uart/timers.h>

// Need to call update() on a regular basis at high frequency (eg. within a loop, every few ms)
class MotorsReceive {
public:
    const int TIMER_SP = 17; // timer set-point in milliseconds.
                             // adjust for frequency of read data.
                             // setting too low may cause packets to be dropped.

    MotorsReceive(RobotExec& exec);

    void update();

private:
    // Robot currently has five motors - see robot_exec.h
    enum read_mode_t {
        RequestMotor1, ReadMotor1,
        RequestMotor2, ReadMotor2,
        RequestMotor3, ReadMotor3,
        RequestMotor4, ReadMotor4,
        RequestMotor5, ReadMotor5
    };

    RobotExec& exec;
    read_mode_t read_mode = RequestMotor1;
    timer_t* timerid = new_timer();
};

#endif
