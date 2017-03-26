#include "motors.h"

#ifdef MOTORS_VESC_UART
#include <vesc_bbb_uart/comm_uart.h>
#include <vesc_bbb_uart/bldc_interface.h>
#else
#include <beaglebone_pwm/BBB_Pwm.h>
#endif

#ifdef MOTORS_VESC_UART
// control variables // ! -- SET THESE VALUES BEFORE RUNNING -- ! //
const int max_erpm = 30;
const float duty = 0;
const float amps = 0;
const float brake = 0;
#else
const int talon_max_forward = 455;
const int talon_max_reverse = 135;
const int talon_neutral = 295;
#endif

void Motor::init() {
#ifdef MOTORS_VESC_UART
    // set the current in amps
    bldc_interface_set_current(amps);

    // set the brake current in amps
    bldc_interface_set_current_brake(brake);

    // set the duty cycle
    bldc_interface_set_duty_cycle(duty);
#else
    // initialize BBB_Pwm
    PWM bbb_pwm("P9_13");
    init_Pwm();
    bbb_pwm.enable_Pwm();
#endif
}

Motor::Motor(int num) : num(num) {
}

void Motor::set(float speed) {
#ifdef MOTORS_VESC_UART
    // TODO UART set speed
#else
    // TODO Talon PWM set speed
#endif
}
