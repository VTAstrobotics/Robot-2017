#include "motors.h"

#ifdef MOTORS_VESC_UART
#include <bldc_uart_beaglebone/comm_uart.h>
#include <bldc_uart_beaglebone/bldc_interface.h>
#else
// TODO Talon PWM includes
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
    // TODO Talon PWM init if applicable
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
