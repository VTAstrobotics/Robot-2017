#include <stdio.h>
#include <string.h>
#include "comm_uart.h"
#include "bldc_interface.h"

/*
void main_printf(const char *fmt, ...) {
	va_list ap;

	va_start(ap, fmt);
	chvprintf((BaseSequentialStream*)&SDU1, fmt, ap);
	va_end(ap);
}

static void cmd_val(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: val\r\n");
		return;
	}

	bldc_interface_get_values();
}



void bldc_val_received(mc_values *val) {
	main_printf("\r\n");
	main_printf("Input voltage: %.2f V\r\n", val->v_in);
	main_printf("Temp:          %.2f degC\r\n", val->temp_pcb);
	main_printf("Current motor: %.2f A\r\n", val->current_motor);
	main_printf("Current in:    %.2f A\r\n", val->current_in);
	main_printf("RPM:           %.1f RPM\r\n", val->rpm);
	main_printf("Duty cycle:    %.1f %%\r\n", val->duty_now * 100.0);
	main_printf("Ah Drawn:      %.4f Ah\r\n", val->amp_hours);
	main_printf("Ah Regen:      %.4f Ah\r\n", val->amp_hours_charged);
	main_printf("Wh Drawn:      %.4f Wh\r\n", val->watt_hours);
	main_printf("Wh Regen:      %.4f Wh\r\n", val->watt_hours_charged);
	main_printf("Tacho:         %i counts\r\n", val->tachometer);
	main_printf("Tacho ABS:     %i counts\r\n", val->tachometer_abs);
	main_printf("Fault Code:    %s\r\n", bldc_interface_fault_to_string(val->fault_code));
}
*/

int main(void) {

	// variables
	int command = 0;
	int erpm = 0;
	
	float duty = 0;
	float amps = 0;
	float brake = 0;
	
	// For the UART interface
	comm_uart_init();

	// Give bldc_interface a function to call when valus are received.
	//bldc_interface_set_rx_value_func(bldc_val_received);

	// Main loop 
	for(;;) {
		printf("Choose a command\n");
		printf("    1 : Set speed\n");
		printf("    2 : Set current\n");
		printf("    3 : Set brake current\n");
		printf("    4 : Set duty cycle\n");
		printf("Enter a number: ");
		scanf("%d", &command);
		switch(command) {
			case 1:
				printf("Enter desired speed in ERPM: ");
				scanf("%d", &erpm);
				bldc_interface_set_rpm(erpm);
				printf("Speed set to %d rpm\n\n", erpm);
				break;
			case 2:
				printf("Enter desired current in Amps: ");
				scanf("%f", &amps);
				bldc_interface_set_current(amps);
				printf("Current set to %f amps\n\n", amps);
				break;
			case 3:
				printf("Enter desired brake current in Amps: ");
				scanf("%f", &brake);
				bldc_interface_set_current_brake(brake);
				printf("Brake current set to %f amps\n\n", brake);
				break;
			case 4:
				printf("Enter desired duty cycle (percentage expressed as decimal): ");
				scanf("%f", &duty);
				bldc_interface_set_duty_cycle(duty);
				printf("Duty cycle set to %f\n\n", duty);
				break;
			default:
				break;
		}
		command = 0;
		erpm = 0;
		duty = 0;
		amps = 0;
		brake = 0;
		}
		
}
