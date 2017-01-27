/*
	Copyright 2015 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * comm_uart.c
 *
 *  Created on: 17 aug 2015
 *      Author: benjamin
 *  Modified on: 23 JAN 2017
 *      By: Ryan Owens
 *          - Modified for use with beaglebone black
 *          - Sending data only implemented
 *          - Receive data to do later
 */

#include "comm_uart.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include "bldc_interface_uart.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

// Settings
#define BAUDRATE B9600   // Change as needed, keep B

/* change this definition for the correct port */
#define MODEMDEVICE1 "/dev/ttyO1" //Beaglebone Black serial port (UART1)
#define MODEMDEVICE2 "/dev/ttyO2" //Beaglebone Black serial port (UART2)

#define _POSIX_SOURCE 1 /* POSIX compliant source */

// Not sure if needed for rx on BEAGLEBONE, keep for now - 1/21/2017
#define SERIAL_RX_BUFFER_SIZE	1024

// Private functions
static void send_packet(unsigned char *data, unsigned int len);

// Threads
// ******NOT FOR BEAGLEBONE******
/*static THD_FUNCTION(timer_thread, arg);
static THD_WORKING_AREA(timer_thread_wa, 512);
static THD_FUNCTION(packet_process_thread, arg);
static THD_WORKING_AREA(packet_process_thread_wa, 4096);
static thread_t *process_tp;*/

// Variables
/*
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0; */

// ********************NOT NEEDED FOR BEAGLEBONE BLACK***********************
/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */

/*static void txend1(UARTDriver *uartp) {
	(void)uartp;
}*/

/*
 * This callback is invoked when a transmission has physically completed.
 */
/*static void txend2(UARTDriver *uartp) {
	(void)uartp;
}*/

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
/*static void rxerr(UARTDriver *uartp, uartflags_t e) {
	(void)uartp;
	(void)e;
}*/

// ******MAY NEED TO BE RE-IMPLEMENTED FOR BEAGLEBONE******
/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
/*static void rxchar(UARTDriver *uartp, uint16_t c) {
	(void)uartp;*/

	/*
	 * Put the character in a buffer and notify a thread that there is data
	 * available. An alternative way is to use
	 *
	 * packet_process_byte(c);
	 *
	 * here directly and skip the thread. However, this could drop bytes if
	 * processing packets takes a long time.
	 */
/*
	serial_rx_buffer[serial_rx_write_pos++] = c;

	if (serial_rx_write_pos == SERIAL_RX_BUFFER_SIZE) {
		serial_rx_write_pos = 0;
	}

	chEvtSignalI(process_tp, (eventmask_t) 1);
}*/

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
/*static void rxend(UARTDriver *uartp) {
	(void)uartp;
}*/

// ******RECEIVE THREAD WILL NOT WORK WITH TERMIOS ON BEAGLEBONE******
// ******WILL IMPLEMENT RECEIVE FUNCTION AFTER KENT STATE******
/*static THD_FUNCTION(packet_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("comm_uart");

	process_tp = chThdGetSelfX();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		
		 // Wait for data to become available and process it as long as there is data.
		 

		while (serial_rx_read_pos != serial_rx_write_pos) {
			bldc_interface_uart_process_byte(serial_rx_buffer[serial_rx_read_pos++]);

			if (serial_rx_read_pos == SERIAL_RX_BUFFER_SIZE) {
				serial_rx_read_pos = 0;
			}
		}
	}
}*/

/**
 * Callback that the packet handler uses to send an assembled packet.
 *
 * @param data
 * Data array pointer
 * @param len
 * Data array length
 */
static void send_packet(unsigned char *data, unsigned int len) {
	if (len > (PACKET_MAX_PL_LEN + 5)) {
		return;
	}

	// Wait for the previous transmission to finish.
	tcdrain(fd);

	// Copy this data to a new buffer in case the provided one is re-used
	// after this function returns.
	static uint8_t buffer[PACKET_MAX_PL_LEN + 5];
	memcpy(buffer, data, len);

	// Send the data over UART
	write(fd, buffer, len);
}

// ******NOT NEED FOR BEAGLEBONE BLACK******
/**
 * This thread is only for calling the timer function once
 * per millisecond. Can also be implementer using interrupts
 * if no RTOS is available.
 */
/*static THD_FUNCTION(timer_thread, arg) {
	(void)arg;
	chRegSetThreadName("packet timer");

	for(;;) {
		bldc_interface_uart_run_timer();
		chThdSleepMilliseconds(1);
	}
}*/

void comm_uart_init() {
	
	// Initialize UART
	
	struct termios newtio;
	
	bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

    /* BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
       CS8     : 8n1 (8bit,no parity,1 stopbit)
       CLOCAL  : local connection, no modem contol
       CREAD   : enable receiving characters */
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;

    /* IGNPAR  : ignore bytes with parity errors
       otherwise make device raw (no other input processing) */
    newtio.c_iflag = IGNPAR;

    /*  Raw output  */
    newtio.c_oflag = 0;
    
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN]  = 0;

    /* ICANON  : enable canonical input
       disable all echo functionality, and don't send signals to calling program */
    newtio.c_lflag = ICANON;
	
    // Load the pin configuration

	int ret = system("echo BB-UART1 > /sys/devices/bone_capemgr.9/slots");
	/* Open modem device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C. */
    fd = open(MODEMDEVICE1, O_RDWR | O_NOCTTY );
    if (fd < 0) { perror(MODEMDEVICE1); exit(-1); }
    
    /* now clean the modem line and activate the settings for the port */
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd,TCSANOW,&newtio);
    
	// Initialize the bldc interface and provide a send function
	bldc_interface_uart_init(send_packet);
	
	
	// ********************************CODE FOR STM32f4xx**************************************
	// **************WILL NEED TO CONVERT OR REPLACE THREADS FOR RECEIVING DATA****************
	// Check termios flags to make sure they match
	/* 
	uartStart(&UART_DEV, &uart_cfg);
	palSetPadMode(UART_TX_PORT, UART_TX_PIN, PAL_MODE_ALTERNATE(UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
	palSetPadMode(UART_RX_PORT, UART_RX_PIN, PAL_MODE_ALTERNATE(UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP); */

	/* NO PROCESSING THREAD ON BBB
	// Start processing thread
	chThdCreateStatic(packet_process_thread_wa, sizeof(packet_process_thread_wa),
			NORMALPRIO, packet_process_thread, NULL); 

	// Start timer thread
	chThdCreateStatic(timer_thread_wa, sizeof(timer_thread_wa),
			NORMALPRIO, timer_thread, NULL); */
}
