This is a library for UART communication between the VESC and a BeagleBone Black, based off the original vedderb/bldc_uart_comm_stm32f4_discovery repository on GitHub.

For now, only setters are working. Getters and CAN forwarding are not working. This means each motor controller must be directly connected to a UART port on the BeagleBone.
Currently only up to two controllers can be connected, to the first two UART ports.

The original tutorial for this library and instructions for porting to other platforms can be found here:
http://vedder.se/2015/10/communicating-with-the-vesc-using-uart/
