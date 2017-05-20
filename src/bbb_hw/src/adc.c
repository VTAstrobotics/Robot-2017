/*
Virginia Tech Astrobotics 2017
Created by: Ryan Owens
Date: 5/7/2017
*/
#include "adc.h"
#include <stdio.h>

int get_adc(int adc) {
	int value;
	FILE *file;
	char buf[64];
	sprintf(buf, "/sys/bus/iio/devices/iio:device0/in_voltage%d_raw", adc);
	file = fopen(buf, "r");
	fseek(file,0,SEEK_SET);
	fscanf(file,"%d",&value);
	fflush(file);
	fclose(file);
	return value;
}