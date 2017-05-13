/*
Virginia Tech Astrobotics 2017
Created by: Ryan Owens
Date: 5/7/2017
*/
#include <stdio.h>
#include "gpio.h"
#include <string.h>
int initPin(int pinnum)
{
	FILE *io;

	io = fopen("/sys/class/gpio/export", "w");
	fseek(io,0,SEEK_SET);
	fprintf(io,"%d",pinnum);
	fflush(io);
	fclose(io);

	return 0;
}

int setPinDirection(int pinnum, char* dir)
{
	FILE *pdir;
	char buf[50];

	//build file path to value file
	sprintf(buf,"/sys/class/gpio/gpio%d/direction", pinnum);

	pdir = fopen(buf, "w");

	fseek(pdir,0,SEEK_SET);
	fprintf(pdir,"%s",dir);
	fflush(pdir);
	fclose(pdir);

	return 0;
}

int setPinValue(int pinnum, int value)
{
	FILE *val;
	char buf[50];

	//build file path to value file
	sprintf(buf,"/sys/class/gpio/gpio%d/value", pinnum);

	val = fopen(buf, "w");
	fseek(val,0,SEEK_SET);
	fprintf(val,"%d",value);
	fflush(val);
	fclose(val);

	return 0;
}

int getPinValue(int pinnum)
{
	FILE *val;
	int value = 0;
	char buf[50];

	//build file path to value file
	sprintf(buf,"/sys/class/gpio/gpio%d/value", pinnum);

	val = fopen(buf, "r");
	fseek(val,0,SEEK_SET);
	fscanf(val,"%d",&value);
	fclose(val);

	return value;
}