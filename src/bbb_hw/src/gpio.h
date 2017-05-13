/*
Virginia Tech Astrobotics 2017
Created by: Ryan Owens
Date: 5/7/2017
*/
#ifndef GPIO_H
#define GPIO_H

#ifdef __cplusplus
extern "C" {
#endif
// pinnum = gpio number
// check pinout to get gpio number
// dir = "in", "out"
int initPin(int pinnum);
int setPinDirection(int pinnum, char* dir);
int setPinValue(int pinnum, int value);
int getPinValue(int pinnum);
#ifdef __cplusplus
}
#endif
#endif