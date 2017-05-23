#include "sensors.h"
#include <gpio.h>
#include <adc.h>

// GPIOs
const int gpioStatusLed        = 67; // FIXME get the right pin number for this
const int gpioStorageDownLimit = 65; // GPIO_65 = P8_18
const int gpioStorageUpLimit   = 11; // GPIO_11 = P8_32

// Potentiometers
const int adcRes = 4095;

const int ainLiftPot    = 0;   // AIN0 = P9_39
const int liftPotDegs   = 250;
const int liftPotOffset = -15;

Sensors::Sensors(bool onPC) : onPC(onPC)
{
    if(!onPC)
    {
        initGpio();
    }
}

void Sensors::initGpio()
{
    // Status LED
    initPin(gpioStatusLed);
    setPinDirection(gpioStatusLed, (char*)"out");

    // Storage limits
    initPin(gpioStorageDownLimit);
    setPinDirection(gpioStorageDownLimit, (char*)"in");
    initPin(gpioStorageUpLimit);
    setPinDirection(gpioStorageUpLimit, (char*)"in");
}

void Sensors::setStatusLed(status_led_t value)
{
    // TODO implement status LED
}

float Sensors::getLiftPosition()
{
    if(onPC) {
        return 0.0f;
    } else {
        int potRaw = get_adc(ainLiftPot);
        float potAngle = 1.0f * potRaw * liftPotDegs / adcRes;
        return potAngle + liftPotOffset;
    }
}

bool Sensors::getStorageDownLimit()
{
    return getPinValue(gpioStorageDownLimit);
}

bool Sensors::getStorageUpLimit()
{
    return getPinValue(gpioStorageUpLimit);
}
