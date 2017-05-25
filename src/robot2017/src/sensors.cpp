#include "sensors.h"
#include <gpio.h>
#include <adc.h>

// GPIOs
const int gpioStatusLedR       = 61; // GPIO_61 = P8_26
const int gpioStatusLedG       = 88; // GPIO_88 = P8_28
const int gpioStatusLedB       = 89; // GPIO_89 = P8_30
const int gpioValues[3][3] = {
    {0, 0, 1}, // READY  = blue
    {0, 1, 0}, // ACTIVE = green
    {1, 0, 0}  // HIBER  = red
};

const int gpioStorageDownLimit = 65; // GPIO_65 = P8_18
const int gpioStorageUpLimit   = 11; // GPIO_11 = P8_32

// Potentiometers
const int adcRes = 4095;

const int ainLiftPot    = 0;   // AIN0 = P9_39
const int liftPotDegs   = 250;
const int liftPotOffset = -25;

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
    initPin(gpioStatusLedR);
    setPinDirection(gpioStatusLedR, (char*)"out");
    initPin(gpioStatusLedG);
    setPinDirection(gpioStatusLedG, (char*)"out");
    initPin(gpioStatusLedB);
    setPinDirection(gpioStatusLedB, (char*)"out");

    // Storage limits
    initPin(gpioStorageDownLimit);
    setPinDirection(gpioStorageDownLimit, (char*)"in");
    initPin(gpioStorageUpLimit);
    setPinDirection(gpioStorageUpLimit, (char*)"in");
}

void Sensors::setStatusLed(status_led_t value)
{
    const int* gpios = gpioValues[value];
    setPinValue(gpioStatusLedR, gpios[0]);
    setPinValue(gpioStatusLedG, gpios[1]);
    setPinValue(gpioStatusLedB, gpios[2]);
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
