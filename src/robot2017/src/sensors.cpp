#include "sensors.h"
#include <gpio.h>
#include <adc.h>

// GPIOs
const int gpioStatusLed  = 67; // FIXME get the right pin number for this

// Potentiometers
const int ainLiftPot    = 0; // AIN0 = P9_39
const int ainStoragePot = 1; // AIN1 = P9_40

const int adcRes         = 4096;
const int liftPotDegs    = 250;
const int storagePotDegs = 360;

// Load cells
const char* leftLoadCellPath  = "/dev/ttyO4";
const char* rightLoadCellPath = "/dev/ttyO5";

Sensors::Sensors(bool onPC) : onPC(onPC)
{
    if(!onPC)
    {
        initGpio();
        initLoadCells();
    }
}

void Sensors::initGpio()
{
    // Status LED
    initPin(gpioStatusLed);
    setPinDirection(gpioStatusLed, (char*)"out");
}

void Sensors::initLoadCells()
{
    leftLoadCell  = new SUNROM((char*) leftLoadCellPath,  LOAD_CELL_SCALE);
    rightLoadCell = new SUNROM((char*) rightLoadCellPath, LOAD_CELL_SCALE);
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
        return 1.0f * potRaw * liftPotDegs / adcRes;
    }
}

float Sensors::getStoragePosition()
{
    if(onPC) {
        return 0.0f;
    } else {
        int potRaw = get_adc(ainStoragePot);
        return 1.0f * potRaw * storagePotDegs / adcRes;
    }
}

float Sensors::getLeftStorageWeight()
{
    if(onPC) {
        return 0.0f;
    } else {
        leftLoadCell->update_load();
        return leftLoadCell->get_load();
    }
}

float Sensors::getRightStorageWeight()
{
    if(onPC) {
        return 0.0f;
    } else {
        rightLoadCell->update_load();
        return rightLoadCell->get_load();
    }
}
