#include "sensors.h"
#include <gpio.h>

// GPIOs
const int gpioStatusLed = 67; // FIXME get the right pin number for this

// Load cells
const char* leftLoadCellPath  = "/dev/ttyO4";
const char* rightLoadCellPath = "/dev/ttyO5";

Sensors::Sensors()
    : leftLoadCell((char*) leftLoadCellPath, LOAD_CELL_SCALE),
      rightLoadCell((char*) rightLoadCellPath, LOAD_CELL_SCALE)
{
    initGpio();
}

void Sensors::initGpio()
{
    // Status LED
    initPin(gpioStatusLed);
    setPinDirection(gpioStatusLed, (char*)"in");
}

void Sensors::setStatusLed(status_led_t value)
{
    // TODO implement status LED
}

float Sensors::getLeftStorageWeight() {
    leftLoadCell.update_load();
    return leftLoadCell.get_load();
}

float Sensors::getRightStorageWeight() {
    rightLoadCell.update_load();
    return rightLoadCell.get_load();
}
