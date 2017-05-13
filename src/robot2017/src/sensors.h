// sensors.h
// Handles communication with sensors used by the robot

#ifndef SENSORS_H
#define SENSORS_H

#include <sunrom.h>

class Sensors {
public:
    Sensors();

private:
    void initGpio();
    void initLoadCells();

    
};

#endif
