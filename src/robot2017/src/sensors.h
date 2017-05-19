// sensors.h
// Handles communication with sensors used by the robot

#ifndef SENSORS_H
#define SENSORS_H

#include <sunrom.h>

class Sensors {
public:
    // Ready:     code started             (blue)
    // Active:    deadman pressed          (green)
    // Hibernate: lost connection to drive (red)
    enum status_led_t {READY, ACTIVE, HIBER};

    Sensors();

    // GPIOs
    void setStatusLed(status_led_t value);
    bool getKillButton();

    // Potentiometers
    float getLiftPosition();
    float getStoragePosition();

    // Load cells
    float getLeftStorageWeight();
    float getRightStorageWeight();

private:
    const float LOAD_CELL_SCALE = 0.38; // Default 40kg load cell scaling factor

    void initGpio();

    SUNROM leftLoadCell;
    SUNROM rightLoadCell;
};

#endif
