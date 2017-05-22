// sensors.h
// Handles communication with sensors used by the robot

#ifndef SENSORS_H
#define SENSORS_H

class Sensors {
public:
    // Ready:     code started             (blue)
    // Active:    deadman pressed          (green)
    // Hibernate: lost connection to drive (red)
    enum status_led_t {READY, ACTIVE, HIBER};

    Sensors(bool onPC);

    // GPIOs
    void setStatusLed(status_led_t value);

    // Potentiometers
    float getLiftPosition();

private:
    void initGpio();

    bool onPC;
};

#endif
