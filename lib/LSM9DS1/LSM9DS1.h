#ifndef LSM9DS1_H
#define LSM9DS1_H

#include <Arduino.h>
#include <Adafruit_LSM9DS1.h>

class lsm9ds1{
    private:
        Adafruit_LSM9DS1 lsm;
    public:
        lsm9ds1();
        void setup();
        void read();
        sensors_event_t a, m, g, t;
};

#endif