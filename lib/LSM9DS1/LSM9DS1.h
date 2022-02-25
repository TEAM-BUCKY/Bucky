#ifndef LSM9DS1_H
#define LSM9DS1_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

class lsm9ds1{
    private:
        Adafruit_LSM9DS1 lsm;
    public:
        lsm9ds1();
        void setup();
        sensors_event_t read();

};

#endif