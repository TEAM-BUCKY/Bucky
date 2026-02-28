//
// Created by koen on 2/28/26.
//

#ifndef BUCKY_COMPASS_H
#define BUCKY_COMPASS_H

#include <Wire.h>
#include <Adafruit_LSM6DS3TRC.h>

class Compass
{
    private:
        Adafruit_LSM6DS3TRC imu;
        float heading = 0;
        float gyroBiasZ = 0;
        unsigned long lastUpdateMicros = 0;

    public:
        bool init(TwoWire& wire, int calibrationSamples = 500);
        void update();
        float getHeading() const;
        void reset();
};

#endif //BUCKY_COMPASS_H
