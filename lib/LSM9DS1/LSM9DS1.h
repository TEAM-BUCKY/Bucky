#ifndef LSM9DS1_H
#define LSM9DS1_H

#include <Arduino.h>
#include <Wire.h>
#include <SparkFunLSM9DS1.h>

class Lsm9ds1 {
    private: 
        int avgAmount, DECLINATION;
        float sum;
    public:
        float average;
        Lsm9ds1(int avgAmount, int DECLINATION);
        void setup();
        void magCalibrate();
        float magGetHeading();
        float magCalculate();

};

#endif