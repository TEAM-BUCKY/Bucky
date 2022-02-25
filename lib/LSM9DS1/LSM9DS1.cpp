#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <LSM9DS1.h>

lsm9ds1::lsm9ds1(){
    Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
}

void lsm9ds1::setup(){
    if(!lsm.begin())
    {
    /* There was a problem detecting the LSM9DS1 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS1 detected ... Check your wiring!"));
    while(1);
    }
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

sensors_event_t lsm9ds1::read(){
    lsm.read();
    sensors_event_t a, m, g, t;
    lsm.getEvent(&a, &m, &g, &t);
    return m;
}