#include <Arduino.h>
#include <Wire.h>
#include <SparkFunLSM9DS1.h>
#include <LSM9DS1.h>

// make LSM9DS1 object
LSM9DS1 sensor;

Lsm9ds1::Lsm9ds1(int avgAmount, int DECLINATION){
    Lsm9ds1::avgAmount = avgAmount;
    Lsm9ds1::DECLINATION = DECLINATION;
}

void Lsm9ds1::setup() {
    // start Wire
    Wire.begin();
    if (sensor.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
    {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1);
    }
    // set scale to 16
    sensor.settings.mag.scale = 16;
    sensor.settings.device.commInterface = IMU_MODE_I2C; // Set mode to I2C
}

float Lsm9ds1::magCalculate() {
    return (Lsm9ds1::magGetHeading() - average);
}


void Lsm9ds1::magCalibrate() {
    float sum = 0.0; // define sum
    // Get values
    for (int i = 0; i < avgAmount; i++) { 
        float val = Lsm9ds1::magGetHeading();
        Serial.println("Found values: " + String(val));
        sum += val;
        
    }
    // Calc and give average
    average = sum/avgAmount; 
    Serial.println("Average value: " + String(average));

}


float Lsm9ds1::magGetHeading() {
    float heading; // gives heading a data type
    sensor.readMag(); // read the magnetometer
    // define variables
    float my = -sensor.my;
    float mx = -sensor.mx;
    // calculate heading
    if (my == 0) {
        heading = (mx < 0) ? PI : 0;
    } else {
        heading = atan2(mx, my);
    }
    // subtract Declination
    heading -= DECLINATION * PI / 180;

    if (heading > PI) {
        heading -= (2 * PI);
    }
    else if (heading < -PI) {
        heading += (2 * PI);
    }
    
    // rad2deg
    heading *= 180.0 / PI;
    return heading;
}