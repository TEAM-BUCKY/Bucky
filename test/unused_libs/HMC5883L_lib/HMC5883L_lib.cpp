#include <Arduino.h>
#include <Core.h>
#include <Wire.h>
#include <HMC5883L_lib.h>
#include <HMC5883L.h>

HMC5883L compass;

HMC5883L_Calibrate::HMC5883L_Calibrate()
{
}

void HMC5883L_Calibrate::setup()
{
    while (!compass.begin())
    {
        delay(500);
    }

    // Set measurement range
    compass.setRange(HMC5883L_RANGE_1_3GA);

    // Set measurement mode
    compass.setMeasurementMode(HMC5883L_CONTINOUS);

    // Set data rate
    compass.setDataRate(HMC5883L_DATARATE_30HZ);

    // Set number of samples averaged
    compass.setSamples(HMC5883L_SAMPLES_8);
}

void HMC5883L_Calibrate::calibrate()
{
    Vector mag = compass.readRaw();

    // Determine Min / Max values
    if (mag.XAxis < minX)
        minX = mag.XAxis;
    if (mag.XAxis > maxX)
        maxX = mag.XAxis;
    if (mag.YAxis < minY)
        minY = mag.YAxis;
    if (mag.YAxis > maxY)
        maxY = mag.YAxis;

    // Calculate offsets
    offX = (maxX + minX) / 2;
    offY = (maxY + minY) / 2;
}

HMC5883L_Compass::HMC5883L_Compass(float DECLINATION, int avgAmount)
{
    HMC5883L_Compass::DECLINATION = DECLINATION;
    HMC5883L_Compass::avgAmount = avgAmount;
}
void HMC5883L_Compass::setup(int offX, int offY)
{
    HMC5883L_Compass::offX = offX;
    HMC5883L_Compass::offY = offY;
    // Set calibration offset.
    compass.setOffset(offX, offY);
}

void HMC5883L_Compass::calibrate() {
    float sum = 0.00;
    for (int i = 0; i < avgAmount; i++) {
        sum += calculate();
    }

    calibration = sum/avgAmount;
}

float HMC5883L_Compass::calculate()
{
    Vector norm = compass.readNormalize();

    // Calculate heading
    float heading = atan2(norm.YAxis, norm.XAxis);

    // Set declination angle on your location and fix heading
    // You can find your declination on: http://magnetic-declination.com/
    heading += DECLINATION;

    // Correct for heading < 0deg and heading > 360deg
    if (heading < 0)
    {
        heading += 2 * PI;
    }

    if (heading > 2 * PI)
    {
        heading -= 2 * PI;
    }

    // Convert to degrees
    float headingDegrees = heading * 180 / M_PI;
    float correctedDegrees = headingDegrees - calibration;
    if (correctedDegrees < 0)
    {
        correctedDegrees += 360;
    }

    if (correctedDegrees > 360)
    {
        correctedDegrees -= 360;
    }
    // output
    return correctedDegrees - 180;
}