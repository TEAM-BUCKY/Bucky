#ifndef BUCKY_IRCONTROLLER_H
#define BUCKY_IRCONTROLLER_H

#include <Wire.h>
#include "TLA2528.h"

#define IR_SENSOR_COUNT 16
#define IR_ADC1_ADDR    0x10
#define IR_ADC2_ADDR    0x14

struct IRVector {
    float angle;      // Ball direction in degrees (0 = front, clockwise)
    float strength;   // Signal strength (0 = no ball)
};

class IRController {
private:
    TLA2528 adc1;
    TLA2528 adc2;
    bool adc1Ok = false;
    bool adc2Ok = false;
    uint16_t rawValues[IR_SENSOR_COUNT] = {};
    uint16_t baseline[IR_SENSOR_COUNT] = {};

    // Unit vectors for 16 sensors at 22.5 degree intervals
    // Sensor 0 = front (0 deg), sensor 4 = right (90 deg), etc.
    static constexpr float unitX[IR_SENSOR_COUNT] = {
         0.0000f,  0.3827f,  0.7071f,  0.9239f,
         1.0000f,  0.9239f,  0.7071f,  0.3827f,
         0.0000f, -0.3827f, -0.7071f, -0.9239f,
        -1.0000f, -0.9239f, -0.7071f, -0.3827f
    };
    static constexpr float unitY[IR_SENSOR_COUNT] = {
         1.0000f,  0.9239f,  0.7071f,  0.3827f,
         0.0000f, -0.3827f, -0.7071f, -0.9239f,
        -1.0000f, -0.9239f, -0.7071f, -0.3827f,
         0.0000f,  0.3827f,  0.7071f,  0.9239f
    };

public:
    IRController();

    bool init(TwoWire& wire);
    void calibrate(uint8_t samples = 16);
    void update();

    IRVector getVector() const;
    uint16_t getRawValue(uint8_t index) const;
    int16_t getValue(uint8_t index) const;
    uint8_t getStrongestSensor() const;
    bool isAdc1Ok() const { return adc1Ok; }
    bool isAdc2Ok() const { return adc2Ok; }
};

#endif //BUCKY_IRCONTROLLER_H
