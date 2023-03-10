#ifndef TSSP_H_
#define TSSP_H_

#include "Arduino.h"
#define IR_NUM 14

const uint8_t SensorPins[IR_NUM] = {17, 16, 14, 40, 38, 39, 41, 15, 18, 24, 26, 27, 25, 19};
const float deltaPulseWidth = 2.0;

typedef struct {
    int activeSensors;       // Number of sensors that responded
    int maxPulseWidth;       // Maximum sensor value
    int maxSensorNumber;     // The number of the sensor that observed the maximum value
} sensorInfo_t;

void setAllSensorPinsInput(void);
bool  getSensorPin ( uint8_t pin);
sensorInfo_t getAllSensorPulseWidth ( float pulseWidth [IR_NUM], uint16_t timeLimit);

#endif