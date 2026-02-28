#ifndef __SENSOR_CONTROL__
#define __SENSOR_CONTROL__

#include "Arduino.h"
#define IR_NUM 12

// ハードウェア依存の変数
const uint8_t   SensorPins[IR_NUM]  = {50, 53, 52, 11, 13, 12, 4, 5, 17, 41, 49, 51};
const float     unitVectorX[IR_NUM] = {0.000, 0.500, 0.866, 1.000, 0.866, 0.500, 0.000, -0.500, -0.866, -1.000, -0.866, -0.500};
const float     unitVectorY[IR_NUM] = {1.000, 0.866, 0.500, 0.000, -0.500, -0.866, -1.000, -0.866, -0.500, 0.000, 0.500, 0.866};
//const float     deltaPulseWidth     = 2.0;

typedef struct {
    int activeSensors;      // 反応したセンサの個数
    int maxPulseWidth;      // 最大のセンサ値
    int maxSensorNumber;    // 最大の値を観測したセンサの番号
} sensorInfo_t;

struct vectorXY_t {
    float x;
    float y;

    inline vectorXY_t operator+=(const vectorXY_t& other) const {
        vectorXY_t res {x+other.x,y+other.y};
        return res;
    }

    inline bool operator==(const vectorXY_t& other) const {
        return (x==other.x and y == other.y);
    }
} ;

typedef struct {
    float radius;
    float theta;
} vectorRT_t;

void setAllSensorPinsInput();
bool getSensorPin(uint8_t pin);
sensorInfo_t getAllSensorPulseWidth(int pulseWidth[IR_NUM], uint16_t timeLimit);
vectorXY_t calcVectorXYFromPulseWidth(const int *pulseWidth);
vectorRT_t calcRTfromXY(vectorXY_t *vectorXY_p);

#endif