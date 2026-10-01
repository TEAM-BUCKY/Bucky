#ifndef BUCKY_CALIBRATION_STORAGE_H
#define BUCKY_CALIBRATION_STORAGE_H

#include <stdint.h>

// EEPROM layout shared by testCalibrate, testCompassCalibrate and loadCalibration().

#define CALIBRATION_MAGIC 0xCA1B0003

constexpr uint8_t NUM_DIRECTIONS = 12;

struct StoredCalibration {
    uint32_t magic;
    float maxTicksPerSec[3];
    float linearityRatio[3];
    float dirScale[NUM_DIRECTIONS];
    float dirOffsetDeg[NUM_DIRECTIONS];
    bool  dirValid;
    float magOffset[3];   // LIS2MDL hard-iron X/Y/Z
    float magScale[3];    // LIS2MDL soft-iron diagonal X/Y/Z
    bool  magValid;
};

#endif // BUCKY_CALIBRATION_STORAGE_H
