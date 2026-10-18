#ifndef BUCKY_CALIBRATION_STORAGE_H
#define BUCKY_CALIBRATION_STORAGE_H

#include <stddef.h>
#include <stdint.h>
#include <utility/stm32_eeprom.h>

#define CALIBRATION_MAGIC 0xCA1B0003
#define CAL_LOG_MAGIC     0xCA106001
#define CAL_LOG_ADDR      512

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

// CalibrationLog::lastDirection while phase 5 is outside the direction loop.
constexpr uint8_t CAL_DIR_NONE       = 0xFF;
constexpr uint8_t CAL_DIR_PREFLIGHT  = 0xFE;
constexpr uint8_t CAL_DIR_LOOP_START = 0xFD;
constexpr uint8_t CAL_DIR_ACCEL_BIAS = 0xFC;

constexpr uint8_t DIR_FLAG_SAT_OK    = 0x01;
constexpr uint8_t DIR_FLAG_SCALE_OK  = 0x02;
constexpr uint8_t DIR_FLAG_OFFSET_OK = 0x04;
constexpr uint8_t DIR_FLAG_MOTION_OK = 0x08;
constexpr uint8_t DIR_FLAG_AGREE_OK  = 0x10;

// Rewritten after every step so a crash leaves a trace of how far calibration got.
struct CalibrationLog {
    uint32_t magic;
    uint32_t timestampMs;

    float staticNoise[3];
    bool  phase1_staticOk;

    float maxTicksPerSec[3];
    float speedAt50[3];
    float linearityRatio[3];

    float phase4_headingDrift;

    float preFlightMotorSpeed[3];
    bool  phase5_signsOk;
    float accelBias[3];
    bool  phase5_accelOk;

    float dirMotorPct[NUM_DIRECTIONS][3];
    float dirActualDeg[NUM_DIRECTIONS];
    float dirOffsetDeg[NUM_DIRECTIONS];
    float dirSpeedPct[NUM_DIRECTIONS];
    float dirScale[NUM_DIRECTIONS];
    uint8_t dirFlags[NUM_DIRECTIONS];

    float dirAccelRawDeg[NUM_DIRECTIONS];
    float dirAccelVelMps[NUM_DIRECTIONS];
    float dirAccelOffsetDeg[NUM_DIRECTIONS];
    float accelFrameDeg;
    bool  phase5_accelFrameOk;

    int32_t dirRawTicks[NUM_DIRECTIONS][3];

    uint8_t lastPhase;
    uint8_t lastDirection;
    bool    completed;

    bool motorValid;
    bool dirValid;
};

// One buffered flash write instead of EEPROM.put's write per byte.
template <typename T>
void eepromWrite(const uint32_t addr, const T& value) {
    eeprom_buffer_fill();
    const auto* bytes = reinterpret_cast<const uint8_t*>(&value);
    for (size_t i = 0; i < sizeof(T); i++)
        eeprom_buffered_write_byte(addr + i, bytes[i]);
    eeprom_buffer_flush();
}

#endif // BUCKY_CALIBRATION_STORAGE_H
