#ifndef BUCKY_MOTOR_CALIBRATION_H
#define BUCKY_MOTOR_CALIBRATION_H

#include <Arduino.h>
#include <tests/tests.h>
#include <tests/calibration/calibration_storage.h>

struct CalibrationResult {
    float maxTicksPerSec[3];
    float speedAt50[3];
    float linearityRatio[3];
    float headingDrift;
    float dirScale[NUM_DIRECTIONS];
    float dirOffsetDeg[NUM_DIRECTIONS];
    bool  dirValid;
    bool  valid;
};

struct MotorCalibration {
    const TestContext& ctx;
    CalibrationResult result;
    CalibrationLog log;

    void saveLog() {
        log.timestampMs = millis();
        eepromWrite(CAL_LOG_ADDR, log);
    }
};

[[noreturn]] void haltForever();

// Open-loop omni drive (no ramp) with M1/M3 negated: their direction pins are flipped on this board.
void driveDirectDegrees(Motors& md, float degrees, float scale, float rotation);
void updateLoop(Motors& md, uint32_t durationMs);
void stopAndSettle(Motors& md, uint32_t settleMs);

template <typename EachStep>
void driveHoldingHeading(const TestContext& ctx, const float degrees, const float scale,
                         const uint32_t durationMs, EachStep eachStep) {
    const uint32_t start = millis();
    while (millis() - start < durationMs) {
        ctx.compass.update();
        const float rotation = ctx.compass.computeRotation(0);
        driveDirectDegrees(ctx.motorDriver, degrees, scale, rotation);
        ctx.motorDriver.syncUpdateAllMotors();
        eachStep();
        delay(10);
    }
}

inline void driveHoldingHeading(const TestContext& ctx, const float degrees, const float scale,
                                const uint32_t durationMs) {
    driveHoldingHeading(ctx, degrees, scale, durationMs, [] {});
}

// Mean |speed| per motor over `samples` periods, ignoring the first `discard`.
void averageSpeeds(Motors& md, int samples, int discard, int periodMs, float out[3]);

bool runStaticPhase(MotorCalibration& cal);
bool runMaxSpeedPhase(MotorCalibration& cal);
void runLinearityPhase(MotorCalibration& cal);
void runForwardDrivePhase(MotorCalibration& cal);
void runDirectionPhase(MotorCalibration& cal);
void reportAndSave(MotorCalibration& cal);

#endif // BUCKY_MOTOR_CALIBRATION_H
