#include "tests.h"
#include "debug.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <limits.h>
#include <cmath>

// Same layout as test_calibrate.cpp — kept local to avoid leaking the struct into a
// header. Any change here must also be mirrored there.
#define CALIBRATION_MAGIC 0xCA1B0003
constexpr uint8_t NUM_DIRECTIONS = 12;

struct StoredCalibration {
    uint32_t magic;
    float maxTicksPerSec[3];
    float linearityRatio[3];
    float dirScale[NUM_DIRECTIONS];
    float dirOffsetDeg[NUM_DIRECTIONS];
    bool  dirValid;
    float magOffset[3];
    float magScale[3];
    bool  magValid;
};

void testCompassCalibrate(const TestContext& ctx) {
    DBG_PRINTLN("=== LIS2MDL Compass Calibration ===");
    DBG_PRINTLN("Rotate the robot slowly through 2+ full turns on a flat surface.");
    DBG_PRINTLN("Tilt it through pitch and roll at least once so Z gets coverage.");
    DBG_PRINTLN("Stay clear of laptops, speakers, and large steel objects.");
    DBG_PRINTLN("Sampling for 60 s. Starting in 3...");
    delay(3000);

    int16_t magMin[3] = { INT16_MAX, INT16_MAX, INT16_MAX };
    int16_t magMax[3] = { INT16_MIN, INT16_MIN, INT16_MIN };

    constexpr uint32_t CAPTURE_MS = 60000;
    constexpr uint32_t SAMPLE_PERIOD_MS = 20;    // ~50 Hz
    constexpr uint32_t REPORT_PERIOD_MS = 1000;

    const uint32_t start = millis();
    uint32_t lastReport = start;

    while (millis() - start < CAPTURE_MS) {
        ctx.compass.update();
        const int16_t xyz[3] = {
            ctx.compass.getRawX(),
            ctx.compass.getRawY(),
            ctx.compass.getRawZ(),
        };
        for (uint8_t i = 0; i < 3; i++) {
            if (xyz[i] < magMin[i]) magMin[i] = xyz[i];
            if (xyz[i] > magMax[i]) magMax[i] = xyz[i];
        }

        if (millis() - lastReport >= REPORT_PERIOD_MS) {
            lastReport = millis();
            const uint32_t elapsed = (millis() - start) / 1000;
            DBG_PRINT("t=");
            DBG_PRINT(elapsed);
            DBG_PRINT("s  min=(");
            DBG_PRINT(magMin[0]); DBG_PRINT(", ");
            DBG_PRINT(magMin[1]); DBG_PRINT(", ");
            DBG_PRINT(magMin[2]);
            DBG_PRINT(")  max=(");
            DBG_PRINT(magMax[0]); DBG_PRINT(", ");
            DBG_PRINT(magMax[1]); DBG_PRINT(", ");
            DBG_PRINT(magMax[2]);
            DBG_PRINT(")  range=(");
            DBG_PRINT(magMax[0] - magMin[0]); DBG_PRINT(", ");
            DBG_PRINT(magMax[1] - magMin[1]); DBG_PRINT(", ");
            DBG_PRINT(magMax[2] - magMin[2]);
            DBG_PRINTLN(")");
        }

        delay(SAMPLE_PERIOD_MS);
    }

    // Hard-iron: midpoint of min/max per axis.
    // Soft-iron (diagonal): scale each axis to the largest range so the sweep becomes
    // a sphere (or at least a circle in X/Y) rather than an ellipse.
    float range[3], offset[3];
    int32_t maxRange = 0;
    for (uint8_t i = 0; i < 3; i++) {
        const int32_t r = static_cast<int32_t>(magMax[i]) - static_cast<int32_t>(magMin[i]);
        range[i]  = static_cast<float>(r);
        offset[i] = 0.5f * (static_cast<float>(magMax[i]) + static_cast<float>(magMin[i]));
        if (r > maxRange) maxRange = r;
    }

    float scale[3];
    bool magValid = true;
    for (uint8_t i = 0; i < 3; i++) {
        if (range[i] < 100.0f) magValid = false;   // axis didn't rotate enough
        scale[i] = (range[i] > 1.0f) ? (static_cast<float>(maxRange) / range[i]) : 1.0f;
        if (scale[i] < 0.3f || scale[i] > 3.0f) magValid = false;
    }

    DBG_PRINTLN("\n===== COMPASS CALIBRATION RESULTS =====");
    DBG_PRINTLN(magValid ? "STATUS: VALID" : "STATUS: SUSPECT - not saved");
    for (uint8_t i = 0; i < 3; i++) {
        const char axis = static_cast<char>('X' + i);
        DBG_PRINT("  ");
        DBG_PRINT(axis);
        DBG_PRINT(": min=");
        DBG_PRINT(magMin[i]);
        DBG_PRINT(" max=");
        DBG_PRINT(magMax[i]);
        DBG_PRINT(" range=");
        DBG_PRINT(range[i], 1);
        DBG_PRINT(" offset=");
        DBG_PRINT(offset[i], 2);
        DBG_PRINT(" scale=");
        DBG_PRINTLN(scale[i], 4);
    }

    if (magValid) {
        // Read-modify-write so motor / direction calibration is preserved.
        StoredCalibration cal = {};
        EEPROM.get(0, cal);
        const bool keepMotor = (cal.magic == CALIBRATION_MAGIC);
        if (!keepMotor) {
            // Start from a clean slate: motor cal unknown, direction cal disabled.
            cal = {};
            for (uint8_t i = 0; i < NUM_DIRECTIONS; i++) {
                cal.dirScale[i]     = 1.0f;
                cal.dirOffsetDeg[i] = 0.0f;
            }
            cal.dirValid = false;
        }
        cal.magic = CALIBRATION_MAGIC;
        for (uint8_t i = 0; i < 3; i++) {
            cal.magOffset[i] = offset[i];
            cal.magScale[i]  = scale[i];
        }
        cal.magValid = true;
        EEPROM.put(0, cal);
        DBG_PRINTLN(keepMotor
            ? "Compass calibration saved (motor cal preserved)."
            : "Compass calibration saved (no prior motor cal found).");

        ctx.compass.setCalibration(offset[0], offset[1], offset[2],
                                   scale[0],  scale[1],  scale[2]);
        ctx.compass.reset();
    } else {
        DBG_PRINTLN("Not saved. Rotate more fully through all axes and retry.");
    }

    DBG_PRINTLN("\nLive corrected heading (Ctrl-C / reset when done):");
    DBG_PRINTLN("ok heading raw=(x,y,z)");
    while (true) {
        const bool ok = ctx.compass.update();
        DBG_PRINT(ok ? "OK  " : "ERR ");
        DBG_PRINT("heading=");
        DBG_PRINT(ctx.compass.getHeading(), 1);
        DBG_PRINT("  raw=(");
        DBG_PRINT(ctx.compass.getRawX()); DBG_PRINT(", ");
        DBG_PRINT(ctx.compass.getRawY()); DBG_PRINT(", ");
        DBG_PRINT(ctx.compass.getRawZ()); DBG_PRINTLN(")");
        delay(100);
    }
}
