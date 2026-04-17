#include "tests.h"
#include "debug.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <cmath>
#include "io/encoder/Encoder.h"

#define CALIBRATION_MAGIC 0xCA1B0001

struct StoredCalibration {
    uint32_t magic;
    float maxTicksPerSec[3];
    float linearityRatio[3];
};

bool loadCalibration(MotorDriver& md) {
    StoredCalibration cal = {};
    EEPROM.get(0, cal);

    if (cal.magic != CALIBRATION_MAGIC) return false;

    for (uint8_t i = 0; i < 3; i++) {
        if (cal.maxTicksPerSec[i] < 100.0f || cal.maxTicksPerSec[i] > 10000.0f)
            return false;
        md.setMaxTicksPerSec(i, cal.maxTicksPerSec[i]);
    }
    return true;
}

static void updateLoop(MotorDriver& md, const uint32_t durationMs) {
    const uint32_t start = millis();
    while (millis() - start < durationMs) {
        md.syncUpdateAllMotors();
        delay(5);
    }
}

static void measureSpeeds(MotorDriver& md, float outSpeeds[3]) {
    constexpr int SAMPLE_PERIOD_MS = 50;
    constexpr int TOTAL_SAMPLES = 40;
    constexpr int DISCARD = 6;

    float sums[3] = {};
    int validCount = 0;

    for (int s = 0; s < TOTAL_SAMPLES; s++) {
        delay(SAMPLE_PERIOD_MS);
        md.syncUpdateAllMotors();
        for (uint8_t i = 0; i < 3; i++)
            encoder_update_speed(i);

        if (s >= DISCARD) {
            for (uint8_t i = 0; i < 3; i++)
                sums[i] += fabsf(encoder_get_speed(i));
            validCount++;
        }
    }

    for (uint8_t i = 0; i < 3; i++)
        outSpeeds[i] = sums[i] / static_cast<float>(validCount);
}

struct CalibrationResult {
    float maxTicksPerSec[3];
    float speedAt50[3];
    float linearityRatio[3];
    float headingDrift;
    bool valid;
};

void testCalibrate(const TestContext& ctx) {
    DBG_PRINTLN("=== Motor Auto-Calibration ===");
    DBG_PRINTLN("Place robot on target surface with ~1m space.");
    DBG_PRINTLN("Starting in 3 seconds...");
    delay(3000);

    CalibrationResult result = {};

    // ========== Phase 1: Static validation ==========
    DBG_PRINTLN("\n[Phase 1/4] Static validation...");

    ctx.motorDriver.driveMotorsDirect(0, 0, 0);
    for (uint8_t i = 0; i < 3; i++)
        encoder_reset(i);

    updateLoop(ctx.motorDriver, 1000);

    bool staticOk = true;
    for (uint8_t i = 0; i < 3; i++) {
        encoder_update_speed(i);
        const float speed = fabsf(encoder_get_speed(i));
        if (speed > 5.0f) {
            DBG_PRINTLN("  FAIL: M" + String(i + 1) + " reads " +
                         String(speed, 1) + " t/s while stopped");
            staticOk = false;
        }
    }

    if (!staticOk) {
        DBG_PRINTLN("Calibration ABORTED: encoder noise too high.");
        result.valid = false;
        while (true) { delay(1000); }
    }
    DBG_PRINTLN("  OK");

    // ========== Phase 2: Max speed measurement ==========
    DBG_PRINTLN("\n[Phase 2/4] Max speed (spinning in place)...");

    // Zero PI gains so motors run at exact commanded PWM percentage (true open-loop).
    ctx.motorDriver.setPIGains(0, 0, 0);

    for (uint8_t i = 0; i < 3; i++)
        encoder_reset(i);

    ctx.motorDriver.driveMotorsDirect(100, 100, 100);
    updateLoop(ctx.motorDriver, 300);

    measureSpeeds(ctx.motorDriver, result.maxTicksPerSec);

    DBG_PRINTLN("  M1: " + String(result.maxTicksPerSec[0], 1) + " t/s   "
                "M2: " + String(result.maxTicksPerSec[1], 1) + " t/s   "
                "M3: " + String(result.maxTicksPerSec[2], 1) + " t/s");

    ctx.motorDriver.driveMotorsDirect(0, 0, 0);
    updateLoop(ctx.motorDriver, 500);

    // ========== Phase 3: Linearity at 50% ==========
    DBG_PRINTLN("\n[Phase 3/4] Linearity at 50%...");

    for (uint8_t i = 0; i < 3; i++)
        encoder_reset(i);

    ctx.motorDriver.driveMotorsDirect(50, 50, 50);
    updateLoop(ctx.motorDriver, 300);

    measureSpeeds(ctx.motorDriver, result.speedAt50);

    for (uint8_t i = 0; i < 3; i++) {
        const float expected = result.maxTicksPerSec[i] * 0.5f;
        result.linearityRatio[i] = (expected > 0) ? result.speedAt50[i] / expected : 0;
    }

    DBG_PRINTLN("  M1: " + String(result.speedAt50[0], 1) + " t/s (" +
                String(result.linearityRatio[0], 3) + ")   "
                "M2: " + String(result.speedAt50[1], 1) + " t/s (" +
                String(result.linearityRatio[1], 3) + ")   "
                "M3: " + String(result.speedAt50[2], 1) + " t/s (" +
                String(result.linearityRatio[2], 3) + ")");

    ctx.motorDriver.driveMotorsDirect(0, 0, 0);
    updateLoop(ctx.motorDriver, 500);

    // ========== Phase 4: Forward drive validation ==========
    DBG_PRINTLN("\n[Phase 4/4] Forward drive validation...");

    // Restore PI gains and apply discovered per-motor calibration for closed-loop driving
    ctx.motorDriver.setPIGains(0.5f, 0.05f, 30.0f);
    for (uint8_t i = 0; i < 3; i++)
        ctx.motorDriver.setMaxTicksPerSec(i, result.maxTicksPerSec[i]);

    ctx.compass.reset();

    constexpr uint32_t DRIVE_DURATION_MS = 3000;
    const uint32_t driveStart = millis();

    while (millis() - driveStart < DRIVE_DURATION_MS) {
        ctx.compass.update();
        const float rotation = ctx.compass.computeRotation(0);
        ctx.motorDriver.driveDegrees(0, 30, rotation);
        ctx.motorDriver.syncUpdateAllMotors();
        delay(10);
    }

    ctx.compass.update();
    result.headingDrift = ctx.compass.getOffset();

    ctx.motorDriver.driveDegrees(0, 0, 0);
    updateLoop(ctx.motorDriver, 500);

    DBG_PRINTLN("  Heading drift: " + String(result.headingDrift, 2) + " deg over 3s");

    // ========== Report & Save ==========
    result.valid = true;
    for (uint8_t i = 0; i < 3; i++) {
        if (result.maxTicksPerSec[i] < 100.0f || result.maxTicksPerSec[i] > 10000.0f)
            result.valid = false;
        if (result.linearityRatio[i] < 0.5f || result.linearityRatio[i] > 1.5f)
            result.valid = false;
    }

    DBG_PRINTLN("\n===== CALIBRATION RESULTS =====");
    DBG_PRINTLN(result.valid ? "STATUS: VALID" : "STATUS: SUSPECT - review values");
    DBG_PRINTLN("");

    for (uint8_t i = 0; i < 3; i++) {
        DBG_PRINTLN("Motor " + String(i + 1) + ":  max=" +
                     String(result.maxTicksPerSec[i], 1) + " t/s  linearity=" +
                     String(result.linearityRatio[i], 3));
    }

    if (result.valid) {
        StoredCalibration cal = {};
        cal.magic = CALIBRATION_MAGIC;
        for (uint8_t i = 0; i < 3; i++) {
            cal.maxTicksPerSec[i] = result.maxTicksPerSec[i];
            cal.linearityRatio[i] = result.linearityRatio[i];
        }
        EEPROM.put(0, cal);
        DBG_PRINTLN("\nCalibration saved to EEPROM.");
    } else {
        DBG_PRINTLN("\nNot saved — results out of range.");
    }

    DBG_PRINTLN("");
    DBG_PRINTLN("// Or apply manually:");
    for (uint8_t i = 0; i < 3; i++) {
        DBG_PRINTLN("motorDriver.setMaxTicksPerSec(" + String(i) + ", " +
                     String(result.maxTicksPerSec[i], 1) + ");");
    }
    DBG_PRINTLN("================================");

    while (true) { delay(1000); }
}
