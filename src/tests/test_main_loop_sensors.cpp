#include "tests.h"
#include "debug.h"
#include <Arduino.h>
#include <cmath>

// Regression test for the P0 main-loop sensor-gather bugs:
//   1. Compass heading was frozen after boot because main never called
//      processRead() between isReadComplete() and getOffset().
//   2. Sonar was using the blocking sonar.read() + an extra startRead(),
//      blocking the loop up to 20 ms per cycle.
//
// Procedure:
//   * Rotate the robot by hand (motors are not driven).
//   * Heading and offset must change continuously.
//   * loop_ms should stay well under 5 ms on average (old code was 20 ms+).
//   * Sonar distances should update every ~20-40 ms.
//
// PASS: heading changes when rotated, loop_ms peak < 10 ms in steady state.
// FAIL: heading stuck at boot value, or loop_ms pegged at 20 ms.
void testMainLoopSensors(const TestContext& ctx)
{
    DBG_PRINTLN("=== Main-Loop Sensor Gather Test ===");
    DBG_PRINTLN("Rotate the robot by hand; heading must update continuously.");

    // Mirror the exact production kickoff from setupEnvironment().
    ctx.compass.startRead();
    ctx.sonar.startRead();

    uint32_t lastTick = millis();
    uint32_t report = lastTick;
    uint32_t loopMaxUs = 0;
    uint32_t loopMinUs = 0xFFFFFFFFu;
    uint32_t loopSumUs = 0;
    uint32_t loopCount = 0;

    float lastHeading = ctx.compass.getHeading();
    bool headingChangedEver = false;
    float sonarM[SONAR_COUNT] = {0, 0, 0, 0};
    bool compassUpdated = false;
    bool sonarUpdated = false;

    while (true) {
        const uint32_t t0 = micros();
        const uint32_t now = millis();
        lastTick = now;

        if (ctx.compass.isReadComplete()) {
            ctx.compass.processRead();
            const float h = ctx.compass.getHeading();
            if (fabsf(h - lastHeading) > 0.5f) headingChangedEver = true;
            lastHeading = h;
            compassUpdated = true;
            ctx.compass.startRead();
        }

        if (ctx.sonar.isReadComplete()) {
            const SonarReading r = ctx.sonar.processRead();
            for (int i = 0; i < SONAR_COUNT; ++i)
                sonarM[i] = r.valid[i] ? r.distance[i] * 0.01f : -1.0f;
            sonarUpdated = true;
            ctx.sonar.startRead();
        }

        const uint32_t dtUs = micros() - t0;
        if (dtUs > loopMaxUs) loopMaxUs = dtUs;
        if (dtUs < loopMinUs) loopMinUs = dtUs;
        loopSumUs += dtUs;
        loopCount++;

        if (now - report >= 500) {
            const uint32_t avgUs = loopSumUs / (loopCount ? loopCount : 1);
            DBG_PRINT("heading=");
            DBG_PRINT(ctx.compass.getHeading(), 1);
            DBG_PRINT(" offset=");
            DBG_PRINT(ctx.compass.getOffset(), 1);
            DBG_PRINT(" | loop us min=");
            DBG_PRINT(loopMinUs);
            DBG_PRINT(" avg=");
            DBG_PRINT(avgUs);
            DBG_PRINT(" max=");
            DBG_PRINT(loopMaxUs);
            DBG_PRINT(" | sonar f=");
            DBG_PRINT(sonarM[0], 2);
            DBG_PRINT(" r=");
            DBG_PRINT(sonarM[1], 2);
            DBG_PRINT(" b=");
            DBG_PRINT(sonarM[2], 2);
            DBG_PRINT(" l=");
            DBG_PRINT(sonarM[3], 2);
            DBG_PRINT(" | ");
            DBG_PRINT(compassUpdated ? "C" : "-");
            DBG_PRINT(sonarUpdated ? "S" : "-");
            DBG_PRINT(headingChangedEver ? " moved" : " STILL");
            DBG_PRINTLN();

            report = now;
            loopMaxUs = 0;
            loopMinUs = 0xFFFFFFFFu;
            loopSumUs = 0;
            loopCount = 0;
            compassUpdated = false;
            sonarUpdated = false;
        }
    }
}
