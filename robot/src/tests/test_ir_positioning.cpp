#include "tests.h"
#include "debug.h"
#include <Arduino.h>
#include <cmath>
#include "helpers/Math.h"
#include "hardware/sensors/IRRing.h"

void testIRPositioning(const TestContext& ctx)
{
    DBG_PRINTLN("=== IR Ball Positioning Test ===");

    if (ctx.irPort == nullptr)
    {
        DBG_PRINTLN("No G port is configured for the IR ring.");
        while (true)
        {
        }
    }
    IRRing ring(*ctx.irPort);
    const uint8_t sensorCount = ring.config().sensors;

    uint32_t lastPrintMs = 0;
    constexpr uint32_t PRINT_INTERVAL_MS = 100;

    while (true)
    {
        if (!ring.update())
        {
            delay(1);
            continue;
        }

        const uint32_t now = millis();
        if (now - lastPrintMs < PRINT_INTERVAL_MS) continue;
        lastPrintMs = now;

        const BallReading& ball = ring.ball();
        if (ball.seen)
        {
            DBG_PRINT("BALL  bearing=");
            DBG_PRINT(Math::radiansToDegrees(ball.angle));
            DBG_PRINT(" deg  ~range=");
            DBG_PRINT(ball.distanceCm);
            DBG_PRINT(" cm  peak=S");
            DBG_PRINT(ball.peakSensor + 1);
            DBG_PRINT("  peakAmp=");
            DBG_PRINT(ball.peakAmp);
            DBG_PRINT("  contribs=");
            DBG_PRINTLN(ball.contributors);
        }
        else
        {
            DBG_PRINTLN("no ball");
        }

        const uint16_t* raw = ring.raw();
        DBG_PRINT("  amp:");
        for (uint32_t ch = 0; ch < sensorCount; ++ch)
        {
            DBG_PRINT("   S");
            DBG_PRINT(ch + 1);
            DBG_PRINT("=");
            DBG_PRINT(raw[ch]);
        }
        DBG_PRINTLN();
    }
}
