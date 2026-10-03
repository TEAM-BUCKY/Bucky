#include "tests.h"
#include "debug.h"
#include <Arduino.h>
#include <cmath>
#include "helpers/Math.h"

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
    const GPort& ir = *ctx.irPort;
    constexpr uint32_t SENSOR_COUNT = 12;
    constexpr float ANGLE_PER_SENSOR_DEG = 360.0f / SENSOR_COUNT;

    constexpr float BASELINE = 100.0f;
    // Low threshold ratio lets neighbor-sensor crosstalk (which carries real
    // angular info) contribute to the weighted centroid. With 0.30 the bearing
    // quantizes to exact sensor angles (30/60/-30/-60) because the neighbor
    // crosstalk gets filtered out; 0.08 keeps the centroid responsive between
    // sensors while still rejecting the idle noise floor.
    constexpr float THRESHOLD_RATIO = 0.08f;
    constexpr float MIN_PEAK = 100.0f;

    constexpr float RANGE_K = 700.0f;

    uint16_t frame[GPort::SENSORS];
    uint32_t lastIrSeq = ir.frameSequence();
    uint32_t lastPrintMs = 0;
    constexpr uint32_t PRINT_INTERVAL_MS = 100;

    while (true)
    {
        if (!ir.hasNewFrame(lastIrSeq) || !ir.readIR(frame))
        {
            delay(1);
            continue;
        }
        lastIrSeq = ir.frameSequence();

        uint16_t rawMax[SENSOR_COUNT];
        float amp[SENSOR_COUNT]; // baseline-subtracted amplitude
        for (uint32_t ch = 0; ch < SENSOR_COUNT; ++ch)
        {
            const uint16_t m = frame[ch];
            rawMax[ch] = m;
            float a = static_cast<float>(m) - BASELINE;
            if (a < 0.0f) a = 0.0f;
            amp[ch] = a;
        }

        float peak = 0.0f;
        uint8_t peakSensor = 0;
        for (uint32_t ch = 0; ch < SENSOR_COUNT; ++ch)
        {
            if (amp[ch] > peak)
            {
                peak = amp[ch];
                peakSensor = static_cast<uint8_t>(ch);
            }
        }

        const uint32_t now = millis();
        if (now - lastPrintMs < PRINT_INTERVAL_MS) continue;
        lastPrintMs = now;

        // Weighted angular centroid over all sensors. Returns true if
        // any sensor contributed; bearingDegOut then holds the centroid angle.
        auto bearingFromAmps = [&](const float* amps, float peak,
                                   float& bearingDegOut, uint8_t& contribsOut)
        {
            const float threshold = peak * THRESHOLD_RATIO;
            float sx = 0.0f, sy = 0.0f, totalW = 0.0f;
            uint8_t n = 0;
            for (uint32_t ch = 0; ch < SENSOR_COUNT; ++ch)
            {
                if (amps[ch] <= threshold) continue;
                const float v = amps[ch] - threshold;
                const float w = v * v;
                const float angleRad = Math::degreesToRadians(
                    ANGLE_PER_SENSOR_DEG * static_cast<float>(ch));
                sx += w * cosf(angleRad);
                sy += w * sinf(angleRad);
                totalW += w;
                ++n;
            }
            contribsOut = n;
            if (totalW <= 0.0f)
            {
                bearingDegOut = 0.0f;
                return false;
            }
            bearingDegOut = Math::radiansToDegrees(atan2f(sy, sx));
            return true;
        };

        if (peak >= MIN_PEAK)
        {
            float bearingDeg = 0.0f;
            uint8_t contribs = 0;
            bearingFromAmps(amp, peak, bearingDeg, contribs);
            const float rangeCm = RANGE_K / sqrtf(peak);

            DBG_PRINT("BALL  bearing=");
            DBG_PRINT(bearingDeg);
            DBG_PRINT(" deg  ~range=");
            DBG_PRINT(rangeCm);
            DBG_PRINT(" cm  peak=S");
            DBG_PRINT(peakSensor + 1);
            DBG_PRINT("  peakAmp=");
            DBG_PRINT(peak);
            DBG_PRINT("  contribs=");
            DBG_PRINTLN(contribs);
        }
        else
        {
            DBG_PRINTLN("no ball");
        }

        DBG_PRINT("  amp:");
        for (uint32_t ch = 0; ch < SENSOR_COUNT; ++ch)
        {
            DBG_PRINT("   S");
            DBG_PRINT(ch + 1);
            DBG_PRINT("=");
            DBG_PRINT(rawMax[ch]);
        }
        DBG_PRINTLN();
    }
}
