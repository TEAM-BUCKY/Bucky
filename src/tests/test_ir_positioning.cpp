#include "tests.h"
#include "debug.h"
#include <Arduino.h>
#include <cmath>
#include "../sensors/IRSensor.h"
#include "../helpers/Math.h"

void testIRPositioning(const TestContext& ctx) {
    (void)ctx;
    DBG_PRINTLN("=== IR Ball Positioning Test (front arc: S1-3, S11-12) ===");

    constexpr uint8_t board = 2;
    constexpr uint8_t FRONT_ARC[] = {0, 1, 2, 10, 11};  // 1-indexed S1-3, S11-12
    constexpr uint8_t FRONT_ARC_N = sizeof(FRONT_ARC) / sizeof(FRONT_ARC[0]);
    constexpr uint32_t SENSOR_COUNT = 12;
    constexpr float ANGLE_PER_SENSOR_DEG = 360.0f / SENSOR_COUNT;

    constexpr float BASELINE = 100.0f;
    // Low threshold ratio lets neighbor-sensor crosstalk (which carries real
    // angular info) contribute to the weighted centroid. With 0.30 the bearing
    // quantizes to exact sensor angles (30/60/-30/-60) because the neighbor
    // crosstalk gets filtered out; 0.08 keeps the centroid responsive between
    // sensors while still rejecting the idle noise floor.
    constexpr float THRESHOLD_RATIO = 0.08f;
    constexpr float MIN_FRONT_PEAK = 100.0f;

    // Rear-arc (S4-S10) acts as an "indicator": these sensors are less reliable
    // at range, but a strong reading is a clear signal the ball is NOT in front.
    constexpr float REAR_INDICATE_MIN = 300.0f;

    constexpr float RANGE_K = 700.0f;

    auto isFront = [&](uint8_t ch) {
        for (uint8_t i = 0; i < FRONT_ARC_N; ++i) if (FRONT_ARC[i] == ch) return true;
        return false;
    };

    // setupEnvironment has already called ir_calibrate_channels. Pick up the
    // detected mux-rotation offset so physical sensor indices map to the
    // right buffer slots.
    const uint8_t bufOffset = ir_get_channel_offset(board);
    DBG_PRINT("Mux channel offset: "); DBG_PRINT(bufOffset);
    DBG_PRINT("  (physical S1 at buf["); DBG_PRINT(bufOffset); DBG_PRINTLN("])");

    auto physToBuf = [bufOffset](uint32_t p) -> uint32_t {
        return (p + bufOffset) % IR_MUX_CHANNELS;
    };

    uint32_t lastIrSeq = ir_get_frame_sequence(board);
    uint32_t lastPrintMs = 0;
    constexpr uint32_t PRINT_INTERVAL_MS = 100;

    while (true) {
        if (!ir_has_new_frame(board, lastIrSeq)) { delay(1); continue; }
        lastIrSeq = ir_get_frame_sequence(board);

        const uint16_t* raw = ir_get_buffer(board);

        uint16_t rawMax[SENSOR_COUNT];
        float frontAmp[SENSOR_COUNT];  // baseline-subtracted, non-front = 0
        float rearAmp[SENSOR_COUNT];   // baseline-subtracted, front = 0
        for (uint32_t ch = 0; ch < SENSOR_COUNT; ++ch) {
            const uint32_t bufIdx = physToBuf(ch);
            uint16_t m = 0;
            for (uint32_t sweep = 0; sweep < IR_SWEEPS_PER_CYCLE; ++sweep) {
                uint16_t v = raw[sweep * IR_MUX_CHANNELS + bufIdx];
                if (v > m) m = v;
            }
            rawMax[ch] = m;
            float a = static_cast<float>(m) - BASELINE;
            if (a < 0.0f) a = 0.0f;
            if (isFront(static_cast<uint8_t>(ch))) {
                frontAmp[ch] = a;
                rearAmp[ch] = 0.0f;
            } else {
                frontAmp[ch] = 0.0f;
                rearAmp[ch] = a;
            }
        }

        float frontPeak = 0.0f;
        uint8_t frontPeakSensor = 0;
        for (uint8_t i = 0; i < FRONT_ARC_N; ++i) {
            const uint8_t ch = FRONT_ARC[i];
            if (frontAmp[ch] > frontPeak) { frontPeak = frontAmp[ch]; frontPeakSensor = ch; }
        }

        float rearPeak = 0.0f;
        uint8_t rearPeakSensor = 0;
        for (uint32_t ch = 0; ch < SENSOR_COUNT; ++ch) {
            if (rearAmp[ch] > rearPeak) { rearPeak = rearAmp[ch]; rearPeakSensor = static_cast<uint8_t>(ch); }
        }

        const uint32_t now = HAL_GetTick();
        if (now - lastPrintMs < PRINT_INTERVAL_MS) continue;
        lastPrintMs = now;

        // Weighted angular centroid over a given amp array. Returns true if
        // any sensor contributed; bearingDegOut then holds the centroid angle.
        auto bearingFromAmps = [&](const float* amps, float peak,
                                   float& bearingDegOut, uint8_t& contribsOut) {
            const float threshold = peak * THRESHOLD_RATIO;
            float sx = 0.0f, sy = 0.0f, totalW = 0.0f;
            uint8_t n = 0;
            for (uint32_t ch = 0; ch < SENSOR_COUNT; ++ch) {
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
            if (totalW <= 0.0f) { bearingDegOut = 0.0f; return false; }
            bearingDegOut = Math::radiansToDegrees(atan2f(sy, sx));
            return true;
        };

        if (frontPeak >= MIN_FRONT_PEAK) {
            float bearingDeg = 0.0f;
            uint8_t contribs = 0;
            bearingFromAmps(frontAmp, frontPeak, bearingDeg, contribs);
            const float rangeCm = RANGE_K / sqrtf(frontPeak);

            DBG_PRINT("BALL  bearing="); DBG_PRINT(bearingDeg);
            DBG_PRINT(" deg  ~range=");  DBG_PRINT(rangeCm);
            DBG_PRINT(" cm  peak=S");    DBG_PRINT(frontPeakSensor + 1);
            DBG_PRINT("  peakAmp=");     DBG_PRINT(frontPeak);
            DBG_PRINT("  contribs=");    DBG_PRINTLN(contribs);
        } else if (rearPeak >= REAR_INDICATE_MIN) {
            // Front arc is quiet but a rear sensor sees the ball. Rear readings
            // are less reliable (narrow FOV, needs the ball close) but still
            // give a useful direction — report it as a separate bearing so the
            // caller knows which way to rotate.
            float bearingDeg = 0.0f;
            uint8_t contribs = 0;
            bearingFromAmps(rearAmp, rearPeak, bearingDeg, contribs);
            const float rangeCm = RANGE_K / sqrtf(rearPeak);

            DBG_PRINT("BALL(rear)  bearing="); DBG_PRINT(bearingDeg);
            DBG_PRINT(" deg  ~range=");        DBG_PRINT(rangeCm);
            DBG_PRINT(" cm  peak=S");          DBG_PRINT(rearPeakSensor + 1);
            DBG_PRINT("  peakAmp=");           DBG_PRINT(rearPeak);
            DBG_PRINT("  contribs=");          DBG_PRINTLN(contribs);
        } else {
            DBG_PRINTLN("no ball");
        }

        // Also flag when the rear is lit during a front-arc detection — means
        // the front-arc bearing may be bleed-through from a ball behind us.
        if (frontPeak >= MIN_FRONT_PEAK && rearPeak >= REAR_INDICATE_MIN) {
            DBG_PRINT("  [REAR] also firing on S"); DBG_PRINT(rearPeakSensor + 1);
            DBG_PRINT(" amp=");                     DBG_PRINT(rearPeak);
            DBG_PRINTLN(" (check ball position)");
        }

        DBG_PRINT("  amp:");
        for (uint32_t ch = 0; ch < SENSOR_COUNT; ++ch) {
            DBG_PRINT(isFront(static_cast<uint8_t>(ch)) ? "  *S" : "   S");
            DBG_PRINT(ch + 1);
            DBG_PRINT("=");
            DBG_PRINT(rawMax[ch]);
        }
        DBG_PRINTLN();
    }
}
