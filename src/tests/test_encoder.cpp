#include "tests.h"
#include "debug.h"
#include <Arduino.h>
#include "io/encoder/Encoder.h"

void testEncoder(const TestContext& ctx) {
    DBG_PRINTLN("=== Encoder Test (All Motors) ===");

    ctx.motorDriver.driveMotorsDirect(100, 100, 100);

    uint32_t lastPrint = millis();

    while (true) {
        ctx.motorDriver.syncUpdateAllMotors();

        for (uint8_t i = 0; i < 3; i++)
            encoder_update_speed(i);

        if (millis() - lastPrint >= 500) {
            const int32_t t0 = encoder_get_ticks(0);
            const int32_t t1 = encoder_get_ticks(1);
            const int32_t t2 = encoder_get_ticks(2);
            const float s0 = encoder_get_speed(0);
            const float s1 = encoder_get_speed(1);
            const float s2 = encoder_get_speed(2);

            DBG_PRINTLN(
                "M1: " + String(t0) + "t " + String(s0, 1) + "t/s  "
                "M2: " + String(t1) + "t " + String(s1, 1) + "t/s  "
                "M3: " + String(t2) + "t " + String(s2, 1) + "t/s");

            lastPrint = millis();
        }
    }
}
