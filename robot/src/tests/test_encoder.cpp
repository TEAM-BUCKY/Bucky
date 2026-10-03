#include "tests.h"
#include "debug.h"
#include <Arduino.h>
#include "hardware/io/encoder/Encoder.h"

void testEncoder(const TestContext& ctx) {
    DBG_PRINTLN("=== Encoder Test (All Motors) ===");

    ctx.motorDriver.driveMotorsDirect(100, 100, 100);

    uint32_t lastPrint = millis();

    while (true) {
        ctx.motorDriver.syncUpdateAllMotors();

        for (uint8_t i = 0; i < 3; i++)
            encoder_update_speed(i);
        MotorEncoderValue encoderValues[3];
        ctx.motorDriver.getEncoderValues(encoderValues);

        if (millis() - lastPrint >= 500) {
            DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_ENCODER,
                "M1: " + String(encoderValues[0].ticks) + "t " + String(encoderValues[0].speed, 1) + "t/s  "
                "M2: " + String(encoderValues[1].ticks) + "t " + String(encoderValues[1].speed, 1) + "t/s  "
                "M3: " + String(encoderValues[2].ticks) + "t " + String(encoderValues[2].speed, 1) + "t/s");

            lastPrint = millis();
        }
    }
}
