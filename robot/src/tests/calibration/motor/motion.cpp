#include <tests/calibration/motor/motor_calibration.h>
#include <cmath>
#include <hardware/io/cordic/cordic.h>
#include <hardware/io/encoder/Encoder.h>
#include <helpers/math/Math.h>

void haltForever() {
    while (true) { delay(1000); }
}

void driveDirectDegrees(Motors& md, const float degrees, const float scale, const float rotation) {
    float sinR, cosR;
    cordic_sin_cos(Math::degreesToRadians(degrees), &sinR, &cosR);
    float raw[Motors::MOTOR_COUNT];
    Motors::wheelSpeeds(sinR, cosR, scale, rotation, raw);
    md.driveMotorsDirect(-raw[0], raw[1], -raw[2]);
}

void updateLoop(Motors& md, const uint32_t durationMs) {
    const uint32_t start = millis();
    while (millis() - start < durationMs) {
        md.syncUpdateAllMotors();
        delay(5);
    }
}

void stopAndSettle(Motors& md, const uint32_t settleMs) {
    md.driveMotorsDirect(0, 0, 0);
    updateLoop(md, settleMs);
}

void averageSpeeds(Motors& md, const int samples, const int discard, const int periodMs, float out[3]) {
    float sums[3] = {0, 0, 0};
    int count = 0;
    for (int s = 0; s < samples; s++) {
        delay(periodMs);
        md.syncUpdateAllMotors();
        for (uint8_t i = 0; i < 3; i++) encoder_update_speed(i);
        if (s >= discard) {
            for (uint8_t i = 0; i < 3; i++) sums[i] += fabsf(encoder_get_speed(i));
            count++;
        }
    }
    for (uint8_t i = 0; i < 3; i++)
        out[i] = sums[i] / static_cast<float>(count);
}
