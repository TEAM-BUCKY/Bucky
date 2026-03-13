#include "tests.h"
#include <Arduino.h>
#include <cmath>

void testHoldHeading(MotorDriver& motorDriver, Compass& compass, I2CManager&) {
    constexpr float KP = 0.4f;
    constexpr float KD = 0.3f;
    constexpr float MAX_ROT = 25.0f;
    constexpr float DEADZONE = 3.0f;

    Serial.println("=== Hold Heading Test (PD) ===");
    Serial.println("Rotate the robot by hand, it should fight back.");

    compass.reset();

    float lastOffset = 0;
    unsigned long lastTime = millis();

    while (true) {
        compass.update();

        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;

        const float offset = compass.getOffset();
        float rotation = 0;

        if (fabs(offset) > DEADZONE) {
            float derivative = (dt > 0) ? (offset - lastOffset) / dt : 0;
            rotation = constrain(-offset * KP - derivative * KD, -MAX_ROT, MAX_ROT);
        }

        lastOffset = offset;

        motorDriver.driveDegrees(0, 0, rotation);

        Serial.print("Offset: ");
        Serial.print(offset, 1);
        Serial.print(" | Rotation: ");
        Serial.println(rotation, 1);

        delay(10);
    }
}
