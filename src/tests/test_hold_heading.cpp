#include "tests.h"
#include <Arduino.h>

void testHoldHeading(MotorDriver& motorDriver, Compass& compass, I2CManager&) {
    Serial.println("=== Hold Heading Test (PD) ===");
    Serial.println("Rotate the robot by hand, it should fight back.");

    compass.reset();

    while (true) {
        compass.update();

        const float rotation = compass.computeRotation(0);

        motorDriver.driveDegrees(0, 0, rotation);

        Serial.print("Offset: ");
        Serial.print(compass.getOffset(), 1);
        Serial.print(" | Rotation: ");
        Serial.println(rotation, 1);

        delay(10);
    }
}
