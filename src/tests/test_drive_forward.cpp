#include "tests.h"
#include <Arduino.h>

void testDriveForward(MotorDriver& motorDriver, Compass&, IRController&, I2CManager&) {
    Serial.println("=== Drive Forward Test ===");

    while (true) {
        motorDriver.driveDegrees(0, 50, 0);
        delay(10);
    }
}
