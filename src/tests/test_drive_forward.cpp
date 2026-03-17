#include "tests.h"
#include "debug.h"
#include <Arduino.h>

void testDriveForward(MotorDriver& motorDriver, Compass&, I2CDMABus&) {
    DBG_PRINTLN("=== Drive Forward Test ===");

    while (true) {
        motorDriver.driveDegrees(0, 50, 0);
        delay(10);
    }
}
