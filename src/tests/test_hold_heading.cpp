#include "tests.h"
#include "debug.h"
#include <Arduino.h>

void testHoldHeading(MotorDriver& motorDriver, Compass& compass, I2CDMABus&) {
    DBG_PRINTLN("=== Hold Heading Test (PD) ===");
    DBG_PRINTLN("Rotate the robot by hand, it should fight back.");

    compass.reset();

    while (true) {
        compass.update();

        const float rotation = compass.computeRotation(0);

        motorDriver.driveDegrees(0, 0, rotation);

        DBG_PRINT("Offset: ");
        DBG_PRINT(compass.getOffset(), 1);
        DBG_PRINT(" | Rotation: ");
        DBG_PRINTLN(rotation, 1);

        delay(10);
    }
}
