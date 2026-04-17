#include "tests.h"
#include "debug.h"
#include <Arduino.h>

void testHoldHeading(const TestContext& ctx) {
    DBG_PRINTLN("=== Hold Heading Test (PD) ===");
    DBG_PRINTLN("Rotate the robot by hand, it should fight back.");

    ctx.compass.reset();
    ctx.motorDriver.setPIGains(0, 0, 0);
    ctx.compass.setPD(2.0f, 0.5f, 30.0f, 1.0f);

    while (true) {
        ctx.compass.update();

        const float rotation = ctx.compass.computeRotation(0);

        // Pure rotation: apply correction equally to all motors.
        // Sign negated to match physical motor layout (M1/M3 pin-swapped).
        ctx.motorDriver.driveMotorsDirect(-rotation, -rotation, -rotation);
        ctx.motorDriver.syncUpdateAllMotors();

        DBG_PRINT("Offset: ");
        DBG_PRINT(ctx.compass.getOffset(), 1);
        DBG_PRINT(" | Rotation: ");
        DBG_PRINTLN(rotation, 1);

        delay(10);
    }
}
