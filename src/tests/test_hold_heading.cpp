#include "tests.h"
#include "debug.h"
#include <Arduino.h>

void testHoldHeading(const TestContext& ctx) {
    DBG_PRINTLN("=== Hold Heading Test (PD) ===");
    DBG_PRINTLN("Rotate the robot by hand, it should fight back.");

    ctx.compass.update();
    ctx.compass.reset();
    ctx.motorDriver.setPIGains(0, 0, 0);
    ctx.compass.setPD(2.0f, 0.5f, 30.0f, 1.0f);

    while (true) {
        ctx.compass.update();

        const float rotation = ctx.compass.computeRotation(0);

        // Pure rotation: all three wheels at the same tangential sign
        // around body center. M1 and M3 have flipped direction pins on
        // this board, so to produce the same *physical* sign on all three
        // wheels, the M1 and M3 software channels are negated. M2 passes
        // through. driveRadians does the same compensation internally for
        // the driveVector path.
        ctx.motorDriver.driveMotorsDirect(-rotation, rotation, -rotation);
        ctx.motorDriver.syncUpdateAllMotors();

        DBG_PRINT("Offset: ");
        DBG_PRINT(ctx.compass.getOffset(), 1);
        DBG_PRINT(" | Rotation: ");
        DBG_PRINTLN(rotation, 1);

        delay(10);
    }
}
