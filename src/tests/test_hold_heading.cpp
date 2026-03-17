#include "tests.h"
#include "debug.h"
#include <Arduino.h>

void testHoldHeading(const TestContext& ctx) {
    DBG_PRINTLN("=== Hold Heading Test (PD) ===");
    DBG_PRINTLN("Rotate the robot by hand, it should fight back.");

    ctx.compass.reset();

    while (true) {
        ctx.compass.update();

        const float rotation = ctx.compass.computeRotation(0);

        ctx.motorDriver.driveDegrees(0, 0, rotation);

        DBG_PRINT("Offset: ");
        DBG_PRINT(ctx.compass.getOffset(), 1);
        DBG_PRINT(" | Rotation: ");
        DBG_PRINTLN(rotation, 1);

        delay(10);
    }
}
