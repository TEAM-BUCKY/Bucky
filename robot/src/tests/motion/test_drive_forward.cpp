#include <tests/tests.h>
#include <debug.h>
#include <Arduino.h>

void testDriveForward(const TestContext& ctx) {
    DBG_PRINTLN("=== Drive Forward Test ===");

    // Once: drive() restarts the ramp only when the command changes.
    ctx.motorDriver.driveDegrees(0, 0.5f, 0);

    while (true) {
        ctx.motorDriver.syncUpdateAllMotors();   // advances the ramp, so call it often
        delay(5);
    }
}
