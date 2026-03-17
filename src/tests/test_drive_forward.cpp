#include "tests.h"
#include "debug.h"
#include <Arduino.h>

void testDriveForward(const TestContext& ctx) {
    DBG_PRINTLN("=== Drive Forward Test ===");

    while (true) {
        ctx.motorDriver.driveDegrees(0, 50, 0);
        delay(10);
    }
}
