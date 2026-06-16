#include "tests.h"
#include "debug.h"
#include <Arduino.h>

void testCompass(const TestContext& ctx) {
    DBG_PRINTLN("=== Compass + Accelerometer Test ===");
    DBG_PRINTLN("heading [deg] | offset [deg] | ax ay az [g] | |a| [g]");

    ctx.compass.reset();

    while (true) {
        ctx.compass.update();

        const float heading = ctx.compass.getHeading();
        const float offset  = ctx.compass.getOffset();

        float ax = 0, ay = 0, az = 0;
        const bool accOk = ctx.accel.read(ax, ay, az);

        DBG_PRINT("heading=");
        DBG_PRINT(heading, 1);
        DBG_PRINT(" offset=");
        DBG_PRINT(offset, 1);

        if (accOk) {
            const float mag = sqrtf(ax * ax + ay * ay + az * az);
            DBG_PRINT(" | ax=");
            DBG_PRINT(ax, 3);
            DBG_PRINT(" ay=");
            DBG_PRINT(ay, 3);
            DBG_PRINT(" az=");
            DBG_PRINT(az, 3);
            DBG_PRINT(" |a|=");
            DBG_PRINT(mag, 3);
        } else {
            DBG_PRINT(" | accel=timeout");
        }
        DBG_PRINTLN();

        delay(50);
    }
}
