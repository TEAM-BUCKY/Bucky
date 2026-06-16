#include "tests.h"
#include "debug.h"
#include <Arduino.h>

void testSonar(const TestContext& ctx) {
    DBG_PRINTLN("=== Sonar Test ===");
    DBG_PRINTLN("Place objects at known distances.");
    DBG_PRINTLN();

    while (true) {
        const SonarReading r = ctx.sonar.read();

        for (int i = 0; i < SONAR_COUNT; i++) {
            DBG_PRINT("S");
            DBG_PRINT(i);
            DBG_PRINT(": ");
            if (r.valid[i]) {
                DBG_PRINT(r.distance[i], 1);
                DBG_PRINT(" cm\t");
            } else {
                DBG_PRINT("TIMEOUT\t");
            }
        }
        DBG_PRINTLN();

        delay(100);
    }
}
