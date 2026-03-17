#include "tests.h"
#include "debug.h"
#include <Arduino.h>

void testSonar(const TestContext& ctx) {
    DBG_PRINTLN("=== Sonar Test ===");
    DBG_PRINTLN("Place objects at known distances.");
    DBG_PRINTLN();

    while (true) {
        auto [distance] = ctx.sonar.read();

        for (int i = 0; i < SONAR_COUNT; i++) {
            DBG_PRINT("S");
            DBG_PRINT(i);
            DBG_PRINT(": ");
            DBG_PRINT(distance[i], 1);
            DBG_PRINT(" cm\t");
        }
        DBG_PRINTLN();

        delay(100);
    }
}
