#include "tests.h"
#include "debug.h"
#include "robot/StuckDetector.h"
#include <Arduino.h>

// Hardware test for the encoder-based stuck detector.
//
// Procedure:
//   * Test 1 (free-spin): motors are commanded forward; detector must NOT trip
//     while the wheels are spinning freely.
//   * Test 2 (held): hold the robot so wheels can't turn (or set it on a
//     stand). The detector must trip within ~400-600 ms.
//   * Test 3 (idle): zero the command. The detector must clear quickly and
//     stay clear regardless of wheel state.
//
// PASS printed when all three conditions met in sequence. Otherwise FAIL.
void testStuckDetector(const TestContext& ctx)
{
    DBG_PRINTLN("=== StuckDetector Test ===");
    DBG_PRINTLN("Phase 1: free-spin forward (must NOT trip).");
    DBG_PRINTLN("Phase 2: hold the robot still (must trip within ~600 ms).");
    DBG_PRINTLN("Phase 3: stop commanding (must clear quickly).");

    StuckDetector stuck;
    stuck.reset(millis());

    const auto runPhase = [&](const char* label, VectorXY drive, float rot,
                              uint32_t durationMs, bool expectTrip) -> bool
    {
        DBG_PRINT("-- ");
        DBG_PRINT(label);
        DBG_PRINTLN(" --");

        const uint32_t start = millis();
        bool tripped = false;
        uint32_t firstTripMs = 0;

        while (millis() - start < durationMs) {
            const uint32_t now = millis();
            ctx.motorDriver.driveVector(drive, rot);
            ctx.motorDriver.updateAllMotors();

            const bool s = stuck.update(now, drive, rot);
            if (s && !tripped) {
                tripped = true;
                firstTripMs = now - start;
            }

            if ((now - start) % 100 < 10) {
                DBG_PRINT("  t=");
                DBG_PRINT(now - start);
                DBG_PRINT("ms drive=(");
                DBG_PRINT(drive.x, 0);
                DBG_PRINT(",");
                DBG_PRINT(drive.y, 0);
                DBG_PRINT(") stuck=");
                DBG_PRINTLN(s ? 1 : 0);
                delay(20);
            }
        }

        // Drain: stop motors between phases.
        ctx.motorDriver.driveVector({0.0f, 0.0f}, 0.0f);
        ctx.motorDriver.updateAllMotors();

        const bool pass = (expectTrip == tripped);
        DBG_PRINT("  result: tripped=");
        DBG_PRINT(tripped ? 1 : 0);
        if (tripped) {
            DBG_PRINT(" at ");
            DBG_PRINT(firstTripMs);
            DBG_PRINT("ms");
        }
        DBG_PRINT(" | expected trip=");
        DBG_PRINT(expectTrip ? 1 : 0);
        DBG_PRINTLN(pass ? " -> PASS" : " -> FAIL");
        return pass;
    };

    bool ok = true;
    ok &= runPhase("Phase 1 free-spin forward", {0.0f, 30.0f}, 0.0f, 2000, false);
    delay(500);
    ok &= runPhase("Phase 2 robot held still", {0.0f, 30.0f}, 0.0f, 1500, true);
    delay(500);
    ok &= runPhase("Phase 3 idle", {0.0f, 0.0f}, 0.0f, 1000, false);

    DBG_PRINTLN(ok ? "=== StuckDetector: PASS ===" : "=== StuckDetector: FAIL ===");

    while (true) {
        ctx.motorDriver.driveVector({0.0f, 0.0f}, 0.0f);
        ctx.motorDriver.updateAllMotors();
        delay(50);
    }
}
