#include <tests/calibration/motor/motor_calibration.h>
#include <debug.h>

void testCalibrate(const TestContext& ctx) {
    DBG_PRINTLN("=== Motor Auto-Calibration ===");
    DBG_PRINTLN("Place robot on target surface with ~1m space.");
    DBG_PRINTLN("Starting in 3 seconds...");
    delay(3000);

    MotorCalibration cal = {ctx, {}, {}};
    cal.log.magic         = CAL_LOG_MAGIC;
    cal.log.lastPhase     = 0;
    cal.log.lastDirection = CAL_DIR_NONE;
    cal.log.completed     = false;
    cal.saveLog();

    if (!runStaticPhase(cal)) haltForever();
    if (!runMaxSpeedPhase(cal)) haltForever();
    runLinearityPhase(cal);
    runForwardDrivePhase(cal);
    runDirectionPhase(cal);
    reportAndSave(cal);

    haltForever();
}
