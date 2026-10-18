#include <tests/calibration/motor/motor_calibration.h>
#include <cmath>
#include <debug.h>
#include <hardware/io/encoder/Encoder.h>

namespace
{
    constexpr int SPEED_SAMPLES = 20;
    constexpr int SPEED_DISCARD = 4;
    constexpr int SPEED_PERIOD_MS = 50;
    constexpr float MAX_STATIC_NOISE_TPS = 5.0f;
    constexpr float MIN_EXPECTED_TPS = 50.0f;

    void resetEncoders() {
        for (uint8_t i = 0; i < 3; i++) encoder_reset(i);
    }
}

bool runStaticPhase(MotorCalibration& cal) {
    DBG_PRINTLN("\n[Phase 1/5] Static validation...");
    cal.log.lastPhase = 1;

    cal.ctx.motorDriver.driveMotorsDirect(0, 0, 0);
    resetEncoders();
    updateLoop(cal.ctx.motorDriver, 1000);

    bool staticOk = true;
    for (uint8_t i = 0; i < 3; i++) {
        encoder_update_speed(i);
        const float speed = fabsf(encoder_get_speed(i));
        cal.log.staticNoise[i] = speed;
        if (speed > MAX_STATIC_NOISE_TPS) {
            DBG_PRINTLN("  FAIL: M" + String(i + 1) + " reads " +
                         String(speed, 1) + " t/s while stopped");
            staticOk = false;
        }
    }
    cal.log.phase1_staticOk = staticOk;

    if (!staticOk) {
        DBG_PRINTLN("Calibration ABORTED: encoder noise too high.");
        cal.result.valid = false;
        cal.saveLog();
        DBG_PRINTLN("Log saved to EEPROM (Phase 1 abort).");
        return false;
    }
    DBG_PRINTLN("  OK");
    cal.saveLog();
    return true;
}

bool runMaxSpeedPhase(MotorCalibration& cal) {
    DBG_PRINTLN("\n[Phase 2/5] Max speed (spinning in place)...");
    cal.log.lastPhase = 2;
    Motors& md = cal.ctx.motorDriver;

    md.setPIGains(0, 0, 0);
    resetEncoders();

    md.driveMotorsDirect(100, 100, 100);
    updateLoop(md, 500);
    averageSpeeds(md, SPEED_SAMPLES, SPEED_DISCARD, SPEED_PERIOD_MS, cal.result.maxTicksPerSec);
    for (uint8_t i = 0; i < 3; i++) {
        cal.log.maxTicksPerSec[i] = cal.result.maxTicksPerSec[i];
        DBG_PRINTLN("  M" + String(i + 1) + ": " + String(cal.result.maxTicksPerSec[i], 1) + " t/s");
    }
    stopAndSettle(md, 3000);

    bool ok = true;
    for (uint8_t i = 0; i < 3; i++) {
        if (cal.result.maxTicksPerSec[i] < MIN_EXPECTED_TPS) {
            DBG_PRINTLN("  FAIL: M" + String(i + 1) + " read " +
                         String(cal.result.maxTicksPerSec[i], 1) +
                         " t/s at 100% PWM (expected > " + String(MIN_EXPECTED_TPS, 0) + ").");
            ok = false;
        }
    }
    if (ok) return true;

    DBG_PRINTLN("Calibration ABORTED at Phase 2.");
    DBG_PRINTLN("  Probable causes:");
    DBG_PRINTLN("    1) Encoder wiring / interrupts (run testEncoder to verify).");
    DBG_PRINTLN("    2) Motor wiring or power (run testDriveForward to verify motion).");
    DBG_PRINTLN("    3) PWM output dead (check MIN_SPEED/MAX_SPEED and pin init).");
    cal.result.valid    = false;
    cal.result.dirValid = false;
    cal.log.motorValid  = false;
    cal.log.dirValid    = false;
    cal.saveLog();
    DBG_PRINTLN("Log saved to EEPROM (address " + String(CAL_LOG_ADDR) + ").");
    md.driveMotorsDirect(0, 0, 0);
    return false;
}

void runLinearityPhase(MotorCalibration& cal) {
    DBG_PRINTLN("\n[Phase 3/5] Linearity at 50%...");
    cal.log.lastPhase = 3;
    Motors& md = cal.ctx.motorDriver;
    CalibrationResult& r = cal.result;

    resetEncoders();
    md.driveMotorsDirect(50, 50, 50);
    updateLoop(md, 500);
    averageSpeeds(md, SPEED_SAMPLES, SPEED_DISCARD, SPEED_PERIOD_MS, r.speedAt50);
    for (uint8_t i = 0; i < 3; i++) {
        const float expected = r.maxTicksPerSec[i] * 0.5f;
        r.linearityRatio[i] = (expected > 0) ? r.speedAt50[i] / expected : 0;
        cal.log.speedAt50[i]      = r.speedAt50[i];
        cal.log.linearityRatio[i] = r.linearityRatio[i];
        DBG_PRINTLN("  M" + String(i + 1) + ": " + String(r.speedAt50[i], 1) + " t/s (" +
                    String(r.linearityRatio[i], 3) + ")");
    }
    stopAndSettle(md, 3000);
}

void runForwardDrivePhase(MotorCalibration& cal) {
    DBG_PRINTLN("\n[Phase 4/5] Forward drive validation...");
    cal.log.lastPhase = 4;
    cal.saveLog();

    cal.ctx.motorDriver.setPIGains(0, 0, 0);
    cal.ctx.compass.setPD(2.0f, 0.5f, 30.0f, 1.0f);
    cal.ctx.compass.reset();

    driveHoldingHeading(cal.ctx, 0, 30, 3000);

    cal.ctx.compass.update();
    cal.result.headingDrift = cal.ctx.compass.getOffset();
    cal.log.phase4_headingDrift = cal.result.headingDrift;

    cal.ctx.motorDriver.driveDegrees(0, 0, 0);
    updateLoop(cal.ctx.motorDriver, 500);

    DBG_PRINTLN("  Heading drift: " + String(cal.result.headingDrift, 2) + " deg over 3s");
}
