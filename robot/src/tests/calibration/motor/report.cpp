#include <tests/calibration/motor/motor_calibration.h>
#include <debug.h>
#include <EEPROM.h>

namespace
{
    bool resultsInRange(const CalibrationResult& r) {
        for (uint8_t i = 0; i < 3; i++) {
            if (r.maxTicksPerSec[i] < 100.0f || r.maxTicksPerSec[i] > 10000.0f) return false;
            if (r.linearityRatio[i] < 0.5f || r.linearityRatio[i] > 1.5f) return false;
        }
        return true;
    }

    // Keeps any compass calibration already in EEPROM; direction data only when the sweep was valid.
    void saveCalibration(const CalibrationResult& r) {
        StoredCalibration cal = {};
        EEPROM.get(0, cal);
        const bool keepMag = (cal.magic == CALIBRATION_MAGIC) && cal.magValid;
        if (!keepMag) {
            for (uint8_t i = 0; i < 3; i++) { cal.magOffset[i] = 0.0f; cal.magScale[i] = 1.0f; }
            cal.magValid = false;
        }
        cal.magic = CALIBRATION_MAGIC;
        for (uint8_t i = 0; i < 3; i++) {
            cal.maxTicksPerSec[i] = r.maxTicksPerSec[i];
            cal.linearityRatio[i] = r.linearityRatio[i];
        }
        for (uint8_t i = 0; i < NUM_DIRECTIONS; i++) {
            cal.dirScale[i]     = r.dirValid ? r.dirScale[i]     : 1.0f;
            cal.dirOffsetDeg[i] = r.dirValid ? r.dirOffsetDeg[i] : 0.0f;
        }
        cal.dirValid = r.dirValid;
        eepromWrite(0, cal);
        DBG_PRINTLN(r.dirValid
            ? "\nCalibration saved to EEPROM (motor + direction)."
            : "\nCalibration saved to EEPROM (motor only; direction cal skipped).");
    }
}

// Every phase ran, so the result is always saved; the range check only sets the STATUS line.
void reportAndSave(MotorCalibration& cal) {
    CalibrationResult& r = cal.result;
    const bool rangesOk = resultsInRange(r);
    r.valid = true;

    DBG_PRINTLN("\n===== CALIBRATION RESULTS =====");
    DBG_PRINTLN(rangesOk ? "STATUS: VALID" : "STATUS: SUSPECT - review values (saved anyway)");
    DBG_PRINTLN("");

    for (uint8_t i = 0; i < 3; i++) {
        DBG_PRINTLN("Motor " + String(i + 1) + ":  max=" +
                     String(r.maxTicksPerSec[i], 1) + " t/s  linearity=" +
                     String(r.linearityRatio[i], 3));
    }

    saveCalibration(r);

    DBG_PRINTLN("");
    DBG_PRINTLN("// Or apply manually:");
    for (uint8_t i = 0; i < 3; i++) {
        DBG_PRINTLN("motorDriver.setMaxTicksPerSec(" + String(i) + ", " +
                     String(r.maxTicksPerSec[i], 1) + ");");
    }
    DBG_PRINTLN("================================");

    cal.log.motorValid = r.valid;
    cal.log.dirValid   = r.dirValid;
    cal.log.completed  = true;
    cal.saveLog();
    DBG_PRINTLN("Calibration log saved to EEPROM (address " + String(CAL_LOG_ADDR) + ").");

    cal.ctx.motorDriver.driveMotorsDirect(0, 0, 0);
}
