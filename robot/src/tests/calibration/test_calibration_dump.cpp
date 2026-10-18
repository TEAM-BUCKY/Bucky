#include <tests/tests.h>
#include <tests/calibration/calibration_storage.h>
#include <debug.h>
#include <Arduino.h>
#include <EEPROM.h>

namespace
{
    String triple(const float v[3], const unsigned int decimals) {
        return String(v[0], decimals) + "/" + String(v[1], decimals) + "/" + String(v[2], decimals);
    }

    const char* yesNo(const bool b) { return b ? "Y" : "N"; }

    void dumpStoredCalibration() {
        StoredCalibration cal = {};
        EEPROM.get(0, cal);
        DBG_PRINTLN("\n[StoredCalibration @ 0]");
        DBG_PRINT("  magic: 0x"); DBG_PRINTLN(cal.magic, HEX);
        if (cal.magic != CALIBRATION_MAGIC) {
            DBG_PRINTLN("  (magic mismatch — no valid applied calibration)");
            return;
        }
        for (uint8_t i = 0; i < 3; i++) {
            DBG_PRINTLN("  M" + String(i + 1) +
                        ": max=" + String(cal.maxTicksPerSec[i], 1) +
                        " t/s  lin=" + String(cal.linearityRatio[i], 3));
        }
        DBG_PRINTLN("  dirValid: " + String(cal.dirValid ? "true" : "false"));
        DBG_PRINTLN("  dir: theta | scale | offsetDeg");
        for (uint8_t i = 0; i < NUM_DIRECTIONS; i++) {
            DBG_PRINTLN("    " + String(i * 30) + "  | " +
                        String(cal.dirScale[i], 3) + " | " +
                        String(cal.dirOffsetDeg[i], 2));
        }
        DBG_PRINTLN("  magValid: " + String(cal.magValid ? "true" : "false"));
        if (cal.magValid) {
            DBG_PRINTLN("  magOffset: " + String(cal.magOffset[0], 1) + " " +
                        String(cal.magOffset[1], 1) + " " +
                        String(cal.magOffset[2], 1));
            DBG_PRINTLN("  magScale:  " + String(cal.magScale[0], 3) + " " +
                        String(cal.magScale[1], 3) + " " +
                        String(cal.magScale[2], 3));
        }
    }

    String lastDirectionName(const uint8_t dir) {
        switch (dir) {
            case CAL_DIR_NONE:       return "none";
            case CAL_DIR_PREFLIGHT:  return "in pre-flight drive";
            case CAL_DIR_LOOP_START: return "entering dir loop";
            case CAL_DIR_ACCEL_BIAS: return "pre-flight done, in accel bias";
            default:                 return String(dir * 30) + "deg (#" + String(dir) + ")";
        }
    }

    String crashLocation(const CalibrationLog& log) {
        if (log.lastPhase < 5) return " in phase " + String(log.lastPhase);
        switch (log.lastDirection) {
            case CAL_DIR_NONE:       return " phase 5 (before pre-flight)";
            case CAL_DIR_PREFLIGHT:  return " phase 5 (during pre-flight drive)";
            case CAL_DIR_ACCEL_BIAS: return " phase 5 (during accel bias)";
            case CAL_DIR_LOOP_START: return " phase 5 (entering direction loop)";
            default:
                return " phase 5 (after dir " + String(log.lastDirection * 30) + "deg, before " +
                       String(((log.lastDirection + 1) % 12) * 30) + "deg)";
        }
    }

    void dumpDirectionTables(const CalibrationLog& log) {
        DBG_PRINTLN("  [Phase 5] theta | m1% | m2% | m3% | act | off | spd% | scale | flags");
        for (uint8_t i = 0; i < NUM_DIRECTIONS; i++) {
            const uint8_t f = log.dirFlags[i];
            DBG_PRINTLN("    " + String(i * 30) + " | " +
                        String(log.dirMotorPct[i][0], 1) + " | " +
                        String(log.dirMotorPct[i][1], 1) + " | " +
                        String(log.dirMotorPct[i][2], 1) + " | " +
                        String(log.dirActualDeg[i], 1) + " | " +
                        String(log.dirOffsetDeg[i], 2) + " | " +
                        String(log.dirSpeedPct[i], 1) + " | " +
                        String(log.dirScale[i], 3) + " | " +
                        ((f & DIR_FLAG_SAT_OK) ? "" : "[SAT]") +
                        ((f & DIR_FLAG_SCALE_OK) ? "" : "[SCALE-OOR]") +
                        ((f & DIR_FLAG_OFFSET_OK) ? "" : "[OFFSET-OOR]") +
                        ((f & DIR_FLAG_MOTION_OK) ? "" : "[NO-MOTION]") +
                        ((f & DIR_FLAG_AGREE_OK) ? "" : "[DISAGREE]"));
        }

        DBG_PRINTLN("  [Phase 5] theta | rawTicks m1 | m2 | m3");
        for (uint8_t i = 0; i < NUM_DIRECTIONS; i++) {
            DBG_PRINTLN("    " + String(i * 30) + " | " +
                        String(log.dirRawTicks[i][0]) + " | " +
                        String(log.dirRawTicks[i][1]) + " | " +
                        String(log.dirRawTicks[i][2]));
        }

        DBG_PRINTLN("  [Phase 5] accelFrameOk=" + String(yesNo(log.phase5_accelFrameOk)) +
                    "  accelFrameDeg=" + String(log.accelFrameDeg, 1));
        DBG_PRINTLN("  [Phase 5] theta | accel_raw_deg | accel_off | |v| m/s");
        for (uint8_t i = 0; i < NUM_DIRECTIONS; i++) {
            DBG_PRINTLN("    " + String(i * 30) + " | " +
                        String(log.dirAccelRawDeg[i], 1) + " | " +
                        String(log.dirAccelOffsetDeg[i], 2) + " | " +
                        String(log.dirAccelVelMps[i], 3));
        }
    }

    void dumpLog() {
        CalibrationLog log = {};
        EEPROM.get(CAL_LOG_ADDR, log);
        DBG_PRINTLN("\n[CalibrationLog @ " + String(CAL_LOG_ADDR) + "]");
        DBG_PRINT("  magic: 0x"); DBG_PRINTLN(log.magic, HEX);
        if (log.magic != CAL_LOG_MAGIC) {
            DBG_PRINTLN("  (magic mismatch — no log saved)");
            return;
        }

        DBG_PRINTLN("  timestampMs: " + String(log.timestampMs));
        DBG_PRINTLN("  completed=" + String(yesNo(log.completed)) +
                    "  lastPhase=" + String(log.lastPhase) +
                    "  lastDir=" + lastDirectionName(log.lastDirection));
        if (!log.completed)
            DBG_PRINTLN("  *** CRASHED" + crashLocation(log) + " ***");
        DBG_PRINTLN("  motorValid=" + String(yesNo(log.motorValid)) +
                    "  dirValid=" + String(yesNo(log.dirValid)));

        DBG_PRINTLN("  [Phase 1] staticOk=" + String(yesNo(log.phase1_staticOk)) +
                    "  noise=" + triple(log.staticNoise, 1) + " t/s");
        DBG_PRINTLN("  [Phase 2] maxTicksPerSec=" + triple(log.maxTicksPerSec, 1));
        DBG_PRINTLN("  [Phase 3] speedAt50=" + triple(log.speedAt50, 1) +
                    "  lin=" + triple(log.linearityRatio, 3));
        DBG_PRINTLN("  [Phase 4] headingDrift=" + String(log.phase4_headingDrift, 2) + " deg");
        DBG_PRINTLN("  [Phase 5] signsOk=" + String(yesNo(log.phase5_signsOk)) +
                    "  preFlight m1/m2/m3=" + triple(log.preFlightMotorSpeed, 1));
        DBG_PRINTLN("  [Phase 5] accelOk=" + String(yesNo(log.phase5_accelOk)) +
                    "  bias=" + triple(log.accelBias, 3) + "g");

        dumpDirectionTables(log);
    }
}

void testCalibrationDump(const TestContext&) {
    DBG_PRINTLN("=== Calibration Dump ===");
    dumpStoredCalibration();
    dumpLog();
    DBG_PRINTLN("\n=== end dump ===");
    while (true) { delay(1000); }
}
