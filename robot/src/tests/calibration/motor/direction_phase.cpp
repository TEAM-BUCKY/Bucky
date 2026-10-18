#include <tests/calibration/motor/motor_calibration.h>
#include <cmath>
#include <debug.h>
#include <hardware/io/cordic/cordic.h>
#include <hardware/io/encoder/Encoder.h>
#include <helpers/constants.h>
#include <helpers/math/Math.h>

namespace
{
    constexpr float    BASELINE_SCALE       = 40.0f;
    constexpr uint32_t DIR_SETTLE_MS        = 200;
    constexpr uint32_t DIR_MEASURE_MS       = 500;
    constexpr uint32_t DIR_STOP_MS          = 300;
    constexpr int      DIR_SAMPLE_PERIOD_MS = 50;
    constexpr int      DIR_DISCARD          = 2;
    constexpr int      DIR_TOTAL_SAMPLES    = DIR_MEASURE_MS / DIR_SAMPLE_PERIOD_MS;
    constexpr float    RETURN_SCALE         = 40.0f;  // match the outbound speed so the robot ends near centre
    constexpr uint32_t DIR_RETURN_MS        = 500;
    constexpr uint32_t DIR_RETURN_SETTLE_MS = 300;
    constexpr float    MIN_ACCEL_VEL_MPS    = 0.05f;

    // Disabled: DMA I2C reads after flash writes hard-fault this MCU.
    constexpr bool ACCEL_CROSSCHECK = false;

    float directionDegrees(const uint8_t i) {
        return static_cast<float>(i) * (360.0f / NUM_DIRECTIONS);
    }

    float wrapSigned(const float degrees) {
        return Math::wrapDegrees(degrees + 180.0f) - 180.0f;
    }

    // Body-frame velocity integrated from the accelerometer while ramping up in each direction.
    struct AccelSweep {
        float bias[3] = {};
        float rawDeg[NUM_DIRECTIONS] = {};
        float velMps[NUM_DIRECTIONS] = {};
        float vx = 0, vy = 0;
        uint32_t lastMs = 0;

        void begin() {
            vx = vy = 0;
            lastMs = millis();
        }

        void sample(Accelerometer& accel) {
            float ax, ay, az;
            if (!accel.read(ax, ay, az)) return;
            const uint32_t now = millis();
            const float dt = static_cast<float>(now - lastMs) * 0.001f;
            lastMs = now;
            vx += (ax - bias[0]) * 9.81f * dt;
            vy += (ay - bias[1]) * 9.81f * dt;
        }

        void finish(const uint8_t i) {
            rawDeg[i] = Math::radiansToDegrees(atan2f(vx, vy));   // clockwise from accel +y, like thetaCmd
            velMps[i] = sqrtf(vx * vx + vy * vy);
        }
    };

    // At theta=0 the kinematics give m1 < 0, m2 ≈ 0, m3 > 0; anything else means the encoders are miswired.
    bool preflightSignsOk(MotorCalibration& cal) {
        cal.log.lastDirection = CAL_DIR_PREFLIGHT;
        cal.log.timestampMs = millis();
        DBG_PRINTLN("  CalibrationLog size: " + String(sizeof(CalibrationLog)) +
                    " bytes, writing to EEPROM addr " + String(CAL_LOG_ADDR) +
                    ", end = " + String(CAL_LOG_ADDR + sizeof(CalibrationLog)));
        eepromWrite(CAL_LOG_ADDR, cal.log);
        DBG_PRINTLN("  EEPROM save OK, starting pre-flight drive...");

        for (uint8_t k = 0; k < 3; k++) encoder_reset(k);
        driveHoldingHeading(cal.ctx, 0, BASELINE_SCALE, 500);
        for (uint8_t k = 0; k < 3; k++) encoder_update_speed(k);
        cal.ctx.motorDriver.driveMotorsDirect(0, 0, 0);
        cal.ctx.motorDriver.syncUpdateAllMotors();

        const float m1 = encoder_get_speed(0);
        const float m2 = encoder_get_speed(1);
        const float m3 = encoder_get_speed(2);
        const bool signsOk = (m1 < 0) && (m3 > 0) && (fabsf(m2) < fabsf(m1) * 0.5f);
        cal.log.preFlightMotorSpeed[0] = m1;
        cal.log.preFlightMotorSpeed[1] = m2;
        cal.log.preFlightMotorSpeed[2] = m3;
        cal.log.phase5_signsOk = signsOk;

        cal.log.lastDirection = CAL_DIR_ACCEL_BIAS;
        cal.saveLog();

        DBG_PRINTLN("  Pre-flight (theta=0): m1=" + String(m1, 1) + " m2=" +
                    String(m2, 1) + " m3=" + String(m3, 1) +
                    (signsOk ? "  [signs OK]" : "  [ENCODER SIGN SUSPECT]"));
        return signsOk;
    }

    void driveBackToCentre(Motors& md, const float thetaCmd) {
        const uint32_t start = millis();
        while (millis() - start < DIR_RETURN_MS) {
            driveDirectDegrees(md, thetaCmd + 180.0f, RETURN_SCALE, 0);
            md.syncUpdateAllMotors();
            delay(10);
        }
        stopAndSettle(md, DIR_RETURN_SETTLE_MS);
    }

    // Drives in direction i, then derives the actual heading of travel from the signed wheel speeds.
    void measureDirection(MotorCalibration& cal, AccelSweep& accel, const uint8_t i) {
        const TestContext& ctx = cal.ctx;
        CalibrationLog& log = cal.log;
        const float thetaCmd = directionDegrees(i);

        for (uint8_t k = 0; k < 3; k++) encoder_reset(k);

        if (ACCEL_CROSSCHECK) accel.begin();
        driveHoldingHeading(ctx, thetaCmd, BASELINE_SCALE, DIR_SETTLE_MS, [&] {
            if (ACCEL_CROSSCHECK) accel.sample(ctx.accel);
        });
        if (ACCEL_CROSSCHECK) accel.finish(i);
        log.dirAccelRawDeg[i] = accel.rawDeg[i];
        log.dirAccelVelMps[i] = accel.velMps[i];

        float sums[3] = {0, 0, 0};
        int   count   = 0;
        for (int s = 0; s < DIR_TOTAL_SAMPLES; s++) {
            ctx.compass.update();
            const float rot = ctx.compass.computeRotation(0);
            driveDirectDegrees(ctx.motorDriver, thetaCmd, BASELINE_SCALE, rot);
            ctx.motorDriver.syncUpdateAllMotors();
            for (uint8_t k = 0; k < 3; k++) encoder_update_speed(k);
            if (s >= DIR_DISCARD) {
                for (uint8_t k = 0; k < 3; k++) sums[k] += encoder_get_speed(k);   // signed
                count++;
            }
            delay(DIR_SAMPLE_PERIOD_MS);
        }

        for (uint8_t k = 0; k < 3; k++)
            log.dirRawTicks[i][k] = encoder_get_ticks(k);

        stopAndSettle(ctx.motorDriver, DIR_STOP_MS);
        driveBackToCentre(ctx.motorDriver, thetaCmd);

        float mPct[3];
        for (uint8_t k = 0; k < 3; k++)
            mPct[k] = 100.0f * (sums[k] / static_cast<float>(count)) / cal.result.maxTicksPerSec[k];

        // Forward kinematics of the three-wheel omni base.
        const float vxS = (mPct[0] - 2.0f * mPct[1] + mPct[2]) / 3.0f;
        const float vyS = (mPct[2] - mPct[0]) / (2.0f * SIN_60_F);

        float angleRad, magPct;
        cordic_atan2_mod(vxS, vyS, &angleRad, &magPct);
        const float actualDeg = Math::radiansToDegrees(angleRad);
        const float offsetDeg = wrapSigned(thetaCmd - actualDeg);

        float sMul = (magPct > 1.0f) ? (BASELINE_SCALE / magPct) : 3.0f;
        if (sMul < 0.3f) sMul = 0.3f;
        if (sMul > 3.0f) sMul = 3.0f;

        const float maxAbs   = fmaxf(fmaxf(fabsf(mPct[0]), fabsf(mPct[1])), fabsf(mPct[2]));
        const bool  satOk    = maxAbs < 95.0f;
        const bool  scaleOk  = sMul >= 0.6f && sMul <= 1.6f;
        const bool  offsetOk = fabsf(offsetDeg) <= 25.0f;

        cal.result.dirScale[i]     = sMul;
        cal.result.dirOffsetDeg[i] = offsetDeg;
        if (!satOk || !scaleOk || !offsetOk) cal.result.dirValid = false;

        for (uint8_t k = 0; k < 3; k++) log.dirMotorPct[i][k] = mPct[k];
        log.dirActualDeg[i] = actualDeg;
        log.dirOffsetDeg[i] = offsetDeg;
        log.dirSpeedPct[i]  = magPct;
        log.dirScale[i]     = sMul;
        log.dirFlags[i]     = (satOk ? DIR_FLAG_SAT_OK : 0) | (scaleOk ? DIR_FLAG_SCALE_OK : 0) |
                              (offsetOk ? DIR_FLAG_OFFSET_OK : 0);

        DBG_PRINTLN("  " + String(thetaCmd, 0) + "  | " +
                    String(actualDeg, 1) + " | " +
                    String(offsetDeg, 2) + " | " +
                    String(magPct, 1) + " | " +
                    String(sMul, 3) +
                    (satOk ? "" : "  [SAT]") +
                    (scaleOk ? "" : "  [SCALE-OOR]") +
                    (offsetOk ? "" : "  [OFFSET-OOR]"));

        log.lastDirection = i;
        cal.saveLog();
    }

    void accelCrossCheck(MotorCalibration& cal, const AccelSweep& accel) {
        float sumSin = 0, sumCos = 0;
        int   moving = 0;
        for (uint8_t i = 0; i < NUM_DIRECTIONS; i++) {
            if (accel.velMps[i] < MIN_ACCEL_VEL_MPS) continue;
            const float deltaRad = Math::degreesToRadians(accel.rawDeg[i] - directionDegrees(i));
            sumSin += sinf(deltaRad);
            sumCos += cosf(deltaRad);
            moving++;
        }
        if (moving < NUM_DIRECTIONS / 2) {
            DBG_PRINTLN("  Accel frame: insufficient motion (" + String(moving) +
                        "/" + String(NUM_DIRECTIONS) + " dirs); cross-check skipped");
            return;
        }

        const float frameDeg = Math::radiansToDegrees(atan2f(sumSin, sumCos));
        cal.log.accelFrameDeg       = frameDeg;
        cal.log.phase5_accelFrameOk = true;
        DBG_PRINTLN("  Accel frame offset (accel-Y vs robot-forward): " +
                    String(frameDeg, 1) + " deg (" + String(moving) +
                    "/" + String(NUM_DIRECTIONS) + " dirs with motion)");
        DBG_PRINTLN("  Accel xcheck: theta | accel_dir | accel_off | enc_off | |v| m/s");
        for (uint8_t i = 0; i < NUM_DIRECTIONS; i++) {
            const float thetaCmd    = directionDegrees(i);
            const float accelRel    = Math::wrapDegrees(accel.rawDeg[i] - frameDeg);
            const float accelOffset = wrapSigned(thetaCmd - accelRel);
            const float disagreement = accelOffset - cal.result.dirOffsetDeg[i];
            const bool  motionOk = accel.velMps[i] >= MIN_ACCEL_VEL_MPS;
            const bool  agreeOk  = motionOk && fabsf(disagreement) < 35.0f;
            if (!agreeOk) cal.result.dirValid = false;

            cal.log.dirAccelOffsetDeg[i] = accelOffset;
            if (motionOk) cal.log.dirFlags[i] |= DIR_FLAG_MOTION_OK;
            if (agreeOk)  cal.log.dirFlags[i] |= DIR_FLAG_AGREE_OK;

            DBG_PRINTLN("  " + String(thetaCmd, 0) + "  | " +
                        String(accelRel, 1) + " | " +
                        String(accelOffset, 2) + " | " +
                        String(cal.result.dirOffsetDeg[i], 2) + " | " +
                        String(accel.velMps[i], 3) +
                        (motionOk ? "" : "  [NO-MOTION]") +
                        (agreeOk  ? "" : "  [DISAGREE]"));
        }
    }
}

void runDirectionPhase(MotorCalibration& cal) {
    DBG_PRINTLN("\n[Phase 5/5] Direction calibration (12 directions, ~22s)...");
    const TestContext& ctx = cal.ctx;

    // Open loop: PI feedback combined with encoder_reset makes the drive stutter.
    ctx.motorDriver.setPIGains(0, 0, 0);
    // Strong heading hold so open-loop motor imbalance doesn't rotate the robot.
    ctx.compass.setPD(2.0f, 0.5f, 30.0f, 1.0f);

    cal.log.lastPhase = 5;
    cal.saveLog();

    ctx.compass.update();
    ctx.compass.reset();

    const bool signsOk = preflightSignsOk(cal);

    AccelSweep accel;
    DBG_PRINTLN("  Accel cross-check disabled (DMA/flash conflict)");
    for (uint8_t k = 0; k < 3; k++) cal.log.accelBias[k] = accel.bias[k];
    cal.log.phase5_accelOk = ACCEL_CROSSCHECK;
    updateLoop(ctx.motorDriver, 200);

    cal.result.dirValid = signsOk;
    DBG_PRINTLN("  theta | actual | offset | speed% | scale");

    cal.log.lastDirection = CAL_DIR_LOOP_START;
    cal.saveLog();

    for (uint8_t i = 0; i < NUM_DIRECTIONS; i++) {
        measureDirection(cal, accel, i);
        delay(2000);
        ctx.compass.update();
        ctx.compass.reset();
    }

    stopAndSettle(ctx.motorDriver, DIR_STOP_MS);

    if (ACCEL_CROSSCHECK) accelCrossCheck(cal, accel);

    DBG_PRINTLN(cal.result.dirValid
                ? "  Direction cal: VALID"
                : "  Direction cal: SUSPECT - not saved");
}
