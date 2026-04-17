#include "tests.h"
#include "debug.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <utility/stm32_eeprom.h>
#include <cmath>
#include "io/encoder/Encoder.h"
#include "io/cordic/cordic.h"
#include "helpers/Math.h"

// EEPROM.put() flushes the whole 2KB flash page after every byte, so saving a
// 596-byte struct takes ~12s and burns hundreds of flash cycles. This helper
// fills the RAM buffer byte-by-byte and flushes the page exactly once.
// IMPORTANT: eeprom_buffer_fill() must run first — otherwise bytes we don't
// touch get flushed as their RAM-default (0), wiping anything else in the page.
template <typename T>
static void fastEEPROMPut(const uint32_t addr, const T& value) {
    eeprom_buffer_fill();
    const auto* ptr = reinterpret_cast<const uint8_t*>(&value);
    for (size_t i = 0; i < sizeof(T); i++) {
        eeprom_buffered_write_byte(addr + i, ptr[i]);
    }
    eeprom_buffer_flush();
}

#define CALIBRATION_MAGIC 0xCA1B0003
#define CAL_LOG_MAGIC     0xCA106001
#define CAL_LOG_ADDR      512

constexpr uint8_t NUM_DIRECTIONS = 12;
constexpr float SIN_60 = 0.8660254037844f;

struct StoredCalibration {
    uint32_t magic;
    float maxTicksPerSec[3];
    float linearityRatio[3];
    float dirScale[NUM_DIRECTIONS];
    float dirOffsetDeg[NUM_DIRECTIONS];
    bool  dirValid;
    float magOffset[3];   // LIS2MDL hard-iron X/Y/Z
    float magScale[3];    // LIS2MDL soft-iron diagonal X/Y/Z
    bool  magValid;
};

// Diagnostic log written unconditionally at the end of every run so calibration
// can be done without a laptop attached and reviewed later via testCalibrationDump.
struct CalibrationLog {
    uint32_t magic;
    uint32_t timestampMs;        // millis() at completion

    // Phase 1
    float staticNoise[3];
    bool  phase1_staticOk;

    // Phase 2 / 3
    float maxTicksPerSec[3];
    float speedAt50[3];
    float linearityRatio[3];

    // Phase 4
    float phase4_headingDrift;

    // Phase 5 pre-flight & bias
    float preFlightMotorSpeed[3];
    bool  phase5_signsOk;
    float accelBias[3];
    bool  phase5_accelOk;

    // Phase 5 per-direction: encoder side
    float dirMotorPct[NUM_DIRECTIONS][3];
    float dirActualDeg[NUM_DIRECTIONS];
    float dirOffsetDeg[NUM_DIRECTIONS];
    float dirSpeedPct[NUM_DIRECTIONS];
    float dirScale[NUM_DIRECTIONS];
    uint8_t dirFlags[NUM_DIRECTIONS];   // bit0=satOk, bit1=scaleOk, bit2=offsetOk, bit3=motionOk, bit4=accelAgreeOk

    // Phase 5 per-direction: accel side
    float dirAccelRawDeg[NUM_DIRECTIONS];
    float dirAccelVelMps[NUM_DIRECTIONS];
    float dirAccelOffsetDeg[NUM_DIRECTIONS];
    float accelFrameDeg;
    bool  phase5_accelFrameOk;

    // Phase 5 encoder diagnostic: raw ticks after settle+measure per direction
    int32_t dirRawTicks[NUM_DIRECTIONS][3];

    // Progress tracking (survives crashes)
    uint8_t lastPhase;          // last phase that started (1-5)
    uint8_t lastDirection;      // last direction completed in Phase 5 (0-11, 0xFF=none)
    bool    completed;          // true if calibration ran to the end

    // Summary
    bool motorValid;
    bool dirValid;
};

bool loadCalibration(MotorDriver& md, Compass& compass) {
    StoredCalibration cal = {};
    EEPROM.get(0, cal);

    if (cal.magic != CALIBRATION_MAGIC) return false;

    for (uint8_t i = 0; i < 3; i++) {
        if (cal.maxTicksPerSec[i] < 100.0f || cal.maxTicksPerSec[i] > 10000.0f)
            return false;
        md.setMaxTicksPerSec(i, cal.maxTicksPerSec[i]);
    }

    if (cal.dirValid) {
        md.setDirectionCalibration(cal.dirScale, cal.dirOffsetDeg);
        md.enableDirectionCalibration(true);
    }

    if (cal.magValid) {
        compass.setCalibration(cal.magOffset[0], cal.magOffset[1], cal.magOffset[2],
                               cal.magScale[0],  cal.magScale[1],  cal.magScale[2]);
    }
    return true;
}

// Compute omni-drive motor speeds and apply via driveMotorsDirect (no smooth ramp).
// Uses the same inverse kinematics + rotation scaling as driveRadians.
// After this, syncUpdateAllMotors will apply PI feedback if gains are non-zero.
static void driveDirectDegrees(MotorDriver& md, const float degrees, const float scale, const float rotation) {
    const float rad = Math::degreesToRadians(degrees);
    float sinR, cosR;
    cordic_sin_cos(rad, &sinR, &cosR);
    // Apply rotation directly as motor % (no scaling down). computeRotation
    // returns ±50 max (boosted for calibration). Rotation sign is negated because
    // M1/M3 pin swap reversed the physical rotation direction.
    const float m1 = fmaxf(-100.0f, fminf(100.0f, (0.5f * sinR - SIN_60 * cosR) * scale - rotation));
    const float m2 = fmaxf(-100.0f, fminf(100.0f, -sinR * scale - rotation));
    const float m3 = fmaxf(-100.0f, fminf(100.0f, (0.5f * sinR + SIN_60 * cosR) * scale - rotation));
    md.driveMotorsDirect(m1, m2, m3);
}

static void updateLoop(MotorDriver& md, const uint32_t durationMs) {
    const uint32_t start = millis();
    while (millis() - start < durationMs) {
        md.syncUpdateAllMotors();
        delay(5);
    }
}

static void measureSpeeds(MotorDriver& md, float outSpeeds[3]) {
    constexpr int SAMPLE_PERIOD_MS = 50;
    constexpr int TOTAL_SAMPLES = 40;
    constexpr int DISCARD = 6;

    float sums[3] = {};
    int validCount = 0;

    for (int s = 0; s < TOTAL_SAMPLES; s++) {
        delay(SAMPLE_PERIOD_MS);
        md.syncUpdateAllMotors();
        for (uint8_t i = 0; i < 3; i++)
            encoder_update_speed(i);

        if (s >= DISCARD) {
            for (uint8_t i = 0; i < 3; i++)
                sums[i] += fabsf(encoder_get_speed(i));
            validCount++;
        }
    }

    for (uint8_t i = 0; i < 3; i++)
        outSpeeds[i] = sums[i] / static_cast<float>(validCount);
}

struct CalibrationResult {
    float maxTicksPerSec[3];
    float speedAt50[3];
    float linearityRatio[3];
    float headingDrift;
    float dirScale[NUM_DIRECTIONS];
    float dirOffsetDeg[NUM_DIRECTIONS];
    bool  dirValid;
    bool  valid;
};

void testCalibrate(const TestContext& ctx) {
    DBG_PRINTLN("=== Motor Auto-Calibration ===");
    DBG_PRINTLN("Place robot on target surface with ~1m space.");
    DBG_PRINTLN("Starting in 3 seconds...");
    delay(3000);

    CalibrationResult result = {};
    CalibrationLog    log    = {};
    log.magic          = CAL_LOG_MAGIC;
    log.lastPhase      = 0;
    log.lastDirection  = 0xFF;
    log.completed      = false;
    log.timestampMs    = millis();
    fastEEPROMPut(CAL_LOG_ADDR, log);   // save initial state so crash leaves a trace

    // ========== Phase 1: Static validation ==========
    DBG_PRINTLN("\n[Phase 1/5] Static validation...");
    log.lastPhase = 1;

    ctx.motorDriver.driveMotorsDirect(0, 0, 0);
    for (uint8_t i = 0; i < 3; i++)
        encoder_reset(i);

    updateLoop(ctx.motorDriver, 1000);

    bool staticOk = true;
    for (uint8_t i = 0; i < 3; i++) {
        encoder_update_speed(i);
        const float speed = fabsf(encoder_get_speed(i));
        log.staticNoise[i] = speed;
        if (speed > 5.0f) {
            DBG_PRINTLN("  FAIL: M" + String(i + 1) + " reads " +
                         String(speed, 1) + " t/s while stopped");
            staticOk = false;
        }
    }
    log.phase1_staticOk = staticOk;

    if (!staticOk) {
        DBG_PRINTLN("Calibration ABORTED: encoder noise too high.");
        result.valid = false;
        log.timestampMs = millis();
        fastEEPROMPut(CAL_LOG_ADDR, log);
        DBG_PRINTLN("Log saved to EEPROM (Phase 1 abort).");
        while (true) { delay(1000); }
    }
    DBG_PRINTLN("  OK");
    log.timestampMs = millis();
    fastEEPROMPut(CAL_LOG_ADDR, log);

    // ========== Phase 2: Max speed measurement ==========
    DBG_PRINTLN("\n[Phase 2/5] Max speed (spinning in place)...");
    log.lastPhase = 2;

    // Zero PI gains so motors run at exact commanded PWM percentage (true open-loop).
    ctx.motorDriver.setPIGains(0, 0, 0);

    for (uint8_t i = 0; i < 3; i++)
        encoder_reset(i);

    // Measure each motor individually to avoid the robot driving into walls.
    for (uint8_t m = 0; m < 3; m++) {
        encoder_reset(m);
        const float speeds[3][3] = {{100,0,0}, {0,100,0}, {0,0,100}};
        ctx.motorDriver.driveMotorsDirect(speeds[m][0], speeds[m][1], speeds[m][2]);
        updateLoop(ctx.motorDriver, 300);
        // Measure this motor's max speed
        constexpr int P2_SAMPLES = 20, P2_DISCARD = 4, P2_PERIOD = 50;
        float sum = 0; int cnt = 0;
        for (int s = 0; s < P2_SAMPLES; s++) {
            delay(P2_PERIOD);
            ctx.motorDriver.syncUpdateAllMotors();
            encoder_update_speed(m);
            if (s >= P2_DISCARD) { sum += fabsf(encoder_get_speed(m)); cnt++; }
        }
        result.maxTicksPerSec[m] = sum / static_cast<float>(cnt);
        log.maxTicksPerSec[m] = result.maxTicksPerSec[m];
        ctx.motorDriver.driveMotorsDirect(0, 0, 0);
        updateLoop(ctx.motorDriver, 200);
        DBG_PRINTLN("  M" + String(m + 1) + ": " + String(result.maxTicksPerSec[m], 1) + " t/s");
    }

    ctx.motorDriver.driveMotorsDirect(0, 0, 0);
    updateLoop(ctx.motorDriver, 500);

    // Abort if any motor reports no motion at 100% PWM. Continuing with
    // maxTicksPerSec=0 makes PI divide by zero and corrupts every later phase.
    constexpr float MIN_EXPECTED_TPS = 50.0f;
    bool phase2Ok = true;
    for (uint8_t i = 0; i < 3; i++) {
        if (result.maxTicksPerSec[i] < MIN_EXPECTED_TPS) {
            DBG_PRINTLN("  FAIL: M" + String(i + 1) + " read " +
                         String(result.maxTicksPerSec[i], 1) +
                         " t/s at 100% PWM (expected > " + String(MIN_EXPECTED_TPS, 0) + ").");
            phase2Ok = false;
        }
    }
    if (!phase2Ok) {
        DBG_PRINTLN("Calibration ABORTED at Phase 2.");
        DBG_PRINTLN("  Probable causes:");
        DBG_PRINTLN("    1) Encoder wiring / interrupts (run testEncoder to verify).");
        DBG_PRINTLN("    2) Motor wiring or power (run testDriveForward to verify motion).");
        DBG_PRINTLN("    3) PWM output dead (check MIN_SPEED/MAX_SPEED and pin init).");
        result.valid    = false;
        result.dirValid = false;
        log.motorValid  = false;
        log.dirValid    = false;
        log.timestampMs = millis();
        fastEEPROMPut(CAL_LOG_ADDR, log);
        DBG_PRINTLN("Log saved to EEPROM (address " + String(CAL_LOG_ADDR) + ").");
        ctx.motorDriver.driveMotorsDirect(0, 0, 0);
        while (true) { delay(1000); }
    }

    // ========== Phase 3: Linearity at 50% ==========
    DBG_PRINTLN("\n[Phase 3/5] Linearity at 50%...");
    log.lastPhase = 3;

    // Measure each motor individually at 50% (same approach as Phase 2).
    for (uint8_t m = 0; m < 3; m++) {
        encoder_reset(m);
        const float speeds[3][3] = {{50,0,0}, {0,50,0}, {0,0,50}};
        ctx.motorDriver.driveMotorsDirect(speeds[m][0], speeds[m][1], speeds[m][2]);
        updateLoop(ctx.motorDriver, 300);
        constexpr int P3_SAMPLES = 20, P3_DISCARD = 4, P3_PERIOD = 50;
        float sum = 0; int cnt = 0;
        for (int s = 0; s < P3_SAMPLES; s++) {
            delay(P3_PERIOD);
            ctx.motorDriver.syncUpdateAllMotors();
            encoder_update_speed(m);
            if (s >= P3_DISCARD) { sum += fabsf(encoder_get_speed(m)); cnt++; }
        }
        result.speedAt50[m] = sum / static_cast<float>(cnt);
        const float expected = result.maxTicksPerSec[m] * 0.5f;
        result.linearityRatio[m] = (expected > 0) ? result.speedAt50[m] / expected : 0;
        log.speedAt50[m]       = result.speedAt50[m];
        log.linearityRatio[m]  = result.linearityRatio[m];
        ctx.motorDriver.driveMotorsDirect(0, 0, 0);
        updateLoop(ctx.motorDriver, 200);
        DBG_PRINTLN("  M" + String(m + 1) + ": " + String(result.speedAt50[m], 1) + " t/s (" +
                    String(result.linearityRatio[m], 3) + ")");
    }

    ctx.motorDriver.driveMotorsDirect(0, 0, 0);
    updateLoop(ctx.motorDriver, 500);

    // ========== Phase 4: Forward drive validation ==========
    DBG_PRINTLN("\n[Phase 4/5] Forward drive validation...");
    log.lastPhase = 4;
    log.timestampMs = millis();
    fastEEPROMPut(CAL_LOG_ADDR, log);

    // Use open-loop drive with direct heading correction (same as Phase 5).
    // PI + driveDegrees has a smooth ramp bug that prevents motors from spinning.
    ctx.motorDriver.setPIGains(0, 0, 0);
    ctx.compass.setPD(2.0f, 0.5f, 30.0f, 1.0f);

    ctx.compass.reset();

    constexpr uint32_t DRIVE_DURATION_MS = 3000;
    const uint32_t driveStart = millis();

    while (millis() - driveStart < DRIVE_DURATION_MS) {
        ctx.compass.update();
        const float rotation = ctx.compass.computeRotation(0);
        driveDirectDegrees(ctx.motorDriver, 0, 30, rotation);
        ctx.motorDriver.syncUpdateAllMotors();
        delay(10);
    }

    ctx.compass.update();
    result.headingDrift = ctx.compass.getOffset();
    log.phase4_headingDrift = result.headingDrift;

    ctx.motorDriver.driveDegrees(0, 0, 0);
    updateLoop(ctx.motorDriver, 500);

    DBG_PRINTLN("  Heading drift: " + String(result.headingDrift, 2) + " deg over 3s");

    // ========== Phase 5: Per-direction calibration ==========
    DBG_PRINTLN("\n[Phase 5/5] Direction calibration (12 directions, ~22s)...");

    // Disable PI for Phase 5: open-loop motor drive with direct heading correction.
    // PI + encoder_reset causes oscillation (the "rickety" stutter).
    ctx.motorDriver.setPIGains(0, 0, 0);

    // Boost compass heading correction: higher kp and maxRotation so the open-loop
    // rotation from motor imbalance doesn't overwhelm heading hold.
    ctx.compass.setPD(2.0f, 0.5f, 30.0f, 1.0f);

    log.lastPhase = 5;
    log.timestampMs = millis();
    fastEEPROMPut(CAL_LOG_ADDR, log);

    constexpr float    BASELINE_SCALE       = 40.0f;
    constexpr uint32_t DIR_SETTLE_MS        = 200;
    constexpr uint32_t DIR_MEASURE_MS       = 500;
    constexpr uint32_t DIR_STOP_MS          = 300;
    constexpr int      DIR_SAMPLE_PERIOD_MS = 50;
    constexpr int      DIR_DISCARD          = 2;
    constexpr float    RETURN_SCALE          = 40.0f;  // match forward speed for symmetric return
    constexpr uint32_t DIR_RETURN_MS         = 500;    // drive back toward center
    constexpr uint32_t DIR_RETURN_SETTLE_MS  = 300;    // settle after return
    constexpr int      DIR_TOTAL_SAMPLES    = DIR_MEASURE_MS / DIR_SAMPLE_PERIOD_MS;

    // Ensure compass is happy (still reset at start of Phase 4); reset again so the
    // target heading for Phase 5 is whatever we're pointing at right now.
    ctx.compass.update();
    ctx.compass.reset();

    // --- Pre-flight encoder sign check ---
    // At theta=0 (forward), driveRadians yields: m1 < 0, m2 ~ 0, m3 > 0.
    // If encoders disagree the whole sweep is garbage; don't save.
    log.lastDirection = 0xFE;  // 0xFE = in pre-flight
    log.timestampMs = millis();
    DBG_PRINTLN("  CalibrationLog size: " + String(sizeof(CalibrationLog)) +
                " bytes, writing to EEPROM addr " + String(CAL_LOG_ADDR) +
                ", end = " + String(CAL_LOG_ADDR + sizeof(CalibrationLog)));
    fastEEPROMPut(CAL_LOG_ADDR, log);
    DBG_PRINTLN("  EEPROM save OK, starting pre-flight drive...");
    for (uint8_t k = 0; k < 3; k++) encoder_reset(k);
    {
        const uint32_t checkStart = millis();
        while (millis() - checkStart < 500) {
            ctx.compass.update();
            const float rot = ctx.compass.computeRotation(0);
            driveDirectDegrees(ctx.motorDriver, 0, BASELINE_SCALE, rot);
            ctx.motorDriver.syncUpdateAllMotors();
            delay(10);
        }
        for (uint8_t k = 0; k < 3; k++) encoder_update_speed(k);
    }
    ctx.motorDriver.driveMotorsDirect(0, 0, 0);
    ctx.motorDriver.syncUpdateAllMotors();

    const float preM1 = encoder_get_speed(0);
    const float preM2 = encoder_get_speed(1);
    const float preM3 = encoder_get_speed(2);
    bool signsOk = (preM1 < 0) && (preM3 > 0) && (fabsf(preM2) < fabsf(preM1) * 0.5f);
    log.preFlightMotorSpeed[0] = preM1;
    log.preFlightMotorSpeed[1] = preM2;
    log.preFlightMotorSpeed[2] = preM3;
    log.phase5_signsOk = signsOk;

    // Save pre-flight results immediately so we can see them if accel bias crashes.
    log.lastDirection = 0xFC;  // 0xFC = pre-flight drive done, entering accel bias
    log.timestampMs = millis();
    fastEEPROMPut(CAL_LOG_ADDR, log);

    DBG_PRINTLN("  Pre-flight (theta=0): m1=" + String(preM1, 1) + " m2=" +
                String(preM2, 1) + " m3=" + String(preM3, 1) +
                (signsOk ? "  [signs OK]" : "  [ENCODER SIGN SUSPECT]"));

    // --- Pre-sweep accelerometer bias at rest ---
    // Accel cross-check disabled: DMA I2C reads after flash writes cause hard-faults
    // on this MCU. The encoder-based calibration is sufficient.
    float ax0 = 0, ay0 = 0, az0 = 0;
    const bool accelOk = false;
    DBG_PRINTLN("  Accel cross-check disabled (DMA/flash conflict)");
    log.accelBias[0] = ax0;
    log.accelBias[1] = ay0;
    log.accelBias[2] = az0;
    log.phase5_accelOk = accelOk;
    updateLoop(ctx.motorDriver, 200);  // small settle before direction loop

    // Per-direction accelerometer capture (body-frame ramp velocity, m/s)
    float accelRawAngle[NUM_DIRECTIONS] = {};
    float accelVelMag[NUM_DIRECTIONS]   = {};

    result.dirValid = signsOk;
    DBG_PRINTLN("  theta | actual | offset | speed% | scale");

    // Save pre-flight + bias results before entering the direction loop.
    log.lastDirection = 0xFD;  // 0xFD = pre-flight done, entering dir loop
    log.timestampMs = millis();
    fastEEPROMPut(CAL_LOG_ADDR, log);

    for (uint8_t i = 0; i < NUM_DIRECTIONS; i++) {
        const float thetaCmd = static_cast<float>(i) * (360.0f / NUM_DIRECTIONS);

        // Reset encoders, ramp up, hold heading. Integrate accel during ramp to get
        // a body-frame velocity estimate (gravity subtracted via pre-sweep bias).
        for (uint8_t k = 0; k < 3; k++) encoder_reset(k);

        float vxAcc = 0, vyAcc = 0;
        uint32_t lastAccMs = millis();
        const uint32_t settleStart = lastAccMs;
        while (millis() - settleStart < DIR_SETTLE_MS) {
            ctx.compass.update();
            const float rot = ctx.compass.computeRotation(0);
            driveDirectDegrees(ctx.motorDriver, thetaCmd, BASELINE_SCALE, rot);
            ctx.motorDriver.syncUpdateAllMotors();
            if (accelOk) {
                float ax, ay, az;
                if (ctx.accel.read(ax, ay, az)) {
                    const uint32_t now = millis();
                    const float dt = static_cast<float>(now - lastAccMs) * 0.001f;
                    lastAccMs = now;
                    vxAcc += (ax - ax0) * 9.81f * dt;   // g -> m/s^2
                    vyAcc += (ay - ay0) * 9.81f * dt;
                }
            }
            delay(10);
        }
        if (accelOk) {
            accelRawAngle[i] = Math::radiansToDegrees(atan2f(vyAcc, vxAcc));
            accelVelMag[i]   = sqrtf(vxAcc * vxAcc + vyAcc * vyAcc);
        }
        log.dirAccelRawDeg[i] = accelRawAngle[i];
        log.dirAccelVelMps[i] = accelVelMag[i];

        // Measurement window: signed-speed average over last (TOTAL - DISCARD) samples.
        float sums[3]   = {0, 0, 0};
        int   validCnt  = 0;
        for (int s = 0; s < DIR_TOTAL_SAMPLES; s++) {
            ctx.compass.update();
            const float rot = ctx.compass.computeRotation(0);
            driveDirectDegrees(ctx.motorDriver, thetaCmd, BASELINE_SCALE, rot);
            ctx.motorDriver.syncUpdateAllMotors();
            for (uint8_t k = 0; k < 3; k++) encoder_update_speed(k);
            if (s >= DIR_DISCARD) {
                for (uint8_t k = 0; k < 3; k++)
                    sums[k] += encoder_get_speed(k);    // SIGNED
                validCnt++;
            }
            delay(DIR_SAMPLE_PERIOD_MS);
        }

        // Capture raw encoder ticks for diagnostics.
        for (uint8_t k = 0; k < 3; k++)
            log.dirRawTicks[i][k] = encoder_get_ticks(k);

        // Stop and coast before returning to center.
        ctx.motorDriver.driveMotorsDirect(0, 0, 0);
        updateLoop(ctx.motorDriver, DIR_STOP_MS);

        // Drive back toward center (opposite direction) to stay within ~15cm.
        // Use driveDegrees with 0 rotation to avoid compass dependency during return.
        {
            const float returnTheta = thetaCmd + 180.0f;
            const uint32_t retStart = millis();
            while (millis() - retStart < DIR_RETURN_MS) {
                driveDirectDegrees(ctx.motorDriver, returnTheta, RETURN_SCALE, 0);
                ctx.motorDriver.syncUpdateAllMotors();
                delay(10);
            }
            ctx.motorDriver.driveMotorsDirect(0, 0, 0);
            updateLoop(ctx.motorDriver, DIR_RETURN_SETTLE_MS);
        }

        // Convert to signed PWM% using per-motor max.
        float mPct[3];
        for (uint8_t k = 0; k < 3; k++) {
            const float avg = sums[k] / static_cast<float>(validCnt);
            mPct[k] = 100.0f * avg / result.maxTicksPerSec[k];
        }

        // Forward kinematics: recover v·s and rotation (rotation term cancels in vy/vx).
        const float vyS = (mPct[0] - 2.0f * mPct[1] + mPct[2]) / 3.0f;
        const float vxS = (mPct[2] - mPct[0]) / (2.0f * SIN_60);

        float angleRad, magPct;
        cordic_atan2_mod(vyS, vxS, &angleRad, &magPct);
        const float actualDeg = Math::radiansToDegrees(angleRad);

        // Offset wrapped to [-180, 180]
        float offsetDeg = thetaCmd - actualDeg;
        offsetDeg = Math::wrapDegrees(offsetDeg + 180.0f) - 180.0f;

        // Scale multiplier: output * sMul -> actual speed ~= commanded.
        float sMul = (magPct > 1.0f) ? (BASELINE_SCALE / magPct) : 3.0f;
        if (sMul < 0.3f) sMul = 0.3f;
        if (sMul > 3.0f) sMul = 3.0f;

        // Saturation guard (post-hoc): if any wheel ran too hot, cancellation breaks.
        const float maxAbs = fmaxf(fmaxf(fabsf(mPct[0]), fabsf(mPct[1])), fabsf(mPct[2]));
        const bool  satOk  = (maxAbs < 95.0f);

        result.dirScale[i]     = sMul;
        result.dirOffsetDeg[i] = offsetDeg;

        // In-range validation
        const bool scaleOk  = (sMul >= 0.6f && sMul <= 1.6f);
        const bool offsetOk = (fabsf(offsetDeg) <= 25.0f);
        if (!satOk || !scaleOk || !offsetOk) result.dirValid = false;

        for (uint8_t k = 0; k < 3; k++) log.dirMotorPct[i][k] = mPct[k];
        log.dirActualDeg[i] = actualDeg;
        log.dirOffsetDeg[i] = offsetDeg;
        log.dirSpeedPct[i]  = magPct;
        log.dirScale[i]     = sMul;
        log.dirFlags[i]     = (satOk ? 0x01 : 0) | (scaleOk ? 0x02 : 0) | (offsetOk ? 0x04 : 0);

        DBG_PRINTLN("  " + String(thetaCmd, 0) + "  | " +
                    String(actualDeg, 1) + " | " +
                    String(offsetDeg, 2) + " | " +
                    String(magPct, 1) + " | " +
                    String(sMul, 3) +
                    (satOk ? "" : "  [SAT]") +
                    (scaleOk ? "" : "  [SCALE-OOR]") +
                    (offsetOk ? "" : "  [OFFSET-OOR]"));

        // Save progress after each direction so a crash preserves partial results.
        log.lastDirection = i;
        log.timestampMs   = millis();
        fastEEPROMPut(CAL_LOG_ADDR, log);

        // Pause between directions so the robot can be repositioned if needed.
        delay(2000);
        // Reset compass heading for the next direction.
        ctx.compass.update();
        ctx.compass.reset();
    }

    ctx.motorDriver.driveMotorsDirect(0, 0, 0);
    updateLoop(ctx.motorDriver, DIR_STOP_MS);

    // --- Accelerometer cross-check ---
    // Fit the accel→robot frame rotation from the directions where motion was detected,
    // then compare accel-derived direction-offset to the encoder-derived offset.
    if (accelOk) {
        float sumSin = 0, sumCos = 0;
        int   frameN = 0;
        constexpr float MIN_VEL_MPS = 0.05f;
        for (uint8_t i = 0; i < NUM_DIRECTIONS; i++) {
            if (accelVelMag[i] < MIN_VEL_MPS) continue;
            const float thetaCmd = static_cast<float>(i) * (360.0f / NUM_DIRECTIONS);
            const float delta    = accelRawAngle[i] - thetaCmd;
            const float deltaRad = Math::degreesToRadians(delta);
            sumSin += sinf(deltaRad);
            sumCos += cosf(deltaRad);
            frameN++;
        }
        if (frameN >= NUM_DIRECTIONS / 2) {
            const float accelFrameDeg = Math::radiansToDegrees(atan2f(sumSin, sumCos));
            log.accelFrameDeg        = accelFrameDeg;
            log.phase5_accelFrameOk  = true;
            DBG_PRINTLN("  Accel frame offset (accel-X vs robot-forward): " +
                        String(accelFrameDeg, 1) + " deg (" + String(frameN) +
                        "/" + String(NUM_DIRECTIONS) + " dirs with motion)");
            DBG_PRINTLN("  Accel xcheck: theta | accel_dir | accel_off | enc_off | |v| m/s");
            for (uint8_t i = 0; i < NUM_DIRECTIONS; i++) {
                const float thetaCmd = static_cast<float>(i) * (360.0f / NUM_DIRECTIONS);
                const float accelRel = Math::wrapDegrees(accelRawAngle[i] - accelFrameDeg);
                float accelOffset    = thetaCmd - accelRel;
                accelOffset          = Math::wrapDegrees(accelOffset + 180.0f) - 180.0f;
                const float disagreement = accelOffset - result.dirOffsetDeg[i];
                const bool  motionOk = accelVelMag[i] >= MIN_VEL_MPS;
                const bool  agreeOk  = motionOk && fabsf(disagreement) < 35.0f;
                if (!agreeOk) result.dirValid = false;

                log.dirAccelOffsetDeg[i] = accelOffset;
                if (motionOk) log.dirFlags[i] |= 0x08;
                if (agreeOk)  log.dirFlags[i] |= 0x10;

                DBG_PRINTLN("  " + String(thetaCmd, 0) + "  | " +
                            String(accelRel, 1) + " | " +
                            String(accelOffset, 2) + " | " +
                            String(result.dirOffsetDeg[i], 2) + " | " +
                            String(accelVelMag[i], 3) +
                            (motionOk ? "" : "  [NO-MOTION]") +
                            (agreeOk  ? "" : "  [DISAGREE]"));
            }
        } else {
            DBG_PRINTLN("  Accel frame: insufficient motion (" + String(frameN) +
                        "/" + String(NUM_DIRECTIONS) + " dirs); cross-check skipped");
        }
    }

    DBG_PRINTLN(result.dirValid
                ? "  Direction cal: VALID"
                : "  Direction cal: SUSPECT - not saved");

    // ========== Report & Save ==========
    result.valid = true;
    for (uint8_t i = 0; i < 3; i++) {
        if (result.maxTicksPerSec[i] < 100.0f || result.maxTicksPerSec[i] > 10000.0f)
            result.valid = false;
        if (result.linearityRatio[i] < 0.5f || result.linearityRatio[i] > 1.5f)
            result.valid = false;
    }

    DBG_PRINTLN("\n===== CALIBRATION RESULTS =====");
    DBG_PRINTLN(result.valid ? "STATUS: VALID" : "STATUS: SUSPECT - review values");
    DBG_PRINTLN("");

    for (uint8_t i = 0; i < 3; i++) {
        DBG_PRINTLN("Motor " + String(i + 1) + ":  max=" +
                     String(result.maxTicksPerSec[i], 1) + " t/s  linearity=" +
                     String(result.linearityRatio[i], 3));
    }

    if (result.valid) {
        StoredCalibration cal = {};
        EEPROM.get(0, cal);          // preserve any existing compass cal
        const bool keepMag = (cal.magic == CALIBRATION_MAGIC) && cal.magValid;
        if (!keepMag) {
            for (uint8_t i = 0; i < 3; i++) { cal.magOffset[i] = 0.0f; cal.magScale[i] = 1.0f; }
            cal.magValid = false;
        }
        cal.magic = CALIBRATION_MAGIC;
        for (uint8_t i = 0; i < 3; i++) {
            cal.maxTicksPerSec[i] = result.maxTicksPerSec[i];
            cal.linearityRatio[i] = result.linearityRatio[i];
        }
        for (uint8_t i = 0; i < NUM_DIRECTIONS; i++) {
            cal.dirScale[i]     = result.dirValid ? result.dirScale[i]     : 1.0f;
            cal.dirOffsetDeg[i] = result.dirValid ? result.dirOffsetDeg[i] : 0.0f;
        }
        cal.dirValid = result.dirValid;
        fastEEPROMPut(0, cal);
        DBG_PRINTLN(result.dirValid
            ? "\nCalibration saved to EEPROM (motor + direction)."
            : "\nCalibration saved to EEPROM (motor only; direction cal skipped).");
    } else {
        DBG_PRINTLN("\nNot saved — results out of range.");
    }

    DBG_PRINTLN("");
    DBG_PRINTLN("// Or apply manually:");
    for (uint8_t i = 0; i < 3; i++) {
        DBG_PRINTLN("motorDriver.setMaxTicksPerSec(" + String(i) + ", " +
                     String(result.maxTicksPerSec[i], 1) + ");");
    }
    DBG_PRINTLN("================================");

    // Always persist the diagnostic log, regardless of motor/direction validity,
    // so a calibration run without a laptop can still be reviewed later.
    log.motorValid   = result.valid;
    log.dirValid     = result.dirValid;
    log.completed    = true;
    log.timestampMs  = millis();
    fastEEPROMPut(CAL_LOG_ADDR, log);
    DBG_PRINTLN("Calibration log saved to EEPROM (address " + String(CAL_LOG_ADDR) + ").");

    ctx.motorDriver.driveMotorsDirect(0, 0, 0);

    while (true) { delay(1000); }
}

// ============================================================================
// Calibration dump: reads both the applied StoredCalibration at address 0 and
// the always-saved CalibrationLog at CAL_LOG_ADDR, prints everything to serial.
// Use by setting RUN_TEST to testCalibrationDump in main.cpp.
// ============================================================================
void testCalibrationDump(const TestContext& /*ctx*/) {
    DBG_PRINTLN("=== Calibration Dump ===");

    // --- Applied StoredCalibration ---
    {
        StoredCalibration cal = {};
        EEPROM.get(0, cal);
        DBG_PRINTLN("\n[StoredCalibration @ 0]");
        DBG_PRINT("  magic: 0x"); DBG_PRINTLN(cal.magic, HEX);
        if (cal.magic != CALIBRATION_MAGIC) {
            DBG_PRINTLN("  (magic mismatch — no valid applied calibration)");
        } else {
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
    }

    // --- Always-saved CalibrationLog ---
    {
        CalibrationLog log = {};
        EEPROM.get(CAL_LOG_ADDR, log);
        DBG_PRINTLN("\n[CalibrationLog @ " + String(CAL_LOG_ADDR) + "]");
        DBG_PRINT("  magic: 0x"); DBG_PRINTLN(log.magic, HEX);
        if (log.magic != CAL_LOG_MAGIC) {
            DBG_PRINTLN("  (magic mismatch — no log saved)");
        } else {
            DBG_PRINTLN("  timestampMs: " + String(log.timestampMs));
            {
                String dirStr;
                if      (log.lastDirection == 0xFF) dirStr = "none";
                else if (log.lastDirection == 0xFE) dirStr = "in pre-flight drive";
                else if (log.lastDirection == 0xFD) dirStr = "entering dir loop";
                else if (log.lastDirection == 0xFC) dirStr = "pre-flight done, in accel bias";
                else dirStr = String(log.lastDirection * 30) + "deg (#" + String(log.lastDirection) + ")";
                DBG_PRINTLN("  completed=" + String(log.completed ? "Y" : "N") +
                            "  lastPhase=" + String(log.lastPhase) +
                            "  lastDir=" + dirStr);
            }
            if (!log.completed) {
                String where;
                if (log.lastPhase < 5) {
                    where = " in phase " + String(log.lastPhase);
                } else if (log.lastDirection == 0xFF) {
                    where = " phase 5 (before pre-flight)";
                } else if (log.lastDirection == 0xFE) {
                    where = " phase 5 (during pre-flight drive)";
                } else if (log.lastDirection == 0xFC) {
                    where = " phase 5 (during accel bias)";
                } else if (log.lastDirection == 0xFD) {
                    where = " phase 5 (entering direction loop)";
                } else {
                    where = " phase 5 (after dir " + String(log.lastDirection * 30) + "deg, before " +
                            String(((log.lastDirection + 1) % 12) * 30) + "deg)";
                }
                DBG_PRINTLN("  *** CRASHED" + where + " ***");
            }
            DBG_PRINTLN("  motorValid=" + String(log.motorValid ? "Y" : "N") +
                        "  dirValid=" + String(log.dirValid ? "Y" : "N"));

            DBG_PRINTLN("  [Phase 1] staticOk=" + String(log.phase1_staticOk ? "Y" : "N") +
                        "  noise=" + String(log.staticNoise[0], 1) + "/" +
                        String(log.staticNoise[1], 1) + "/" +
                        String(log.staticNoise[2], 1) + " t/s");

            DBG_PRINTLN("  [Phase 2] maxTicksPerSec=" +
                        String(log.maxTicksPerSec[0], 1) + "/" +
                        String(log.maxTicksPerSec[1], 1) + "/" +
                        String(log.maxTicksPerSec[2], 1));

            DBG_PRINTLN("  [Phase 3] speedAt50=" +
                        String(log.speedAt50[0], 1) + "/" +
                        String(log.speedAt50[1], 1) + "/" +
                        String(log.speedAt50[2], 1) +
                        "  lin=" +
                        String(log.linearityRatio[0], 3) + "/" +
                        String(log.linearityRatio[1], 3) + "/" +
                        String(log.linearityRatio[2], 3));

            DBG_PRINTLN("  [Phase 4] headingDrift=" + String(log.phase4_headingDrift, 2) + " deg");

            DBG_PRINTLN("  [Phase 5] signsOk=" + String(log.phase5_signsOk ? "Y" : "N") +
                        "  preFlight m1/m2/m3=" +
                        String(log.preFlightMotorSpeed[0], 1) + "/" +
                        String(log.preFlightMotorSpeed[1], 1) + "/" +
                        String(log.preFlightMotorSpeed[2], 1));

            DBG_PRINTLN("  [Phase 5] accelOk=" + String(log.phase5_accelOk ? "Y" : "N") +
                        "  bias=" + String(log.accelBias[0], 3) + "/" +
                        String(log.accelBias[1], 3) + "/" +
                        String(log.accelBias[2], 3) + "g");

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
                            ((f & 0x01) ? "" : "[SAT]") +
                            ((f & 0x02) ? "" : "[SCALE-OOR]") +
                            ((f & 0x04) ? "" : "[OFFSET-OOR]") +
                            ((f & 0x08) ? "" : "[NO-MOTION]") +
                            ((f & 0x10) ? "" : "[DISAGREE]"));
            }

            DBG_PRINTLN("  [Phase 5] theta | rawTicks m1 | m2 | m3");
            for (uint8_t i = 0; i < NUM_DIRECTIONS; i++) {
                DBG_PRINTLN("    " + String(i * 30) + " | " +
                            String(log.dirRawTicks[i][0]) + " | " +
                            String(log.dirRawTicks[i][1]) + " | " +
                            String(log.dirRawTicks[i][2]));
            }

            DBG_PRINTLN("  [Phase 5] accelFrameOk=" + String(log.phase5_accelFrameOk ? "Y" : "N") +
                        "  accelFrameDeg=" + String(log.accelFrameDeg, 1));
            DBG_PRINTLN("  [Phase 5] theta | accel_raw_deg | accel_off | |v| m/s");
            for (uint8_t i = 0; i < NUM_DIRECTIONS; i++) {
                DBG_PRINTLN("    " + String(i * 30) + " | " +
                            String(log.dirAccelRawDeg[i], 1) + " | " +
                            String(log.dirAccelOffsetDeg[i], 2) + " | " +
                            String(log.dirAccelVelMps[i], 3));
            }
        }
    }

    DBG_PRINTLN("\n=== end dump ===");
    while (true) { delay(1000); }
}
