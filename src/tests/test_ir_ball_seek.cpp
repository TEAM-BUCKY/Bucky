#include "tests.h"
#include "debug.h"

#include <Arduino.h>
#include <cmath>

#include "control/ball/IRBallProcessor.h"
#include "control/pos/SelfLocalizationEKF.h"
#include "field/DigitalField.h"
#include "helpers/Math.h"
#include "io/cordic/cordic.h"
#include "robot/RobotBrain.h"
#include "robot/StuckDetector.h"
#include "sensors/IRSensor.h"
#include "strategy/StrategyFSM.h"

// Hardware integration test:
//   * Full RobotBrain pipeline drives toward the IR ball (chase → drive_to_point).
//   * Rotation overridden with compass PD holding the start heading — the
//     "hold starting heading" strategy layered on top of production drive.
//   * Latches motors to zero when ball-in-cage geometry holds for 3 ticks.
//   * Safety kill: if |heading offset| > 45° for 2 s, latch-stop.
//
// Assumes setupEnvironment() in main.cpp ran first (IR ring calibrated,
// compass/sonar startRead live, motor calibration loaded from EEPROM).

namespace
{
constexpr uint8_t kBoardIR         = 2;
constexpr int     kCaptureStreakN  = 3;
constexpr float   kCageDistM       = 0.15f;
constexpr float   kCageLateralM    = 0.05f;
constexpr float   kKillOffsetDeg   = 45.0f;
constexpr uint32_t kKillDurationMs = 2000u;
constexpr uint32_t kReportMs       = 200u;

const char* stateName(const GameState_t s)
{
    switch (s) {
        case STATE_FIND_BALL:       return "FIND_BALL";
        case STATE_CHASE_BALL:      return "CHASE_BALL";
        case STATE_DRIBBLE:         return "DRIBBLE";
        case STATE_SHOOT:           return "SHOOT";
        case STATE_DEFEND:          return "DEFEND";
        case STATE_INTERCEPT:       return "INTERCEPT";
        case STATE_RETURN_POSITION: return "RETURN_POS";
        case STATE_LINE_AVOID:      return "LINE_AVOID";
        case STATE_STUCK_RECOVERY:  return "STUCK";
    }
    return "?";
}
} // namespace

void testIRBallSeek(const TestContext& ctx)
{
    DBG_PRINTLN("=== IR Ball Seek (hold-heading + capture-latch) ===");
    DBG_PRINTLN("Place active IR ball in front; robot will strafe toward it");
    DBG_PRINTLN("and stop when ball enters the front cage.");

    // Anchor hold-heading PD at the current yaw. After reset(), getOffset()
    // reads 0 while the robot sits at its current heading; computeRotation(0)
    // then pulls back to that anchor.
    ctx.compass.reset();
    ctx.compass.setPD(0.8f, 0.25f, 12.0f, 2.0f);

    IRBallProcessor ballTracker;
    ballTracker.setChannelOffset(ir_get_channel_offset(kBoardIR));

    RobotBrain brain;
    brain.reset(0.0f, -0.2f, 0.0f);

    StuckDetector stuck;
    stuck.reset(HAL_GetTick());

    uint32_t lastIrSeq = ir_get_frame_sequence(kBoardIR);
    VectorXY lastDrive = {0.0f, 0.0f};
    float    lastRotation = 0.0f;

    int      captureStreak    = 0;
    bool     captured         = false;
    uint32_t badHeadingStartMs = 0;
    bool     killTriggered    = false;

    uint32_t irFramesThisReport      = 0;
    uint32_t compassFusesThisReport  = 0;
    uint32_t sonarFusesThisReport    = 0;
    uint32_t reportAt                = HAL_GetTick() + kReportMs;

    // Raw bearing from the most recent valid IR frame in this report window.
    // Kept separately from the IMM-smoothed `brg` column so we can see what
    // IRBallProcessor reports *before* any world/body-frame math touches it.
    float lastRawBearingDeg = 0.0f;
    bool  haveRawBearing    = false;

    // Snapshots captured at the end of the previous report window so we can
    // print per-window deltas (offset rate, theta rate, self-loc drift).
    float lastReportOffsetDeg = 0.0f;
    float lastReportThetaDeg  = 0.0f;
    float lastReportSelfX     = 0.0f;
    float lastReportSelfY     = 0.0f;
    bool  haveLastReport      = false;

    // Set lastTick immediately before the while-loop so dt starts near zero
    // rather than reflecting any setup delays.
    uint32_t lastTick = HAL_GetTick();

    // Throttle compass fusion to the LIS2MDL's 100 Hz ODR. See main.cpp's
    // identical gate: without it, the ~180 µs DMA cycle fuses the same stale
    // register into the EKF thousands of times per physical sample, locking
    // theta to compass and letting a single EMI hit yank self-loc.
    uint32_t lastCompassFuseMs = 0;

    while (true) {
        const uint32_t now = HAL_GetTick();
        const float dt = static_cast<float>(now - lastTick) * 0.001f;
        lastTick = now;

        // --- Gather sensors (mirror main.cpp sensor-gather block) ---
        BrainSensors sensors = {};
        sensors.dt = dt;
        sensors.now_ms = now;

        if (ctx.compass.isReadComplete()) {
            ctx.compass.processRead();
            if (now - lastCompassFuseMs >= 10) {
                sensors.compass_ready = true;
                sensors.compass_heading_rad = Math::degreesToRadians(ctx.compass.getOffset());
                lastCompassFuseMs = now;
                ++compassFusesThisReport;
            }
            ctx.compass.startRead();
        }

        if (ctx.sonar.isReadComplete()) {
            const SonarReading r = ctx.sonar.processRead();
            sensors.sonar_ready = true;
            for (uint8_t i = 0; i < SONAR_COUNT; ++i)
                sensors.sonar_distance_m[i] = r.valid[i] ? r.distance[i] * 0.01f : -1.0f;
            ctx.sonar.startRead();
            ++sonarFusesThisReport;
        }

        if (ir_has_new_frame(kBoardIR, lastIrSeq)) {
            lastIrSeq = ir_get_frame_sequence(kBoardIR);
            ++irFramesThisReport;
            const uint16_t* raw = ir_get_buffer(kBoardIR);
            const uint32_t sensorCount = ir_get_sensor_count(kBoardIR);

            if (const IRBallObservation observation = ballTracker.process(raw, sensorCount);
                observation.valid && observation.confidence >= 0.05f) {
                if (!ballTracker.isS0Locked()) {
                    ballTracker.lockS0AtSensor(observation.peakSensor);
                } else {
                    lastRawBearingDeg = observation.bearingDeg;
                    haveRawBearing    = true;
                    const SelfLocState selfState = selfloc_get_state(&brain.selfLoc());
                    const float alpha = selfState.theta
                                      + Math::degreesToRadians(observation.bearingDeg);
                    const float rangeM = observation.rangeCm * 0.01f;
                    sensors.ball_visible = true;
                    float sA, cA;
                    cordic_sin_cos(alpha, &sA, &cA);
                    sensors.ball_field_x = selfState.x + rangeM * cA;
                    sensors.ball_field_y = selfState.y + rangeM * sA;
                    sensors.ball_range_m = rangeM;
                }
            }
        }
        sensors.ir_s0_locked = ballTracker.isS0Locked();

        // Possession hint against the PREVIOUS tick's fused estimate — same
        // as main.cpp:183-191.
        sensors.possession_hint = BALL_MODE_FREE;
        {
            const DigitalField& prev = brain.field();
            if (prev.ball.visible
                && prev.ball.distM < kCageDistM
                && prev.ball.byBody > 0.0f
                && fabsf(prev.ball.bxBody) < kCageLateralM)
                sensors.possession_hint = BALL_MODE_FRIENDLY;
        }

        sensors.stuck_detected = stuck.update(now, lastDrive, lastRotation);

        // --- Run brain ---
        const BrainCommand cmd = brain.tick(sensors);
        const DigitalField& f = brain.field();

        // --- Capture latch: gate on raw-this-tick visibility AND fused
        //     geometry so an IMM prediction during IR dropout can't latch us.
        const bool ballInCage =
            sensors.ball_visible && f.ball.visible
            && f.ball.distM < kCageDistM
            && f.ball.byBody > 0.0f
            && fabsf(f.ball.bxBody) < kCageLateralM;

        if (!captured) {
            if (ballInCage) {
                if (++captureStreak >= kCaptureStreakN) {
                    captured = true;
                    DBG_PRINT("CAPTURED @ t=");
                    DBG_PRINT(now);
                    DBG_PRINT(" ms  distM=");
                    DBG_PRINT(f.ball.distM, 3);
                    DBG_PRINT("  bxBody=");
                    DBG_PRINT(f.ball.bxBody, 3);
                    DBG_PRINT("  byBody=");
                    DBG_PRINT(f.ball.byBody, 3);
                    DBG_PRINT("  headingOff=");
                    DBG_PRINT(ctx.compass.getOffset(), 1);
                    DBG_PRINTLN();
                }
            } else {
                captureStreak = 0;
            }
        }

        // --- Safety kill switch on sustained heading error ---
        const float offsetDeg = ctx.compass.getOffset();
        if (fabsf(offsetDeg) > kKillOffsetDeg) {
            if (badHeadingStartMs == 0) {
                badHeadingStartMs = now;
            } else if (!killTriggered && (now - badHeadingStartMs) > kKillDurationMs) {
                killTriggered = true;
                captured = true;  // reuse latch path
                DBG_PRINT("KILL:heading offset=");
                DBG_PRINT(offsetDeg, 1);
                DBG_PRINT(" deg sustained > ");
                DBG_PRINT(kKillDurationMs);
                DBG_PRINTLN(" ms — stopping.");
            }
        } else {
            badHeadingStartMs = 0;
        }

        // --- Pick final drive/rotation (override FSM rotation with
        //     compass-held yaw; zero everything on latch). ---
        VectorXY drive;
        float    rotation;
        if (captured) {
            drive = {0.0f, 0.0f};
            rotation = 0.0f;
        } else {
            drive = cmd.drive;
            rotation = ctx.compass.computeRotation(0.0f);
        }

        ctx.motorDriver.driveVector(drive, rotation);
        ctx.motorDriver.updateAllMotors();

        lastDrive = drive;
        lastRotation = rotation;

        // --- Periodic telemetry ---
        if (now >= reportAt) {
            reportAt = now + kReportMs;

            const SelfLocState self = selfloc_get_state(&brain.selfLoc());
            const float headingDeg  = ctx.compass.getHeading();
            const float thetaDeg    = Math::radiansToDegrees(self.theta);

            // Per-window deltas: if the compass offset is winding up while
            // the commanded rotation sits at zero (or has the wrong sign),
            // we can see it here instead of inferring it between reports.
            const float dOffDeg   = haveLastReport ? (offsetDeg - lastReportOffsetDeg) : 0.0f;
            const float dThetaDeg = haveLastReport ? (thetaDeg  - lastReportThetaDeg)  : 0.0f;
            const float dSelfX    = haveLastReport ? (self.x - lastReportSelfX) : 0.0f;
            const float dSelfY    = haveLastReport ? (self.y - lastReportSelfY) : 0.0f;

            // Ball bearing in body frame from fused estimate. Atan2(bx, by)
            // because byBody is the forward axis, bxBody is lateral.
            const float ballBearingDeg = f.ball.visible
                ? Math::radiansToDegrees(atan2f(f.ball.bxBody, f.ball.byBody))
                : 0.0f;

            DBG_PRINT(captured ? "[LATCH] " : "[SEEK]  ");
            DBG_PRINT("state=");
            DBG_PRINT(stateName(cmd.state));
            DBG_PRINT(" vis=");
            DBG_PRINT(static_cast<int>(f.ball.visible));
            DBG_PRINT(" dist=");
            DBG_PRINT(f.ball.distM, 3);
            DBG_PRINT(" bx=");
            DBG_PRINT(f.ball.bxBody, 3);
            DBG_PRINT(" by=");
            DBG_PRINT(f.ball.byBody, 3);
            DBG_PRINT(" brg=");
            DBG_PRINT(ballBearingDeg, 1);
            DBG_PRINT(" irBrg=");
            if (haveRawBearing) DBG_PRINT(lastRawBearingDeg, 1);
            else                DBG_PRINT("--");
            DBG_PRINT(" hdg=");
            DBG_PRINT(headingDeg, 1);
            DBG_PRINT(" off=");
            DBG_PRINT(offsetDeg, 1);
            DBG_PRINT(" dOff=");
            DBG_PRINT(dOffDeg, 1);
            DBG_PRINT(" theta=");
            DBG_PRINT(thetaDeg, 1);
            DBG_PRINT(" dTheta=");
            DBG_PRINT(dThetaDeg, 1);
            DBG_PRINT(" pos=(");
            DBG_PRINT(self.x, 3);
            DBG_PRINT(",");
            DBG_PRINT(self.y, 3);
            DBG_PRINT(") dPos=(");
            DBG_PRINT(dSelfX, 3);
            DBG_PRINT(",");
            DBG_PRINT(dSelfY, 3);
            DBG_PRINT(") drv=(");
            DBG_PRINT(drive.x, 1);
            DBG_PRINT(",");
            DBG_PRINT(drive.y, 1);
            DBG_PRINT(") rotFSM=");
            DBG_PRINT(cmd.rotation, 1);
            DBG_PRINT(" rotCmd=");
            DBG_PRINT(rotation, 1);
            DBG_PRINT(" streak=");
            DBG_PRINT(captureStreak);
            DBG_PRINT(" irFr=");
            DBG_PRINT(irFramesThisReport);
            DBG_PRINT(" cmpF=");
            DBG_PRINT(compassFusesThisReport);
            DBG_PRINT(" snF=");
            DBG_PRINTLN(sonarFusesThisReport);
            irFramesThisReport      = 0;
            compassFusesThisReport  = 0;
            sonarFusesThisReport    = 0;

            lastReportOffsetDeg = offsetDeg;
            lastReportThetaDeg  = thetaDeg;
            lastReportSelfX     = self.x;
            lastReportSelfY     = self.y;
            haveLastReport      = true;
        }
    }
}
