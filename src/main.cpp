#include <Arduino.h>
#include <cmath>

#include "debug.h"
#include "motor/MotorDriver.h"
#include "control/ball/IRBallProcessor.h"
#include <control/pos/SelfLocalizationEKF.h>
#include <field/DigitalField.h>
#include <io/i2c/I2CDMA.h>
#include <sensors/pos/Compass.h>
#include <sensors/pos/Sonar.h>
#include <io/cordic/cordic.h>
#include <sensors/IRSensor.h>
#include <strategy/StrategyFSM.h>
#include <tests/tests.h>

#include "robot/RobotBrain.h"
#include "robot/SpinDetector.h"
#include "robot/StuckDetector.h"

// Set to run a test instead of the main loop. testADCRaw runs before
// setupEnvironment for clean isolation; all others run after.
// #define RUN_TEST testDriveForward
// #define RUN_TEST_EARLY  // only for testADCRaw (runs before setupEnvironment)

MotorPin m1 = {PA8, PA9};
MotorPin m2 = {PB12, PB13};
MotorPin m3 = {PC7, PC6};

MotorDriver motorDriver(m1, m2, m3);
I2CDMABus i2c1;
Compass compass;
Accelerometer accel;
Sonar sonar;

I2C_DMA_RX_HANDLER(1, 6, i2c1)

void setupEnvironment() {
    Serial.begin(115200);
    cordic_init();

    i2c_dma_init<I2CFrequency::FMP_3M4>(&i2c1, I2C1,
                  GPIOB, 9, 4,    // SDA: PB9  AF4
                  GPIOA, 15, 4,   // SCL: PA15 AF4
                  DMA1, DMA1_Channel6, DMAMUX1_Channel5,
                  DMAMUX_REQ_I2C1_RX, DMA1_Channel6_IRQn);

    compass.begin(i2c1);
    accel.begin(i2c1);

    analogReadResolution(12);

    constexpr EncoderPins encoderPins[3] = {
        {PA7, PA6},   // M1
        {PB2, PB1},   // M2
        {PA5, PA2},   // M3
    };
    motorDriver.init(MIN_SPEED, MAX_SPEED, encoderPins);


    constexpr SonarPins sonarPins = {.trigPin = PB11, .echoPins = {PA10, PC10, PC11, PC12}};
    sonar.begin(sonarPins);

    ir_sensor_init();

    // Detect the free-running mux-counter rotation so sensor indices line up
    // with physical positions. Must run after ir_sensor_init (DMA must be
    // streaming) and before anything reads the IR ring. Assumes the field is
    // clear of active IR sources at boot.
    const uint8_t irOffset = ir_calibrate_channels(2);
    DBG_PRINT("IR Board 2 channel offset: "); DBG_PRINTLN(irOffset);

    while (!compass.tick()) {}

    // if (loadCalibration(motorDriver, compass)) {
    //     DBG_PRINTLN("Calibration loaded from EEPROM.");
    // } else {
    //     DBG_PRINTLN("No valid calibration in EEPROM; using defaults.");
    // }

    // Anchor startHeading to a calibrated reading. The boot-time sampling
    // in compass.tick() runs before loadCalibration(), so it cannot set a
    // valid startHeading on its own.
    if (compass.isReady()) {
        compass.update();
        compass.reset();
        compass.startRead();
    }
    sonar.startRead();
}

// Board 1 is disabled in IRSensor.h; the active IR ring is Board 2.
constexpr uint8_t boardIR = 2;

[[noreturn]] int main() {
    init();
//
// #ifdef RUN_TEST
//     delay(5000);
// #endif

#if defined(RUN_TEST) && defined(RUN_TEST_EARLY)
    // Early tests run BEFORE setupEnvironment to avoid DMA/timer/OPAMP contamination.
    Serial.begin(115200);
    { constexpr TestContext ctx = {motorDriver, compass, accel, sonar, i2c1}; RUN_TEST(ctx); }
#endif

    setupEnvironment();

#if defined(RUN_TEST) && !defined(RUN_TEST_EARLY)
    constexpr TestContext ctx = {motorDriver, compass, accel, sonar, i2c1};

    RUN_TEST(ctx);
#else

    RobotBrain brain;
    IRBallProcessor ballTracker;
    ballTracker.setChannelOffset(ir_get_channel_offset(boardIR));

    brain.reset(0.0f, -0.2f, 0.0f);

    // Hold-heading PD gains — match testIRBallSeek. Softer than the Compass
    // defaults (kp=0.4, kd=0.3, maxRot=25, dead=3) so the rotation correction
    // doesn't saturate on small transient compass noise and compete with the
    // FSM's translation commands. setupEnvironment already calls compass.reset()
    // to anchor the startHeading at boot.
    compass.setPD(0.8f, 0.25f, 12.0f, 2.0f);

    StuckDetector stuck;
    stuck.reset(HAL_GetTick());

    // Uncontrolled-spin watchdog. Anchor lastHeading at the compass's current
    // offset so the first dt has ~zero delta — prevents a phantom trip on
    // the first tick after boot.
    SpinDetector spin;
    spin.reset(HAL_GetTick(), Math::degreesToRadians(compass.getOffset()));

    uint32_t lastTick = HAL_GetTick();
    uint32_t lastIrSeq = ir_get_frame_sequence(boardIR);
    VectorXY lastDrive = {0.0f, 0.0f};
    float lastRotation = 0.0f;

    // Throttle compass fusion to match the LIS2MDL's 100 Hz ODR. The I²C DMA
    // cycle finishes in ~180 µs, so without this gate the same stale register
    // is fused into the EKF hundreds of times per physical sample — effective
    // R shrinks ~1000×, the filter locks to compass, and a single EMI-disturbed
    // reading yanks theta (and, via the sonar Jacobian, pos).
    uint32_t lastCompassFuseMs = 0;

    while (true) {
        const uint32_t now = HAL_GetTick();
        const float dt = static_cast<float>(now - lastTick) * 0.001f;
        lastTick = now;

        // --- Gather sensor measurements ---
        BrainSensors sensors = {};
        sensors.dt = dt;
        sensors.now_ms = now;

        // Compass. Async pattern: after the DMA transfer completes we must
        // call processRead() to parse rx_buf into `heading` — without it,
        // getOffset() returns the same value forever and the EKF thinks the
        // robot's yaw never changes. Gate on isReady() so a failed WHO_AM_I
        // handshake doesn't feed noise from an unstarted rx_buf into the EKF.
        // processRead parses every completed DMA transfer so getOffset stays
        // fresh, but compass_ready — the EKF fuse gate — only flips true
        // once per ODR period (~10 ms).
        if (compass.isReady() && compass.isReadComplete()) {
            compass.processRead();
            if (now - lastCompassFuseMs >= 10) {
                sensors.compass_ready = true;
                sensors.compass_heading_rad = Math::degreesToRadians(compass.getOffset());
                lastCompassFuseMs = now;
            }
            compass.startRead();
        }

        // Sonar. Async pattern: processRead() + startRead(). Using sonar.read()
        // here instead would block the loop for up to SONAR_TIMEOUT_US (20 ms)
        // and fire the trigger twice per cycle.
        if (sonar.isReadComplete()) {
            const SonarReading r = sonar.processRead();
            sensors.sonar_ready = true;
            // Sentinel -1.0f = invalid reading (timeout / no echo). Downstream
            // consumers gate on >0 to distinguish a real measurement from a
            // dead sensor — otherwise 0 cm would masquerade as "wall right in
            // front" and wall-avoidance would push maximum repulsion.
            for (uint8_t i = 0; i < SONAR_COUNT; ++i)
                sensors.sonar_distance_m[i] = r.valid[i] ? r.distance[i] * 0.01f : -1.0f;
            sonar.startRead();
        }

        // IR ball. Gate low-confidence observations so marginal frames don't
        // push the EKF/IMM around — IMM predict is a safer fallback.
        if (ir_has_new_frame(boardIR, lastIrSeq)) {
            lastIrSeq = ir_get_frame_sequence(boardIR);
            const uint16_t* raw = ir_get_buffer(boardIR);
            const uint32_t sensorCount = ir_get_sensor_count(boardIR);

            if (const IRBallObservation observation = ballTracker.process(raw, sensorCount);
                observation.valid && observation.confidence >= 0.05f) {
                if (!ballTracker.isS0Locked()) {
                    // Boot calibration keeps rotating the DMA mapping wrong.
                    // While uncalibrated the FSM has been driving forward, so
                    // whichever sensor just fired is by construction the
                    // front — lock it as sensor 0 and drop this frame (the
                    // bearing was computed against the pre-rotation angle
                    // table). Subsequent frames fuse normally.
                    //
                    // Gate with a much higher confidence than the generic
                    // 0.05 processing threshold: spurious boot-time IR noise
                    // will easily pass 0.05 but a real ball sighting gives
                    // confidence in the 0.4-0.9 range. A false lock here
                    // derails the FSM (ball-lost branch becomes RETURN_POS
                    // instead of FIND_BALL), so the robot sits still / orbits
                    // instead of driving forward to find the ball.
                    if (observation.confidence >= 0.30f) {
                        ballTracker.lockS0AtSensor(observation.peakSensor);
                    }
                    // else: drop the frame silently; IMM stays unchanged,
                    // FSM stays in FIND_BALL, robot keeps crawling forward.
                } else {
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

        // Possession hint: no dribbler contact sensor, so reuse the strategy's
        // geometric "ball in cage" predicate against the previous tick's IMM
        // estimate. Lets the tracker commit to BALL_MODE_FRIENDLY sooner when
        // the ball is physically trapped in front of the robot. Body-frame
        // delta + range are already cached by RobotBrain.
        sensors.possession_hint = BALL_MODE_FREE;
        {
            const DigitalField& f = brain.field();
            if (f.ball.visible
                && f.ball.distM < 0.15f
                && f.ball.byBody > 0.0f
                && fabsf(f.ball.bxBody) < 0.05f)
                sensors.possession_hint = BALL_MODE_FRIENDLY;
        }

        // Stuck detection from the previous iteration's commanded drive.
        // OR with spin detection so a runaway rotation trips STUCK_RECOVERY
        // regardless of whether the wheels are stalled.
        const float headingRad = Math::degreesToRadians(compass.getOffset());
        const bool spinTrip = spin.update(now, headingRad, lastRotation);
        sensors.stuck_detected = stuck.update(now, lastDrive, lastRotation) || spinTrip;

        // --- Run the shared algorithm pipeline ---
        auto [drive, rotation, state] = brain.tick(sensors);

        // Pre-S0-lock bootstrap: force a forward translation regardless of
        // whatever the FSM produced. Boot-time IR channel calibration keeps
        // misfiring, so the FSM's body-frame decisions can't be trusted until
        // IRBallProcessor has locked the first-firing sensor as index 0. The
        // compass PD holds yaw (below), so this crawl is physically forward;
        // the first confident IR hit will fire the lock and normal behavior
        // resumes on the next tick.
        if (!ballTracker.isS0Locked()) {
            drive = {0.0f, 30.0f};
        }

        // Hold heading at the startup anchor (compass offset = 0) regardless
        // of what the FSM produced. Design decision: the robot never spins to
        // search — the FSM controls translation, the compass PD holds yaw.
        // This is the same override testIRBallSeek uses, now in the production
        // path for parity.
        rotation = compass.computeRotation(0.0f);

        // --- Output to motors ---
        motorDriver.driveVector(drive, rotation);
        motorDriver.updateAllMotors();

        lastDrive = drive;
        lastRotation = rotation;
    }
#endif
}
