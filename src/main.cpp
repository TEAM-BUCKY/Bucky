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
#include "robot/StuckDetector.h"

// Set to run a test instead of the main loop. testADCRaw runs before
// setupEnvironment for clean isolation; all others run after.
#define RUN_TEST testIRPositioning
// #define RUN_TEST_EARLY  // only for testADCRaw (runs before setupEnvironment)

MotorPin m1 = {PA9, PA8};
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
        {PA6, PA7},   // M1
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

    if (loadCalibration(motorDriver, compass)) {
        DBG_PRINTLN("Calibration loaded from EEPROM.");
    } else {
        DBG_PRINTLN("No valid calibration in EEPROM; using defaults.");
    }

    // Kick off the first async reads so the main loop's first iteration has
    // data in flight rather than waiting a full sample period.
    compass.startRead();
    sonar.startRead();
}

// Board 1 is disabled in IRSensor.h; the active IR ring is Board 2.
constexpr uint8_t boardIR = 2;

[[noreturn]] int main() {
    init();
    delay(5000);

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

    StuckDetector stuck;
    stuck.reset(HAL_GetTick());

    uint32_t lastTick = HAL_GetTick();
    uint32_t lastIrSeq = ir_get_frame_sequence(boardIR);
    VectorXY lastDrive = {0.0f, 0.0f};
    float lastRotation = 0.0f;

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
        // robot's yaw never changes.
        if (compass.isReadComplete()) {
            compass.processRead();
            sensors.compass_ready = true;
            sensors.compass_heading_rad = Math::degreesToRadians(compass.getOffset());
            compass.startRead();
        }

        // Sonar. Async pattern: processRead() + startRead(). Using sonar.read()
        // here instead would block the loop for up to SONAR_TIMEOUT_US (20 ms)
        // and fire the trigger twice per cycle.
        if (sonar.isReadComplete()) {
            const SonarReading r = sonar.processRead();
            sensors.sonar_ready = true;
            for (uint8_t i = 0; i < SONAR_COUNT; ++i)
                sensors.sonar_distance_m[i] = r.distance[i] * 0.01f;
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
                const SelfLocState selfState = selfloc_get_state(&brain.selfLoc());
                const float alpha = selfState.theta
                                  + Math::degreesToRadians(observation.bearingDeg);
                const float rangeM = observation.rangeCm * 0.01f;
                sensors.ball_visible = true;
                sensors.ball_field_x = selfState.x + rangeM * cosf(alpha);
                sensors.ball_field_y = selfState.y + rangeM * sinf(alpha);
                sensors.ball_range_m = rangeM;
            }
        }

        // Possession hint: no dribbler contact sensor, so reuse the strategy's
        // geometric "ball in cage" predicate against the previous tick's IMM
        // estimate. Lets the tracker commit to BALL_MODE_FRIENDLY sooner when
        // the ball is physically trapped in front of the robot.
        sensors.possession_hint = BALL_MODE_FREE;
        {
            const DigitalField& f = brain.field();
            if (f.ball.visible) {
                const float bdx = f.ball.bx - f.self.x;
                const float bdy = f.ball.by - f.self.y;
                const float ballDist = hypotf(bdx, bdy);
                float bxBody = 0.0f, byBody = 0.0f;
                fieldToBody(bdx, bdy, f.self.theta, &bxBody, &byBody);
                if (ballDist < 0.15f && byBody > 0.0f && fabsf(bxBody) < 0.05f)
                    sensors.possession_hint = BALL_MODE_FRIENDLY;
            }
        }

        // Stuck detection from the previous iteration's commanded drive.
        sensors.stuck_detected = stuck.update(now, lastDrive, lastRotation);

        // --- Run the shared algorithm pipeline ---
        auto [drive, rotation, state] = brain.tick(sensors);

        // --- Output to motors ---
        motorDriver.driveVector(drive, rotation);
        motorDriver.updateAllMotors();

        lastDrive = drive;
        lastRotation = rotation;
    }
#endif
}
