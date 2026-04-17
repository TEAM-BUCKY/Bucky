#include <Arduino.h>
#include <cmath>

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

// Set to run a test instead of the main loop. testADCRaw runs before
// setupEnvironment for clean isolation; all others run after.
#define RUN_TEST testCalibrate
// #define RUN_TEST_EARLY  // only for testADCRaw (runs before setupEnvironment)

MotorPin m1 = {PA8, PA9};
MotorPin m2 = {PB12, PB13};
MotorPin m3 = {PC6, PC7};

MotorDriver motorDriver(m1, m2, m3);
I2CDMABus i2c1;
Compass compass;
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

    while (!compass.tick()) {}
}

constexpr uint8_t boardIR = 1;

[[noreturn]] int main() {
    init();
    delay(5000);

#if defined(RUN_TEST) && defined(RUN_TEST_EARLY)
    // Early tests run BEFORE setupEnvironment to avoid DMA/timer/OPAMP contamination.
    Serial.begin(115200);
    { constexpr TestContext ctx = {motorDriver, compass, sonar, i2c1}; RUN_TEST(ctx); }
#endif

    setupEnvironment();

#if defined(RUN_TEST) && !defined(RUN_TEST_EARLY)
    constexpr TestContext ctx = {motorDriver, compass, sonar, i2c1};

    RUN_TEST(ctx);
#else

    RobotBrain brain;
    IRBallProcessor ballTracker;

    brain.reset(0.0f, -0.2f, 0.0f);

    uint32_t lastTick = HAL_GetTick();
    uint32_t lastIrSeq = ir_get_frame_sequence(boardIR);

    while (true) {
        const uint32_t now = HAL_GetTick();
        const float dt = static_cast<float>(now - lastTick) * 0.001f;
        lastTick = now;

        // --- Gather sensor measurements ---
        BrainSensors sensors = {};
        sensors.dt = dt;
        sensors.now_ms = now;

        // Compass
        if (compass.isReadComplete()) {
            sensors.compass_ready = true;
            sensors.compass_heading_rad = Math::degreesToRadians(compass.getOffset());
            compass.startRead();
        }

        // Sonar
        if (sonar.isReadComplete()) {
            sensors.sonar_ready = true;
            auto [distance] = sonar.read();
            for (uint8_t i = 0; i < SONAR_COUNT; ++i)
                sensors.sonar_distance_m[i] = distance[i] * 0.01f;
            sonar.startRead();
        }

        // IR ball
        if (ir_has_new_frame(boardIR, lastIrSeq)) {
            lastIrSeq = ir_get_frame_sequence(boardIR);
            const uint16_t* raw = ir_get_buffer(boardIR);
            const uint32_t sensorCount = ir_get_sensor_count(boardIR);

            if (const IRBallObservation observation = ballTracker.process(raw, sensorCount);
                observation.valid) {
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

        // Possession hint (no dribbler contact sensor yet)
        sensors.possession_hint = BALL_MODE_FREE;

        // --- Run the shared algorithm pipeline ---
        auto [drive, rotation, state] = brain.tick(sensors);

        // --- Output to motors ---
        motorDriver.driveVector(drive, rotation);
        motorDriver.updateAllMotors();
    }
#endif
}
