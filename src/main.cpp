#include <Arduino.h>
#include <cmath>

#include "motor/MotorDriver.h"
#include "control/pos/EnemyTracker.h"
#include "control/ball/IMMBallTracker.h"
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

// #define RUN_TEST testIR

MotorPin m1 = {PA8, PA9};
MotorPin m2 = {PA10, PC10};
MotorPin m3 = {PB6, PB7};

MotorDriver motorDriver(m1, m2, m3);
I2CDMABus i2c1;
Compass compass;
Sonar sonar;

I2C_DMA_RX_HANDLER(1, 6, i2c1)

void setupEnvironment() {
    init();
    cordic_init();

    i2c_dma_init<I2CFrequency::FMP_3M4>(&i2c1, I2C1,
                  GPIOB, 9, 4,    // SDA: PB9  AF4
                  GPIOA, 15, 4,   // SCL: PA15 AF4
                  DMA1, DMA1_Channel6, DMAMUX1_Channel5,
                  DMAMUX_REQ_I2C1_RX, DMA1_Channel6_IRQn);

    compass.begin(i2c1);

    constexpr SonarPins sonarPins = {.trigPin = PB11, .echoPins = {PA10, PC10, PC11, PC12}};
    sonar.begin(sonarPins);

    analogReadResolution(12);

    motorDriver.init();

    // Initialize IR

    ir_sensor_init();

    while (!compass.tick()) {}
}

constexpr uint8_t boardIR = 1;

[[noreturn]] int main() {
    setupEnvironment();

    IRBallProcessor ballTracker;
    SelfLocalizationFilter selfLoc = {};
    IMMBallTracker ballImm;
    EnemyTracker enemyTracker;
    StrategyFSM strategy;
    DigitalField field = {};

    selfloc_reset(&selfLoc, 0.0f, -0.2f, 0.0f);
    ballImm.reset();
    enemyTracker.reset();


#ifdef RUN_TEST
    TestContext ctx = {motorDriver, compass, sonar, i2c1};

    RUN_TEST(ctx);
#else

    VectorXY driveVector = {0.0f, 0.0f};
    float rotation = 0.0f;
    GameState_t gameState = STATE_FIND_BALL;
    uint32_t lastTick = HAL_GetTick();
    uint32_t lastIrSeq = ir_get_frame_sequence(boardIR);

    constexpr float kCmdToMps = 0.012f;
    constexpr float kCmdToRadS = 0.02f;

    while (true) {
        const uint32_t now = HAL_GetTick();
        const float dt = static_cast<float>(now - lastTick) * 0.001f;
        lastTick = now;

        selfloc_predict(&selfLoc, dt, driveVector.x * kCmdToMps, driveVector.y * kCmdToMps, rotation * kCmdToRadS);

        if (compass.isReadComplete()) {
            const float headingRad = Math::degreesToRadians(compass.getOffset());
            selfloc_update_compass(&selfLoc, headingRad);

            compass.startRead();
        }

        if (sonar.isReadComplete()) {
            auto [distance] = sonar.read();
            for (uint8_t i = 0; i < SONAR_COUNT; ++i) {
                const float distanceM = distance[i] * 0.01f;
                if (const SonarUpdateResult result = selfloc_update_sonar(&selfLoc, i, distanceM); result.anomaly_detected)
                    enemyTracker.update(result.obstacle_x, result.obstacle_y, now);
            }
            enemyTracker.decay(now);

            sonar.startRead();
        }

        const SelfLocState selfState = selfloc_get_state(&selfLoc);
        const EnemyState enemyState = enemyTracker.getState();

        bool hasBallMeasurement = false;
        float ballMx = 0.0f;
        float ballMy = 0.0f;
        float ballRangeM = 0.0f;

        if (ir_has_new_frame(boardIR, lastIrSeq)) {
            lastIrSeq = ir_get_frame_sequence(boardIR);
            const uint16_t* raw = ir_get_buffer(boardIR);
            const uint32_t sensorCount = ir_get_sensor_count(boardIR);

            if (const IRBallObservation observation = ballTracker.process(raw, sensorCount); observation.valid) {
                const float alpha = selfState.theta + Math::degreesToRadians(observation.bearingDeg);
                ballRangeM = observation.rangeCm * 0.01f;
                ballMx = selfState.x + ballRangeM * cosf(alpha);
                ballMy = selfState.y + ballRangeM * sinf(alpha);
                hasBallMeasurement = true;
            }
        }

        ballImm.setPossessionHint((enemyState.confidence > 0.35f) ? BALL_MODE_ENEMY : BALL_MODE_FREE);
        ballImm.step(dt, hasBallMeasurement, ballMx, ballMy, ballRangeM, selfState, enemyState, now);

        const auto [bx, by, bvx, bvy, P, mu, innovation_mag, visible, lost_ms] = ballImm.getEstimate();

        field.self.x = selfState.x;
        field.self.y = selfState.y;
        field.self.theta = selfState.theta;
        field.self.vx = selfState.vx;
        field.self.vy = selfState.vy;
        field.self.omega = selfState.omega;
        field.self.P_xy = selfState.P_xy;
        field.ball.bx = bx;
        field.ball.by = by;
        field.ball.bvx = bvx;
        field.ball.bvy = bvy;
        field.ball.mu[0] = mu[0];
        field.ball.mu[1] = mu[1];
        field.ball.mu[2] = mu[2];
        field.ball.P_xy = P[0][0] + P[1][1];
        field.ball.innovation_mag = innovation_mag;
        field.ball.visible = visible;
        field.ball.lost_ms = lost_ms;
        field.enemy[0].x = enemyState.x;
        field.enemy[0].y = enemyState.y;
        field.enemy[0].vx = enemyState.vx;
        field.enemy[0].vy = enemyState.vy;
        field.enemy[0].confidence = enemyState.confidence;
        field.timestamp_ms = now;

        const auto [drive, _rotation, state] = strategy.update(field, gameState, false, false);
        gameState = state;

        driveVector = drive;
        rotation = _rotation;

        motorDriver.driveVector(driveVector, rotation);
        motorDriver.updateAllMotors();
    }
#endif
}
