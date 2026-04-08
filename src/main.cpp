#include <Arduino.h>

#include "motor/MotorDriver.h"
#include "control/EKF.h"
#include "control/IRBallTracker.h"
#include "io/i2c/I2CDMA.h"
#include "sensors/pos/Compass.h"
#include "sensors/pos/Sonar.h"
#include "io/cordic/cordic.h"
#include "sensors/IRSensor.h"
#include "tests/tests.h"

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

[[noreturn]] int main() {
    setupEnvironment();

    IRBallTracker ballTracker;
    EKF ballEkf;
    ballEkf.reset();
    ballEkf.setProcessNoise(120.0f);
    ballEkf.setMeasurementNoise(18.0f, 10.0f);


#ifdef RUN_TEST
    TestContext ctx = {motorDriver, compass, sonar, i2c1};

    RUN_TEST(ctx);
#else

    VectorXY driveVector = {0.0f, 0.0f};
    float rotation = 0.0f;
    uint32_t lastTick = HAL_GetTick();

    while (true) {
        const uint32_t now = HAL_GetTick();
        const float dt = static_cast<float>(now - lastTick) * 0.001f;
        lastTick = now;

        ballEkf.predict(dt);

        compass.update();
        rotation = compass.computeRotation(0.0f);

        IRBallObservation bestObservation = {};
        for (const uint8_t board : {static_cast<uint8_t>(1), static_cast<uint8_t>(2)}) {
            const uint16_t* raw = ir_get_buffer(board);
            const uint32_t sensorCount = ir_get_sensor_count(board);
            const IRBallObservation observation = ballTracker.process(raw, sensorCount);
            if (!observation.valid) {
                continue;
            }

            const float confidenceDelta = observation.confidence - bestObservation.confidence;
            if (!bestObservation.valid || confidenceDelta > 1.0e-6f ||
                (confidenceDelta > -1.0e-6f && observation.strength > bestObservation.strength)) {
                bestObservation = observation;
            }
        }

        ballEkf.updateObservation(bestObservation);

        driveVector = {ballEkf.getX(), ballEkf.getY()};
        motorDriver.driveVector(driveVector, rotation);
        motorDriver.updateAllMotors();
    }
#endif

    return 0;
}
