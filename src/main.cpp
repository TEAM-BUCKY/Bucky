#include <Arduino.h>

#include "motor/MotorDriver.h"
#include "io/i2c/I2CDMA.h"
#include "sensors/pos/Compass.h"
#include "sensors/pos/Sonar.h"
#include "io/cordic/cordic.h"
#include "tests/tests.h"

// #define RUN_TEST testHoldHeading

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

    i2c_dma_init(&i2c1, I2C1,
                  GPIOB, 9, 4,    // SDA: PB9  AF4
                  GPIOA, 15, 4,   // SCL: PA15 AF4
                  DMA1, DMA1_Channel6, DMAMUX1_Channel5,
                  DMAMUX_REQ_I2C1_RX, DMA1_Channel6_IRQn);

    // compass.begin(i2c1);

    // SonarPins sonarPins = {.trigPin = PXX, .echoPins = {PXX, PXX, PXX, PXX}};
    // sonar.begin(sonarPins);

    analogReadResolution(12);

    motorDriver.init();

    // while (!compass.tick()) {}
}

int main() {
    setupEnvironment();

    TestContext ctx = {motorDriver, compass, sonar, i2c1};

#ifdef RUN_TEST
    RUN_TEST(ctx);
#else
    while (true) {
        // const float rotation = compass.computeRotation(0);

        motorDriver.driveDegrees(0, 30, 0);
        motorDriver.updateAllMotors();
    }
#endif

    return 0;
}
