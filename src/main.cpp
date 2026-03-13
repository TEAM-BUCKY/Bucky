#include <Arduino.h>

#include "motor/MotorDriver.h"
#include "i2c/I2CManager.h"
#include "pos/Compass.h"
#include "cordic/cordic.h"
#include "tests/tests.h"

// #define RUN_TEST testHoldHeading

MotorPin m1 = {PA8, PA9};
MotorPin m2 = {PA10, PC10};
MotorPin m3 = {PB6, PB7};

MotorDriver motorDriver(m1, m2, m3);
I2CManager i2c;
Compass compass;

void setupEnvironment() {
    init();
    cordic_init();

    i2c.configure(I2CBus::BUS1, PB9, PA15);
    i2c.init(I2CBus::BUS1);

    // compass.begin(i2c.getBus(I2CBus::BUS1));

    analogReadResolution(12);

    motorDriver.init();

    // while (!compass.tick()) {}
}

int main() {
    setupEnvironment();


#ifdef RUN_TEST
    RUN_TEST(motorDriver, compass, i2c);
#else
    while (true) {
        // const float rotation = compass.computeRotation(0);

        motorDriver.driveDegrees(0, 30, 0);
        motorDriver.updateAllMotors();
    }
#endif

    return 0;
}
