#ifndef BUCKY_TESTS_H
#define BUCKY_TESTS_H

#include "../motor/MotorDriver.h"
#include "../pos/Compass.h"
#include "../pos/Sonar.h"
#include "io/i2c/I2CDMA.h"

struct TestContext {
    MotorDriver& motorDriver;
    Compass& compass;
    Sonar& sonar;
    I2CDMABus& i2c;
};

void testHoldHeading(const TestContext& ctx);
void testIR(TestContext& ctx);
void testI2CScan(const TestContext& ctx);
void testDriveForward(const TestContext& ctx);
void testSonar(const TestContext& ctx);

#endif //BUCKY_TESTS_H
