#ifndef BUCKY_TESTS_H
#define BUCKY_TESTS_H

#include "../motor/MotorDriver.h"
#include "../pos/Compass.h"
#include "io/i2c/I2CDMA.h"

void testHoldHeading(MotorDriver& motorDriver, Compass& compass, I2CDMABus& i2c);
void testIR(MotorDriver& motorDriver, Compass& compass, I2CDMABus& i2c);
void testI2CScan(MotorDriver& motorDriver, Compass& compass, I2CDMABus& i2c);
void testDriveForward(MotorDriver& motorDriver, Compass& compass, I2CDMABus& i2c);

#endif //BUCKY_TESTS_H
