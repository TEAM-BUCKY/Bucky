#ifndef BUCKY_TESTS_H
#define BUCKY_TESTS_H

#include "../motor/MotorDriver.h"
#include "../pos/Compass.h"
#include "../ir/IRController.h"
#include "../i2c/I2CManager.h"

void testHoldHeading(MotorDriver& motorDriver, Compass& compass, IRController& irController, I2CManager& i2c);
void testIR(MotorDriver& motorDriver, Compass& compass, IRController& irController, I2CManager& i2c);
void testI2CScan(MotorDriver& motorDriver, Compass& compass, IRController& irController, I2CManager& i2c);
void testDriveForward(MotorDriver& motorDriver, Compass& compass, IRController& irController, I2CManager& i2c);

#endif //BUCKY_TESTS_H
