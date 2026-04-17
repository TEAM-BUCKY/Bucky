#ifndef BUCKY_TESTS_H
#define BUCKY_TESTS_H

#include "../motor/MotorDriver.h"
#include "../sensors/pos/Compass.h"
#include "../sensors/pos/Accelerometer.h"
#include "../sensors/pos/Sonar.h"
#include "io/i2c/I2CDMA.h"

struct TestContext {
    MotorDriver& motorDriver;
    Compass& compass;
    Accelerometer& accel;
    Sonar& sonar;
    I2CDMABus& i2c;
};

void testHoldHeading(const TestContext& ctx);
void testIR(const TestContext& ctx);
void testI2CScan(const TestContext& ctx);
void testDriveForward(const TestContext& ctx);
void testSonar(const TestContext& ctx);
void testEncoder(const TestContext& ctx);
void testCalibrate(const TestContext& ctx);
void testCalibrationDump(const TestContext& ctx);
void testCompassCalibrate(const TestContext& ctx);
void testADCRaw(const TestContext& ctx);
void testCompass(const TestContext& ctx);
void testIRPositioning(const TestContext& ctx);

// Load calibration from EEPROM (saved by testCalibrate / testCompassCalibrate).
// Returns true if valid calibration was found and applied.
bool loadCalibration(MotorDriver& md, Compass& compass);

#endif //BUCKY_TESTS_H
