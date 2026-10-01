#ifndef BUCKY_TESTS_H
#define BUCKY_TESTS_H

#include "hardware/motor/MotorDriver.h"
#include "hardware/sensors/pos/Compass.h"
#include "hardware/sensors/pos/Accelerometer.h"
#include "hardware/sensors/pos/Sonar.h"
#include "hardware/io/i2c/I2CDMA.h"
#include "hardware/sensors/GPort.h"

struct TestContext {
    MotorDriver& motorDriver;
    Compass& compass;
    Accelerometer& accel;
    Sonar& sonar;
    I2CDMABus& i2c;
    GPort* irPort;   // nullptr when no G port carries the IR ring
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
void testCompass(const TestContext& ctx);
void testIRPositioning(const TestContext& ctx);
void testIRBallSeek(const TestContext& ctx);
void testMainLoopSensors(const TestContext& ctx);
void testStuckDetector(const TestContext& ctx);
void testStrategyFSM(const TestContext& ctx);

// Load calibration from EEPROM (saved by testCalibrate / testCompassCalibrate).
// Returns true if valid calibration was found and applied.
bool loadCalibration(MotorDriver& md, Compass& compass);

#endif //BUCKY_TESTS_H
