#ifndef BUCKY_TESTS_H
#define BUCKY_TESTS_H

#include <hardware/motor/Motors.h>
#include <hardware/sensors/pos/Compass.h>
#include <hardware/sensors/pos/Accelerometer.h>
#include <hardware/sensors/pos/Sonar.h>
#include <hardware/io/i2c/I2CDMA.h>
#include <hardware/motor/Kicker.h>
#include <hardware/sensors/gport/GPort.h>
#include <hardware/comms/HM10.h>

struct TestContext {
    Motors& motorDriver;
    Kicker* kicker;   // nullptr when the board has no kicker
    Compass& compass;
    Accelerometer& accel;
    Sonar& sonar;
    I2CDMABus& i2c;
    GPort* irPort;     // nullptr when no G port carries the IR ring
    GPort* linePort;   // nullptr when no G port carries the line sensors
    HM10* bluetooth;   // nullptr when the board has no Bluetooth module
};

void testHoldHeading(const TestContext& ctx);
void testIR(const TestContext& ctx);
void testLine(const TestContext& ctx);
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
void testBluetooth(const TestContext& ctx);
void testBluetoothProbe(const TestContext& ctx);
void testBluetoothBaud(const TestContext& ctx);
void testBluetoothTxCheck(const TestContext& ctx);

// Load calibration from EEPROM (saved by testCalibrate / testCompassCalibrate).
// Returns true if valid calibration was found and applied.
bool loadCalibration(Motors& md, Compass& compass);

#endif //BUCKY_TESTS_H
