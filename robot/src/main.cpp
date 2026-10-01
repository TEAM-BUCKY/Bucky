#include "debug.h"

#include "board/board.h"
#include "hardware/io/cordic/cordic.h"
#include "hardware/io/i2c/I2CDMA.h"
#include "hardware/io/serial/SerialPort.h"
#include "hardware/motor/MotorDriver.h"
#include "hardware/sensors/pos/Compass.h"
#include "hardware/sensors/pos/Sonar.h"
#include "tests/tests.h"

// Set to run a test instead of the main loop.
// #define RUN_TEST testDriveForward

MotorDriver motorDriver(board::MOTOR1, board::MOTOR2, board::MOTOR3);
I2CDMABus sensorI2C;
Compass compass;
Accelerometer accel;
Sonar sonar;

#ifdef BOARD_HAS_GPORTS
GPort gPort1;
GPort gPort2;

[[maybe_unused]] static GPort* irPort() {
    if (board::G_PORT1_KIND == GSensorKind::IR) return &gPort1;
    if (board::G_PORT2_KIND == GSensorKind::IR) return &gPort2;
    return nullptr;
}
#else
[[maybe_unused]] [[maybe_unused]] static GPort* irPort() { return nullptr; }
#endif

UsbSerialPort host;

#ifdef BOARD_HAS_BLUETOOTH
UartSerialPort<512, 256> bluetooth;
#endif

void setupEnvironment() {
    host.begin();
    cordic_init();

    i2c_dma_init<board::SENSOR_I2C_FREQ>(&sensorI2C, board::SENSOR_I2C.instance,
                                         board::SENSOR_I2C.sda, board::SENSOR_I2C.scl,
                                         board::SENSOR_I2C.dmaRx, board::SENSOR_I2C.dmaRxRequest);

    compass.begin(sensorI2C);
    accel.begin(sensorI2C);

#ifdef BOARD_HAS_BLUETOOTH
    bluetooth.begin(board::BLUETOOTH_UART, board::BLUETOOTH_BAUD);
#endif

    analogReadResolution(12);

    motorDriver.init(MIN_SPEED, MAX_SPEED, board::ENCODERS);

    sonar.begin(board::SONAR);

#ifdef BOARD_HAS_GPORTS
    if (!gPort1.begin(board::G_PORT1, board::G_PORT1_KIND)) DBG_PRINTLN("G port 1 init failed");
    if (!gPort2.begin(board::G_PORT2, board::G_PORT2_KIND)) DBG_PRINTLN("G port 2 init failed");
#endif

    while (!compass.tick()) {}

    // if (loadCalibration(motorDriver, compass)) {
    //     DBG_PRINTLN("Calibration loaded from EEPROM.");
    // } else {
    //     DBG_PRINTLN("No valid calibration in EEPROM; using defaults.");
    // }

    // Anchor startHeading to a calibrated reading. The boot-time sampling
    // in compass.tick() runs before loadCalibration(), so it cannot set a
    // valid startHeading on its own.
    if (compass.isReady()) {
        compass.update();
        compass.reset();
        compass.startRead();
    }
    sonar.startRead();
}

[[noreturn]] int main() {
    init();
    setupEnvironment();

#ifdef RUN_TEST
    dbg.setBlocking(true);   // test reports are long; let them arrive complete
    const TestContext ctx = {motorDriver, compass, accel, sonar, sensorI2C, irPort()};
    RUN_TEST(ctx);
#endif

    while (true) {
    }
}
