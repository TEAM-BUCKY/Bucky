#include "debug.h"

#include "board/board.h"
#include "hardware/io/cordic/cordic.h"
#include "hardware/io/gpio/gpio.h"
#include "hardware/io/i2c/I2CDMA.h"
#include "hardware/io/serial/SerialPort.h"
#include "hardware/motor/Kicker.h"
#include "hardware/motor/Motors.h"
#include "hardware/sensors/Button.h"
#include "hardware/sensors/pos/Compass.h"
#include "hardware/sensors/pos/Sonar.h"
#include "tests/tests.h"

// Set to run a test instead of the main loop.
#define RUN_TEST testLine

Motors motorDriver(Board::MOTOR1, Board::MOTOR2, Board::MOTOR3);
I2CDMABus sensorI2C;
Compass compass;
Accelerometer accel;
Sonar sonar;
Kicker kicker(Board::KICKER);

#ifdef BOARD_HAS_GPORTS
GPort gPort1;
GPort gPort2;

[[maybe_unused]] static GPort* irPort() {
    if constexpr (Board::G_PORT1_KIND == GSensorKind::IR) return &gPort1;
    if constexpr (Board::G_PORT2_KIND == GSensorKind::IR) return &gPort2;
    return nullptr;
}

[[maybe_unused]] static GPort* linePort() {
    if constexpr (Board::G_PORT1_KIND == GSensorKind::Line) return &gPort1;
    if constexpr (Board::G_PORT2_KIND == GSensorKind::Line) return &gPort2;
    return nullptr;
}
#else
[[maybe_unused]] [[maybe_unused]] static GPort* irPort() { return nullptr; }
[[maybe_unused]] static GPort* linePort() { return nullptr; }
#endif

UsbSerialPort host;

#ifdef BOARD_HAS_BLUETOOTH
UartSerialPort<512, 256> bluetooth;
#endif

void setupEnvironment() {
    host.begin();
    cordic_init();


    i2c_dma_init<Board::SENSOR_I2C_FREQ>(&sensorI2C, Board::SENSOR_I2C.instance,
                                         Board::SENSOR_I2C.sda, Board::SENSOR_I2C.scl,
                                         Board::SENSOR_I2C.dmaRx, Board::SENSOR_I2C.dmaRxRequest);

    compass.begin(sensorI2C);
    accel.begin(sensorI2C);

#ifdef BOARD_HAS_BLUETOOTH
    bluetooth.begin(Board::BLUETOOTH_UART, Board::BLUETOOTH_BAUD);
#endif

    analogReadResolution(12);

    motorDriver.init(MIN_SPEED, MAX_SPEED, Board::ENCODERS);
    kicker.init();

    sonar.begin(Board::SONAR);

#ifdef BOARD_HAS_GPORTS
    if (!gPort1.begin(Board::G_PORT1, Board::G_PORT1_KIND)) DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_GPORT, "General Purpose port 1 init failed");
    if (!gPort2.begin(Board::G_PORT2, Board::G_PORT2_KIND)) DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_GPORT, "General Purpose port 2 init failed");
#endif

    while (!compass.tick()) {}

    if (loadCalibration(motorDriver, compass)) {
        DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_CALIBRATION, "Calibration loaded from flash.");
    } else {
        DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_CALIBRATION, "No valid calibration in flash; using defaults.");
    }


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

    if (!button1.begin(Board::BUTTON1)) DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_MAIN, "Button 1 EXTI line taken");
    if (!button2.begin(Board::BUTTON2)) DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_MAIN, "Button 2 EXTI line taken");


#ifdef RUN_TEST
    dbg.setBlocking(true);   // test reports are long; let them arrive complete
    const TestContext ctx = {motorDriver, kicker, compass, accel, sonar, sensorI2C, irPort(), linePort()};
    RUN_TEST(ctx);
#endif
    kicker.kick(1.0f);
    motorDriver.driveMotorsDirect(100, 0, 0);

    uint32_t lastTick = millis();
    while (true) {
        motorDriver.syncUpdateAllMotors();

        if (button1.pressed()) DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_MAIN, "Button 1 pressed");
        if (button2.pressed()) DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_MAIN, "Button 2 pressed");

        if (millis() - lastTick >= 1000) {
            lastTick += 1000;
            DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_MAIN, "Main loop running...");
        }
        delay(2);   // fast enough to catch a press, idle enough to be free
    }
}
