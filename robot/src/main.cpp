#include <debug.h>

#include <board/board.h>
#include <hardware/io/cordic/cordic.h>
#include <hardware/io/gpio/gpio.h>
#include <hardware/io/i2c/I2CDMA.h>
#include <hardware/io/serial/SerialPort.h>
#include <hardware/comms/HM10.h>
#include <hardware/motor/Kicker.h>
#include <hardware/motor/Motors.h>
#include <hardware/sensors/Button.h>
#include <hardware/sensors/pos/Compass.h>
#include <hardware/sensors/pos/Sonar.h>
#include <tests/tests.h>

#define RUN_TEST testBluetooth

Motors motorDriver(Board::MOTOR1, Board::MOTOR2, Board::MOTOR3);
I2CDMABus sensorI2C;
Compass compass;
Accelerometer accel;
Sonar sonar;
#ifdef BOARD_HAS_KICKER
Kicker kicker(Board::KICKER);
#endif

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
HM10 bluetooth;
#endif

void setupEnvironment() {
    host.begin();
    cordic_init();

    delay(3000);


    i2c_dma_init<Board::SENSOR_I2C_FREQ>(&sensorI2C, Board::SENSOR_I2C.instance,
                                         Board::SENSOR_I2C.sda, Board::SENSOR_I2C.scl,
                                         Board::SENSOR_I2C.dmaRx, Board::SENSOR_I2C.dmaRxRequest);

    compass.begin(sensorI2C);
    accel.begin(sensorI2C);

#ifdef BOARD_HAS_BLUETOOTH
    if (!bluetooth.begin(Board::BLUETOOTH_UART, Board::BLUETOOTH_BAUD, Board::BLUETOOTH_NAME))
        DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_MAIN, "HM-10 not configured (absent, or still linked from before reset)");
#endif

    analogReadResolution(12);

    motorDriver.init(0.0f, 1.0f, Board::ENCODERS);
#ifdef BOARD_HAS_KICKER
    kicker.init();
#endif

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

#ifdef BOARD_HAS_BUTTONS
    if (!button1.begin(Board::BUTTON1)) DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_MAIN, "Button 1 EXTI line taken");
    if (!button2.begin(Board::BUTTON2)) DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_MAIN, "Button 2 EXTI line taken");
#endif



    Kicker* const kickerPtr = &kicker;

#ifdef BOARD_HAS_BLUETOOTH
    HM10* const bluetoothPtr = &bluetooth;
#else
    HM10* const bluetoothPtr = nullptr;
#endif
    const TestContext ctx = {motorDriver, kickerPtr, compass, accel, sonar, sensorI2C, irPort(), linePort(),
                             bluetoothPtr};

#ifdef RUN_TEST
    dbg.setBlocking(true);   // test reports are long; let them arrive complete
    RUN_TEST(ctx);
#endif

    kicker.kick(1.0f);
    motorDriver.driveMotorsDirect(100, 0, 0);

    uint32_t lastTick = millis();
    while (true) {
        motorDriver.syncUpdateAllMotors();

#ifdef BOARD_HAS_BUTTONS
        if (button1.pressed()) DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_MAIN, "Button 1 pressed");
        if (button2.pressed()) DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_MAIN, "Button 2 pressed");
#endif

        if (millis() - lastTick >= 1000) {
            lastTick += 1000;
            DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_MAIN, "Main loop running...");
        }
        delay(2);   // fast enough to catch a press, idle enough to be free
    }
}
