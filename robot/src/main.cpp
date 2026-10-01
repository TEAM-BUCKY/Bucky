#include <cmath>

#include "debug.h"

#include "board/board.h"
#include "hardware/motor/MotorDriver.h"
#include <hardware/io/i2c/I2CDMA.h>
#include <hardware/io/uart/UARTDMA.h>
#include <hardware/sensors/pos/Compass.h>
#include <hardware/sensors/pos/Sonar.h>
#include <hardware/io/cordic/cordic.h>
#include <hardware/sensors/IRSensor.h>
#include <tests/tests.h>

// Set to run a test instead of the main loop. testADCRaw runs before
// setupEnvironment for clean isolation; all others run after.
// #define RUN_TEST testDriveForward
// #define RUN_TEST_EARLY  // only for testADCRaw (runs before setupEnvironment)

MotorDriver motorDriver(board::MOTOR1, board::MOTOR2, board::MOTOR3);
I2CDMABus sensorI2C;
Compass compass;
Accelerometer accel;
Sonar sonar;

#ifdef BOARD_HAS_GPORTS
GPort gPort1;
GPort gPort2;
#endif

#ifdef BOARD_HAS_UART
static uint8_t uartTxBuf[512];
static uint8_t uartRxBuf[256];
UartDma uart;
#endif

void setupEnvironment() {
    Serial.begin(115200);
    cordic_init();

    i2c_dma_init<board::SENSOR_I2C_FREQ>(&sensorI2C, board::SENSOR_I2C.instance,
                                         board::SENSOR_I2C.sda, board::SENSOR_I2C.scl,
                                         board::SENSOR_I2C.dmaRx, board::SENSOR_I2C.dmaRxRequest);

    compass.begin(sensorI2C);
    accel.begin(sensorI2C);

#ifdef BOARD_HAS_UART
    const UartDmaConfig uartConfig = {
        .uart = board::UART.instance,
        .tx_pin = board::UART.tx,
        .rx_pin = board::UART.rx,
        .baud = 115200,
        .tx_dma = board::UART.dmaTx,
        .tx_request = board::UART.dmaTxRequest,
        .tx_buf = uartTxBuf,
        .tx_size = sizeof(uartTxBuf),
        .rx_dma = board::UART.dmaRx,
        .rx_request = board::UART.dmaRxRequest,
        .rx_buf = uartRxBuf,
        .rx_size = sizeof(uartRxBuf),
        .irq_priority = 4,
        .on_rx = nullptr,
        .on_rx_ctx = nullptr,
    };
    uart_dma_init(&uart, &uartConfig);
#endif

    analogReadResolution(12);

    motorDriver.init(MIN_SPEED, MAX_SPEED, board::ENCODERS);

    sonar.begin(board::SONAR);

#ifdef BOARD_HAS_GPORTS
    if (!gPort1.begin(board::G_PORT1, board::G_PORT1_KIND)) DBG_PRINTLN("G port 1 init failed");
    if (!gPort2.begin(board::G_PORT2, board::G_PORT2_KIND)) DBG_PRINTLN("G port 2 init failed");
#else
    ir_sensor_init();
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

// Board 1 is disabled in IRSensor.h; the active IR ring is Board 2.
constexpr uint8_t boardIR = 2;

[[noreturn]] int main() {
    init();
//
// #ifdef RUN_TEST
//     delay(5000);
// #endif

#if defined(RUN_TEST) && defined(RUN_TEST_EARLY)
    // Early tests run BEFORE setupEnvironment to avoid DMA/timer/OPAMP contamination.
    Serial.begin(115200);
    { constexpr TestContext ctx = {motorDriver, compass, accel, sonar, sensorI2C}; RUN_TEST(ctx); }
#endif

    setupEnvironment();

#if defined(RUN_TEST) && !defined(RUN_TEST_EARLY)
    constexpr TestContext ctx = {motorDriver, compass, accel, sonar, sensorI2C};

    RUN_TEST(ctx);
#else


    while (true) {

    }
#endif
}
