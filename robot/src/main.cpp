#include <Arduino.h>
#include <cmath>

#include "debug.h"
#include "motor/MotorDriver.h"
#include "control/ball/IRBallProcessor.h"
#include <control/pos/SelfLocalizationEKF.h>
#include <field/DigitalField.h>
#include <io/i2c/I2CDMA.h>
#include <sensors/pos/Compass.h>
#include <sensors/pos/Sonar.h>
#include <io/cordic/cordic.h>
#include <sensors/IRSensor.h>
#include <strategy/StrategyFSM.h>
#include <tests/tests.h>

#include "robot/RobotBrain.h"
#include "robot/SpinDetector.h"
#include "robot/StuckDetector.h"

// Set to run a test instead of the main loop. testADCRaw runs before
// setupEnvironment for clean isolation; all others run after.
// #define RUN_TEST testDriveForward
// #define RUN_TEST_EARLY  // only for testADCRaw (runs before setupEnvironment)

MotorPin m1 = {PA8, PA9};
MotorPin m2 = {PB12, PB13};
MotorPin m3 = {PC7, PC6};

MotorDriver motorDriver(m1, m2, m3);
I2CDMABus i2c1;
Compass compass;
Accelerometer accel;
Sonar sonar;

I2C_DMA_RX_HANDLER(1, 6, i2c1)

void setupEnvironment() {
    Serial.begin(115200);
    cordic_init();

    i2c_dma_init<I2CFrequency::FMP_3M4>(&i2c1, I2C1,
                  GPIOB, 9, 4,    // SDA: PB9  AF4
                  GPIOA, 15, 4,   // SCL: PA15 AF4
                  DMA1, DMA1_Channel6, DMAMUX1_Channel5,
                  DMAMUX_REQ_I2C1_RX, DMA1_Channel6_IRQn);

    compass.begin(i2c1);
    accel.begin(i2c1);

    analogReadResolution(12);

    constexpr EncoderPins encoderPins[3] = {
        {PA7, PA6},   // M1
        {PB2, PB1},   // M2
        {PA5, PA2},   // M3
    };
    motorDriver.init(MIN_SPEED, MAX_SPEED, encoderPins);


    constexpr SonarPins sonarPins = {.trigPin = PB11, .echoPins = {PA10, PC10, PC11, PC12}};
    sonar.begin(sonarPins);

    ir_sensor_init();

    // Detect the free-running mux-counter rotation so sensor indices line up
    // with physical positions. Must run after ir_sensor_init (DMA must be
    // streaming) and before anything reads the IR ring. Assumes the field is
    // clear of active IR sources at boot.
    const uint8_t irOffset = ir_calibrate_channels(2);
    DBG_PRINT("IR Board 2 channel offset: "); DBG_PRINTLN(irOffset);

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
    { constexpr TestContext ctx = {motorDriver, compass, accel, sonar, i2c1}; RUN_TEST(ctx); }
#endif

    setupEnvironment();

#if defined(RUN_TEST) && !defined(RUN_TEST_EARLY)
    constexpr TestContext ctx = {motorDriver, compass, accel, sonar, i2c1};

    RUN_TEST(ctx);
#else


    while (true) {

    }
#endif
}
