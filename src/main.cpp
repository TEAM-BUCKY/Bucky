#include <Arduino.h>
#include <cmath>

#include "controls/Joystick.h"
#include "motor/MotorDriver.h"
#include "i2c/I2CManager.h"
#include "pos/Compass.h"
#include "pos/Sonar.h"
#include "tests/tests.h"

// Uncomment to run a test instead of normal operation
// #define RUN_TEST testHoldHeading

MotorPin m1 = {PA8, PA9};
MotorPin m2 = {PA10, PC10};
MotorPin m3 = {PB6, PB7};

MotorDriver motorDriver(m1, m2, m3);
I2CManager i2c;
Compass compass;

constexpr float HEADING_KP = 0.4f;
constexpr float HEADING_KD = 0.3f;
constexpr float MAX_ROTATION = 25.0f;
constexpr float HEADING_DEADZONE = 3.0f;

int main() {
    init();
    analogReadResolution(12);

    motorDriver.init();

    i2c.configure(I2CBus::BUS1, PB9, PA15);
    i2c.init(I2CBus::BUS1);

    compass.init(i2c.getBus(I2CBus::BUS1));

#ifdef RUN_TEST
    RUN_TEST(motorDriver, compass);
#else
    float lastOffset = 0;
    unsigned long lastTime = millis();

    while (true) {
        compass.update();

        unsigned long now = millis();
        float dTimeMs = static_cast<float>(now - lastTime);
        float dTimeS = dTimeMs / 1000.0f;
        lastTime = now;

        // auto [degrees, speed] = readJoystick();

        // const float offset = compass.getOffset();
        // float rotation = 0;
        // if (fabs(offset) > HEADING_DEADZONE) {
        //     float derivative = (dTimeS > 0) ? (offset - lastOffset) / dTimeS : 0;
        //     rotation = constrain(-offset * HEADING_KP - derivative * HEADING_KD, -MAX_ROTATION, MAX_ROTATION);
        // }
        // lastOffset = offset;

        motorDriver.driveDegrees(0, 30, 0);
        motorDriver.updateAllMotors();
        delay(10);
    }
#endif

    return 0;
}
