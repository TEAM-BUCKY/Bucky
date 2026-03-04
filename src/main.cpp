#include <Arduino.h>
#include <cmath>

#include "controls/Joystick.h"
#include "motor/MotorDriver.h"
#include "i2c/I2CManager.h"
#include "pos/Compass.h"
#include "pos/Sonar.h"
#include "ir/IRController.h"
#include "tests/tests.h"

// Uncomment to run a test instead of normal operation
#define RUN_TEST testDriveForward

Motor m1 = {PA8, PA9};
Motor m2 = {PA10, PC10};
Motor m3 = {PB6, PB7};

MotorDriver motorDriver(m1, m2, m3);
I2CManager i2c;
Compass compass;
IRController irController;

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

    i2c.configure(I2CBus::BUS3, PC9, PC8);  // SDA3=PC9, SCL3=PC8
    i2c.init(I2CBus::BUS3);

    compass.init(i2c.getBus(I2CBus::BUS1));
    irController.init(i2c.getBus(I2CBus::BUS3));

#ifdef RUN_TEST
    RUN_TEST(motorDriver, compass, irController, i2c);
#else
    float lastOffset = 0;
    unsigned long lastTime = millis();

    while (true) {
        compass.update();

        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;

        auto [degrees, speed] = readJoystick();

        const float offset = compass.getOffset();
        float rotation = 0;
        if (fabs(offset) > HEADING_DEADZONE) {
            float derivative = (dt > 0) ? (offset - lastOffset) / dt : 0;
            rotation = constrain(-offset * HEADING_KP - derivative * HEADING_KD, -MAX_ROTATION, MAX_ROTATION);
        }
        lastOffset = offset;

        motorDriver.driveDegrees(degrees, speed, rotation);

        delay(10);
    }
#endif

    return 0;
}
