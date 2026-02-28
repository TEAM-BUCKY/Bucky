#include <Arduino.h>
#include <cmath>

#include "controls/Joystick.h"
#include "motor/MotorDriver.h"
#include "i2c/I2CManager.h"
#include "pos/Compass.h"
#include "pos/Sonar.h"

Motor m1 = {PA8, PA9};
Motor m2 = {PA10, PC10};
Motor m3 = {PB6, PB7};

MotorDriver motorDriver(m1, m2, m3);
I2CManager i2c;
Compass compass;

[[noreturn]] int main() {
    init();
    analogReadResolution(12);

    motorDriver.init();

    i2c.configure(I2CBus::BUS1, PB9, PA15);
    i2c.init(I2CBus::BUS1);

    compass.init(i2c.getBus(I2CBus::BUS1));

    while (true) {
        compass.update();

        auto [degrees, speed] = readJoystick();
        motorDriver.applyBalanceDegrees(-compass.getHeading());
        motorDriver.driveDegrees(degrees, speed);

        delay(10);
    }
}
