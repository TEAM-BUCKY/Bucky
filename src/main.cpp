#include <Arduino.h>
#include <cmath>

#include "motor/MotorDriver.h"
#include "pos/Compass.h"
#include "pos/Sonar.h"

const int JOYSTICK_CENTER = 2048;
const int DEADZONE = 100;
const float ANGLE_OFFSET = 135.0f;

struct JoystickVector {
    float degrees;
    float speed;
};

JoystickVector readJoystick() {
    int rawX = analogRead(PB1);
    int rawY = analogRead(PA7);

    float x = (rawX - JOYSTICK_CENTER) / (float)JOYSTICK_CENTER;
    float y = (rawY - JOYSTICK_CENTER) / (float)JOYSTICK_CENTER;

    x = fmaxf(-1.0f, fminf(1.0f, x));
    y = fmaxf(-1.0f, fminf(1.0f, y));

    float magnitude = sqrtf(x * x + y * y);
    float deadzoneThreshold = DEADZONE / (float)JOYSTICK_CENTER;

    if (magnitude < deadzoneThreshold) {
        return {0.0f, 0.0f};
    }

    if (magnitude > 1.0f) magnitude = 1.0f;

    // Remap so deadzone edge = 0, full deflection = 1
    float scaled = (magnitude - deadzoneThreshold) / (1.0f - deadzoneThreshold);

    // Square for finer low-speed control
    float curved = scaled * scaled;

    float angle = atan2f(y, x) * (180.0f / M_PI) + ANGLE_OFFSET;
    if (angle < 0)   angle += 360.0f;
    if (angle >= 360) angle -= 360.0f;

    return {angle, curved * MAX_SPEED};
}

Motor m1 = {PA8, PA9};
Motor m2 = {PA10, PC10};
Motor m3 = {PB6, PB7};

MotorDriver motorDriver(m1, m2, m3);

int main() {
    init();
    analogReadResolution(12);

    motorDriver.init();

    while (true) {
        auto [degrees, speed] = readJoystick();
        motorDriver.driveDegrees(0, 180);

        delay(10);
    }

    return 0;
}