//
// Created by koen on 2/28/26.
//

#include "Joystick.h"

#include <cmath>
#include <pins_arduino_analog.h>
#include <variant_generic.h>
#include <wiring_analog.h>

#include "cordic/cordic.h"
#include "motor/MotorDriver.h"

JoystickVector readJoystick()
{
    const int rawX = analogRead(PB1);
    const int rawY = analogRead(PA7);

    float x = (rawX - JOYSTICK_CENTER) / static_cast<float>(JOYSTICK_CENTER);
    float y = (rawY - JOYSTICK_CENTER) / static_cast<float>(JOYSTICK_CENTER);


    Serial.print("X: ");
    Serial.print(x);
    Serial.print(", Y: ");
    Serial.println(y);

    x = fmaxf(-1.0f, fminf(1.0f, x));
    y = fmaxf(-1.0f, fminf(1.0f, y));

    float magnitude = sqrtf(x * x + y * y);
    constexpr float deadzoneThreshold = DEADZONE / static_cast<float>(JOYSTICK_CENTER);

    if (magnitude < deadzoneThreshold) {
        return {0.0f, 0.0f};
    }

    if (magnitude > 1.0f) magnitude = 1.0f;

    // Remap so deadzone edge = 0, full deflection = 1
    const float scaled = (magnitude - deadzoneThreshold) / (1.0f - deadzoneThreshold);

    // Square for finer low-speed control
    const float curved = scaled * scaled;

    float angle = cordicAtan2(y, x) * (180.0f / PI_F) + ANGLE_OFFSET;
    if (angle < 0)   angle += 360.0f;
    if (angle >= 360) angle -= 360.0f;

    return {angle, curved * 100};
}
