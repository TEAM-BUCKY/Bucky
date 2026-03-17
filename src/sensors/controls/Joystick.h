#ifndef BUCKY_JOYSTICK_H
#define BUCKY_JOYSTICK_H

constexpr int JOYSTICK_CENTER = 2048;
constexpr int DEADZONE = 100;
constexpr float ANGLE_OFFSET = 135.0f;

struct JoystickVector {
    float degrees;
    float speed;
};

JoystickVector readJoystick();

#endif //BUCKY_JOYSTICK_H