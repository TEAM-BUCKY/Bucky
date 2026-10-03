#ifndef ROBOT_KICKER_H
#define ROBOT_KICKER_H
#include "MotorDriver.h"

class KickerDriver {
    PinName kickerPin;
    PwmPin kickerPwm;

    SpeedRange speedRange = {MIN_SPEED, MAX_SPEED, (MAX_SPEED - MIN_SPEED) / 100.0f};




public:
    explicit KickerDriver(const PinName pin) : kickerPin(pin) {}

    void init(float minSpeed = MIN_SPEED, float maxSpeed = MAX_SPEED);

    void kick(float speed) const;
    void stop() const;

    void changeSpeed(const float minSpeed = MIN_SPEED, const float maxSpeed = MAX_SPEED) {
        speedRange = {minSpeed, maxSpeed, (maxSpeed - minSpeed) / 100.0f};
    }


};

#endif //ROBOT_KICKER_H