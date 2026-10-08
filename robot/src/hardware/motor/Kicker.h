#ifndef ROBOT_KICKER_H
#define ROBOT_KICKER_H
#include "Motors.h"

class Kicker {
    PinName kickerPin;
    PwmPin kickerPwm;

public:
    explicit Kicker(const PinName pin) : kickerPin(pin) {}

    void init();

    void kick(float speed) const;
    void stop() const;
};

#endif //ROBOT_KICKER_H