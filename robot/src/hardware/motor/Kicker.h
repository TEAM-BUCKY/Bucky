#ifndef ROBOT_KICKER_H
#define ROBOT_KICKER_H
#include <hardware/motor/Motors.h>

class Kicker {
    PinName kickerPin;
    PWMPin kickerPWM;

public:
    explicit Kicker(const PinName pin) : kickerPin(pin) {}

    void init();

    void kick(Percentage speed) const;
    void stop() const;
};

#endif //ROBOT_KICKER_H