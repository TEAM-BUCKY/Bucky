#ifndef BUCKY_MOTORDRIVER_H
#define BUCKY_MOTORDRIVER_H

#include <Arduino.h>
#include "../gpio/pwm.h"

#define MIN_SPEED 150
#define MAX_SPEED 250

struct Motor {
    int inA;
    int inB;
};

struct MotorPwm {
    PwmPin inA;
    PwmPin inB;
};

struct VectorXY
{
    float x;
    float y;
};

struct SpeedRange {
    int min;
    int max;
};

class MotorDriver {
    Motor m1;
    Motor m2;
    Motor m3;

    MotorPwm pw1;
    MotorPwm pw2;
    MotorPwm pw3;

    SpeedRange speedRange = {MIN_SPEED, MAX_SPEED};

    double balanceDegrees = 0;

    static void drive(const MotorPwm& motor, double speedPercentage);

public:
    MotorDriver(const Motor m1, const Motor m2, const Motor m3) : m1(m1), m2(m2), m3(m3) {};

    void init(int minSpeed = MIN_SPEED, int maxSpeed = MAX_SPEED);

    void driveDegrees(double degrees, double scale = 100) const;
    void driveVector(VectorXY vector) const;
    void applyBalanceDegrees(double degreesOffset);
    void applyBalanceVector(VectorXY balanceVector);

    void changeSpeed(const int minSpeed = MIN_SPEED, const int maxSpeed = MAX_SPEED) {
        this->speedRange.min = minSpeed;
        this->speedRange.max = maxSpeed;
    }

    [[nodiscard]] SpeedRange getSpeedRange() const
    {
        return this->speedRange;
    }


};

#endif //BUCKY_MOTORDRIVER_H
