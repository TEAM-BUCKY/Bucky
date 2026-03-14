#ifndef BUCKY_MOTORDRIVER_H
#define BUCKY_MOTORDRIVER_H

#include <Arduino.h>
#include "../gpio/pwm.h"

#define MIN_SPEED 130
#define MAX_SPEED 250

struct MotorPin {
    int inA;
    int inB;
};

struct MotorPwm {
    PwmPin inA;
    PwmPin inB;
    float currentSpeed = 0;
};

struct Motor {
    MotorPwm motor;
    float beginSpeed = 0;
    float targetSpeed = 0;
    float totalSpeed = 0;
    uint32_t beginTimeMs = 0;
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
    MotorPin m1;
    MotorPin m2;
    MotorPin m3;

    MotorPwm pw1;
    MotorPwm pw2;
    MotorPwm pw3;

    Motor motor1;
    Motor motor2;
    Motor motor3;

    SpeedRange speedRange = {MIN_SPEED, MAX_SPEED};

    static void setMotorSpeed(const MotorPwm& motor, float targetSpeed);
    static void updateMotor(const Motor& motor);
    static void drive(Motor& motor, float speed, float totalSpeed);

    void stageMotorSpeed(const MotorPwm& motor, float targetSpeed) const;
    void syncUpdateMotor(const Motor& motor) const;

public:
    MotorDriver(const MotorPin m1, const MotorPin m2, const MotorPin m3) : m1(m1), m2(m2), m3(m3) {};

    void init(int minSpeed = MIN_SPEED, int maxSpeed = MAX_SPEED);

    void updateAllMotors() const;
    void syncUpdateAllMotors() const;

    void driveDegrees(float degrees, float scale = 100, float rotation = 0);
    void driveRadians(float radians, float scale = 100, float rotation = 0);
    void driveVector(VectorXY vector, float rotation = 0);

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
