#include "MotorDriver.h"

void MotorDriver::init(const int minSpeed, const int maxSpeed)
{
    speedRange.min = minSpeed;
    speedRange.max = maxSpeed;

    pw1 = {PwmPin(m1.inA), PwmPin(m1.inB)};
    pw2 = {PwmPin(m2.inA), PwmPin(m2.inB)};
    pw3 = {PwmPin(m3.inA), PwmPin(m3.inB)};

    pwmInit(pw1.inA, m1.inA);
    pwmInit(pw1.inB, m1.inB);
    pwmInit(pw2.inA, m2.inA);
    pwmInit(pw2.inB, m2.inB);
    pwmInit(pw3.inA, m3.inA);
    pwmInit(pw3.inB, m3.inB);
}

void MotorDriver::drive(const MotorPwm& motor, const double speedPercentage) {
    if (speedPercentage > 100 || speedPercentage < -100) {
        Serial.println("Error: speedPercentage must be between -100 and 100");
        return;
    }

    if (speedPercentage == 0)
    {
        pwmWrite(motor.inA, 0);
        pwmWrite(motor.inB, 0);
        return;
    }
    // Map -100%-100% to an actual speed value
    const double speed = map(abs(speedPercentage), -100, 100, MIN_SPEED, MAX_SPEED);

    if (speedPercentage < 0) {
        pwmWrite(motor.inA, 0);
        pwmWrite(motor.inB, static_cast<int>(round(speed)));
        return;
    }

    pwmWrite(motor.inA, static_cast<int>(round(speed)));
    pwmWrite(motor.inB, 0);
}

void MotorDriver::driveDegrees(double degrees, const double scale) const
{
    // Apply balance degrees to the input degrees
    degrees += balanceDegrees;

    const double m1Speed = sin((degrees - 60) * PI / 180) * scale;
    const double m2Speed = -sin(degrees * PI / 180) * scale;
    const double m3Speed = sin((degrees - 300) * PI / 180) * scale;

    drive(this->pw1, m1Speed);
    drive(this->pw2, m2Speed);
    drive(this->pw3, m3Speed);
}

void MotorDriver::driveVector(const VectorXY vector) const {
    const double angle = atan2(vector.y, vector.x) * 180 / PI;
    const double magnitude = sqrt(pow(vector.x, 2) + pow(vector.y, 2));

    driveDegrees(angle, magnitude);
}

void MotorDriver::applyBalanceDegrees(const double degreesOffset)
{
    if (round(degreesOffset) == 0) balanceDegrees = 0;
    else balanceDegrees = degreesOffset;
}

void MotorDriver::applyBalanceVector(const VectorXY balanceVector)
{
    const double angle = atan2(balanceVector.y, balanceVector.x) * 180 / PI;

    applyBalanceDegrees(angle);
}
