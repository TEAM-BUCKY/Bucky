#include "MotorDriver.h"

void MotorDriver::init(const int minSpeed, const int maxSpeed)
{
    speedRange.min = minSpeed;
    speedRange.max = maxSpeed;

    pinMode(m1.inA, OUTPUT);
    pinMode(m1.inB, OUTPUT);
    pinMode(m2.inA, OUTPUT);
    pinMode(m2.inB, OUTPUT);
    pinMode(m3.inA, OUTPUT);
    pinMode(m3.inB, OUTPUT);
}

void MotorDriver::drive(const Motor motor, const double speedPercentage) {
    if (speedPercentage > 100 || speedPercentage < -100) {
        Serial.println("Error: speedPercentage must be between -100 and 100");
        return;
    }

    if (speedPercentage == 0)
    {
        analogWrite(motor.inA, 0);
        analogWrite(motor.inB, 0);
        return;
    }
    // Map -100%-100% to an actual speed value
    const double speed = map(abs(speedPercentage), -100, 100, MIN_SPEED, MAX_SPEED);

    if (speedPercentage < 0) {
        analogWrite(motor.inA, 0);
        analogWrite(motor.inB, static_cast<int>(round(speed)));
        return;
    }

    analogWrite(motor.inA, static_cast<int>(round(speed)));
    analogWrite(motor.inB, 0);
}

void MotorDriver::driveDegrees(double degrees, const double scale) const
{
    // Apply balance degrees to the input degrees
    degrees += balanceDegrees;

    const double m1Speed = sin((degrees - 60) * PI / 180) * scale;
    const double m2Speed = -sin(degrees * PI / 180) * scale;
    const double m3Speed = sin((degrees - 300) * PI / 180) * scale;

    drive(this->m1, m1Speed);
    drive(this->m2, m2Speed);
    drive(this->m3, m3Speed);
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