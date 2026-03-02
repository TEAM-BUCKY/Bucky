#include "MotorDriver.h"
#include <cmath>

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

    pwmWrite(pw1.inA, 0);
    pwmWrite(pw1.inB, 0);
    pwmWrite(pw2.inA, 0);
    pwmWrite(pw2.inB, 0);
    pwmWrite(pw3.inA, 0);
    pwmWrite(pw3.inB, 0);
}

void MotorDriver::drive(const MotorPwm& motor, const double targetSpeed) {
    if (targetSpeed > 100 || targetSpeed < -100) {
        Serial.println("Error: speedPercentage must be between -100 and 100");
        return;
    }

    if (targetSpeed == 0)
    {
        pwmWrite(motor.inA, 0);
        pwmWrite(motor.inB, 0);
        return;
    }
    // Map -100%-100% to an actual speed value
    const double speed = map(abs(targetSpeed), 0, 100, MIN_SPEED, MAX_SPEED);

    if (targetSpeed < 0) {
        pwmWrite(motor.inA, 0);
        pwmWrite(motor.inB, static_cast<int>(round(speed)));
        return;
    }

    pwmWrite(motor.inA, static_cast<int>(round(speed)));
    pwmWrite(motor.inB, 0);
}


void MotorDriver::driveDegrees(const double degrees, const double scale, const double rotation) const {
    // Translation component: drive in the desired direction
    // Rotation component: scaled by drive speed so correction stays proportional,
    // but always allows at least the raw rotation for pure spin (scale=0)
    const double rotationScale = fmax(scale, fabs(rotation)) / 100.0;
    const double scaledRotation = rotation * rotationScale;

    double m1Speed = sin((degrees - 60) * PI / 180) * scale + scaledRotation;
    double m2Speed = -sin(degrees * PI / 180) * scale + scaledRotation;
    double m3Speed = sin((degrees - 300) * PI / 180) * scale + scaledRotation;

    m1Speed = constrain(m1Speed, -100.0, 100.0);
    m2Speed = constrain(m2Speed, -100.0, 100.0);
    m3Speed = constrain(m3Speed, -100.0, 100.0);

    drive(this->pw1, m1Speed);
    drive(this->pw2, m2Speed);
    drive(this->pw3, m3Speed);
}

void MotorDriver::driveVector(const VectorXY vector, const double rotation) {
    const double angle = atan2(vector.y, vector.x) * 180 / PI;
    const double magnitude = sqrt(pow(vector.x, 2) + pow(vector.y, 2));

    driveDegrees(angle, magnitude, rotation);
}
