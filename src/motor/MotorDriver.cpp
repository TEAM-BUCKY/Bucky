#include "MotorDriver.h"
#include "../cordic/cordic.h"
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

    // Register all PWM pins for sync and align timer counters
    pwmSyncRegister(pw1.inA);
    pwmSyncRegister(pw1.inB);
    pwmSyncRegister(pw2.inA);
    pwmSyncRegister(pw2.inB);
    pwmSyncRegister(pw3.inA);
    pwmSyncRegister(pw3.inB);
    pwmSyncTimers();

    const uint32_t currentTime = micros();
    motor1 = {pw1, 0, 0, 0, currentTime};
    motor2 = {pw2, 0, 0, 0, currentTime};
    motor3 = {pw3, 0, 0, 0, currentTime};
}

void MotorDriver::setMotorSpeed(const MotorPwm& motor, const double targetSpeed) {
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
    const double speed = map(static_cast<long>(abs(targetSpeed)), 0, 100, MIN_SPEED, MAX_SPEED);

    if (targetSpeed < 0) {
        pwmWrite(motor.inA, 0);
        pwmWrite(motor.inB, static_cast<int>(round(speed)));
        return;
    }

    pwmWrite(motor.inA, static_cast<int>(round(speed)));
    pwmWrite(motor.inB, 0);
}

constexpr double timePer100 = 30000; // Time required to go from speed 0 to speed 100 in ms
double getSmoothFunction(const double begin, const double target, const double totalSpeed, const uint32_t time) {
    // https://www.desmos.com/calculator/0kpi5zggrd?lang=nl , in desmos: i = begin, o = target, s = totalSpeed, t = timePer100, x = time. (b = beginTime, already implemented inside updateMotor)
    // totalSpeed is used to make different motors sync, it's the scale given in driveDegrees, so if different motors have different speeds, they need the same time to reach targetSpeed
    if (time > abs(begin - totalSpeed) * timePer100) { // Constrain if outside function limits ( {b<x<\frac{\left|i-s\right|}{100}t+b\right\} )
        return target;
    }
    return -(cordic_cos(PI*time*(100/(abs(begin-totalSpeed)*timePer100)))-1)/2 * (target - begin);
}

void MotorDriver::updateMotor(const Motor &motor) { // Update the Motor to drive at the right speed following the smoothing function
    const uint32_t timeSinceBeginSmooth = micros() - motor.beginTimeMs;
    setMotorSpeed(motor.motor, getSmoothFunction(motor.beginSpeed, motor.targetSpeed, motor.totalSpeed, timeSinceBeginSmooth));
}

void MotorDriver::updateAllMotors() const {
    updateMotor(motor1);
    updateMotor(motor2);
    updateMotor(motor3);
}

void MotorDriver::stageMotorSpeed(const MotorPwm& motor, const double targetSpeed) {
    if (targetSpeed > 100 || targetSpeed < -100) {
        Serial.println("Error: speedPercentage must be between -100 and 100");
        return;
    }

    if (targetSpeed == 0) {
        pwmStage(motor.inA, 0);
        pwmStage(motor.inB, 0);
        return;
    }

    const double speed = map(static_cast<long>(abs(targetSpeed)), 0, 100, MIN_SPEED, MAX_SPEED);

    if (targetSpeed < 0) {
        pwmStage(motor.inA, 0);
        pwmStage(motor.inB, static_cast<int>(round(speed)));
        return;
    }

    pwmStage(motor.inA, static_cast<int>(round(speed)));
    pwmStage(motor.inB, 0);
}

void MotorDriver::syncUpdateMotor(const Motor& motor) {
    const uint32_t timeSinceBeginSmooth = micros() - motor.beginTimeMs;
    stageMotorSpeed(motor.motor, getSmoothFunction(motor.beginSpeed, motor.targetSpeed, motor.totalSpeed, timeSinceBeginSmooth));
}

void MotorDriver::syncUpdateAllMotors() const {
    syncUpdateMotor(motor1);
    syncUpdateMotor(motor2);
    syncUpdateMotor(motor3);
    pwmCommit();
}

void MotorDriver::drive(Motor& motor, const double speed, const double totalSpeed) {
    motor.beginSpeed = motor.motor.currentSpeed;
    motor.targetSpeed = speed;
    motor.totalSpeed = totalSpeed;
    motor.beginTimeMs = micros();
}

void MotorDriver::driveDegrees(const double degrees, const double scale, const double rotation) {
    // Translation component: drive in the desired direction
    // Rotation component: scaled by drive speed so correction stays proportional
    // but always allows at least the raw rotation for pure spin (scale=0)
    const double rotationScale = fmax(scale, fabs(rotation)) / 100.0;
    const double scaledRotation = rotation * rotationScale;

    double m1Speed = sin((degrees - 60) * PI / 180) * scale + scaledRotation;
    double m2Speed = -sin(degrees * PI / 180) * scale + scaledRotation;
    double m3Speed = sin((degrees - 300) * PI / 180) * scale + scaledRotation;

    m1Speed = constrain(m1Speed, -100.0, 100.0);
    m2Speed = constrain(m2Speed, -100.0, 100.0);
    m3Speed = constrain(m3Speed, -100.0, 100.0);

    drive(this->motor1, m1Speed, scale);
    drive(this->motor2, m2Speed, scale);
    drive(this->motor3, m3Speed, scale);
}

void MotorDriver::driveVector(const VectorXY vector, const double rotation) {
    const double angle = cordic_atan2(vector.y, vector.x) * 180 / PI;
    const double magnitude = sqrt(pow(vector.x, 2) + pow(vector.y, 2));

    driveDegrees(angle, magnitude, rotation);
}
