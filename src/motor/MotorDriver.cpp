#include "MotorDriver.h"
#include "debug.h"
#include "io/cordic/cordic.h"
#include <cmath>

void MotorDriver::init(const int minSpeed, const int maxSpeed)
{
    speedRange.min = minSpeed;
    speedRange.max = maxSpeed;

    pw1 = {pwm_pin_init(m1.inA), pwm_pin_init(m1.inB)};
    pw2 = {pwm_pin_init(m2.inA), pwm_pin_init(m2.inB)};
    pw3 = {pwm_pin_init(m3.inA), pwm_pin_init(m3.inB)};

    pwm_init(&pw1.inA, m1.inA, 1000, 255);
    pwm_init(&pw1.inB, m1.inB, 1000, 255);
    pwm_init(&pw2.inA, m2.inA, 1000, 255);
    pwm_init(&pw2.inB, m2.inB, 1000, 255);
    pwm_init(&pw3.inA, m3.inA, 1000, 255);
    pwm_init(&pw3.inB, m3.inB, 1000, 255);

    pwm_write(&pw1.inA, 0);
    pwm_write(&pw1.inB, 0);
    pwm_write(&pw2.inA, 0);
    pwm_write(&pw2.inB, 0);
    pwm_write(&pw3.inA, 0);
    pwm_write(&pw3.inB, 0);

    // Register all PWM pins for sync and align timer counters
    pwm_sync_register(&pw1.inA);
    pwm_sync_register(&pw1.inB);
    pwm_sync_register(&pw2.inA);
    pwm_sync_register(&pw2.inB);
    pwm_sync_register(&pw3.inA);
    pwm_sync_register(&pw3.inB);
    pwm_sync_timers();

    const uint32_t currentTime = micros();
    motor1 = {pw1, 0, 0, 0, currentTime};
    motor2 = {pw2, 0, 0, 0, currentTime};
    motor3 = {pw3, 0, 0, 0, currentTime};
}

void MotorDriver::setMotorSpeed(const MotorPwm& motor, const float targetSpeed) {
    if (targetSpeed > 100 || targetSpeed < -100) {
        DBG_PRINTLN("Error: speedPercentage must be between -100 and 100");
        return;
    }

    if (targetSpeed == 0)
    {
        pwm_write(&motor.inA, 0);
        pwm_write(&motor.inB, 0);
        return;
    }
    // Map -100%-100% to an actual speed value
    const float speed = map(static_cast<long>(abs(targetSpeed)), 0, 100, MIN_SPEED, MAX_SPEED);

    if (targetSpeed < 0) {
        pwm_write(&motor.inA, 0);
        pwm_write(&motor.inB, static_cast<int>(round(speed)));
        return;
    }

    pwm_write(&motor.inA, static_cast<int>(round(speed)));
    pwm_write(&motor.inB, 0);
}

constexpr float timePer100 = 30000; // Time required to go from speed 0 to speed 100 in ms

// New implementation using Hermite smoothstep
float getSmoothFunction(const float begin, const float target, const float totalSpeed, const uint32_t time)
{
    if (time > fabsf(begin - totalSpeed) * timePer100) {
        return target;
    }
    const float t = time / (fabsf(begin - totalSpeed) * timePer100); // Normalize time to [0, 1]
    const float smoothStep = t * t * (3 - 2 * t); // Hermite smoothstep function
    return begin + smoothStep * (target - begin);
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

void MotorDriver::stageMotorSpeed(const MotorPwm& motor, const float targetSpeed) const {
    if (targetSpeed > 100 || targetSpeed < -100) {
        DBG_PRINTLN("Error: speedPercentage must be between -100 and 100");
        return;
    }

    if (targetSpeed == 0) {
        pwm_stage(&motor.inA, 0);
        pwm_stage(&motor.inB, 0);
        return;
    }

    const float speed = abs(targetSpeed) * (speedRange.max - speedRange.min) / 100.0f + speedRange.min;

    if (targetSpeed < 0) {
        pwm_stage(&motor.inA, 0);
        pwm_stage(&motor.inB, static_cast<int>(round(speed)));
        return;
    }

    pwm_stage(&motor.inA, static_cast<int>(round(speed)));
    pwm_stage(&motor.inB, 0);
}

void MotorDriver::syncUpdateMotor(const Motor& motor) const
{
    const uint32_t timeSinceBeginSmooth = micros() - motor.beginTimeMs;
    stageMotorSpeed(motor.motor, getSmoothFunction(motor.beginSpeed, motor.targetSpeed, motor.totalSpeed, timeSinceBeginSmooth));
}

void MotorDriver::syncUpdateAllMotors() const {
    syncUpdateMotor(motor1);
    syncUpdateMotor(motor2);
    syncUpdateMotor(motor3);
    pwm_commit();
}

void MotorDriver::drive(Motor& motor, const float speed, const float totalSpeed) {
    motor.beginSpeed = motor.motor.currentSpeed;
    motor.targetSpeed = speed;
    motor.totalSpeed = totalSpeed;
    motor.beginTimeMs = micros();
}

constexpr float SIN_60 = 0.8660254037844f;

void MotorDriver::driveDegrees(const float degrees, const float scale, const float rotation) {
    const float rotationScale = fmaxf(scale, fabsf(rotation)) / 100.0f;
    const float scaledRotation = rotation * rotationScale;

    float sinDegrees, cosDegrees;
    cordic_sin_cos(degrees * PI_F / 180, &sinDegrees, &cosDegrees);

    float m1Speed = (0.5f * sinDegrees - SIN_60 * cosDegrees) * scale + scaledRotation;
    float m2Speed = -sinDegrees * scale + scaledRotation;
    float m3Speed = (0.5f * sinDegrees + SIN_60 * cosDegrees) * scale + scaledRotation;

    m1Speed = constrain(m1Speed, -100.0f, 100.0f);
    m2Speed = constrain(m2Speed, -100.0f, 100.0f);
    m3Speed = constrain(m3Speed, -100.0f, 100.0f);

    drive(this->motor1, m1Speed, scale);
    drive(this->motor2, m2Speed, scale);
    drive(this->motor3, m3Speed, scale);
}

void MotorDriver::driveRadians(const float radians, const float scale, const float rotation) {
    const float rotationScale = fmaxf(scale, fabsf(rotation)) / 100.0f;
    const float scaledRotation = rotation * rotationScale;

    float sinDegrees, cosDegrees;
    cordic_sin_cos(radians, &sinDegrees, &cosDegrees);

    float m1Speed = (0.5f * sinDegrees - SIN_60 * cosDegrees) * scale + scaledRotation;
    float m2Speed = -sinDegrees * scale + scaledRotation;
    float m3Speed = (0.5f * sinDegrees + SIN_60 * cosDegrees) * scale + scaledRotation;

    m1Speed = constrain(m1Speed, -100.0f, 100.0f);
    m2Speed = constrain(m2Speed, -100.0f, 100.0f);
    m3Speed = constrain(m3Speed, -100.0f, 100.0f);

    drive(this->motor1, m1Speed, scale);
    drive(this->motor2, m2Speed, scale);
    drive(this->motor3, m3Speed, scale);
}

void MotorDriver::driveVector(const VectorXY vector, const float rotation) {
    float angle, magnitude;
    cordic_atan2_mod(vector.y, vector.x, &angle, &magnitude);

    driveRadians(angle, magnitude, rotation);
}
