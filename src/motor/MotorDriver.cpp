#include "MotorDriver.h"
#include "debug.h"
#include "io/cordic/cordic.h"
#include <cmath>

#include "helpers/Math.h"
#include "optimizations/logic.h"

void MotorDriver::init(const float minSpeed, const float maxSpeed, const EncoderPins enc[3])
{
    speedRange.min = minSpeed;
    speedRange.max = maxSpeed;
    speedRange.scale = (maxSpeed - minSpeed) / 100.0f;

    pw1 = {pwm_pin_init(m1.inA), pwm_pin_init(m1.inB)};
    pw2 = {pwm_pin_init(m2.inA), pwm_pin_init(m2.inB)};
    pw3 = {pwm_pin_init(m3.inA), pwm_pin_init(m3.inB)};

    pwm_init(&pw1.inA, m1.inA, 5000, 3399);
    pwm_init(&pw1.inB, m1.inB, 5000, 3399);
    pwm_init(&pw2.inA, m2.inA, 5000, 3399);
    pwm_init(&pw2.inB, m2.inB, 5000, 3399);
    pwm_init(&pw3.inA, m3.inA, 5000, 3399);
    pwm_init(&pw3.inB, m3.inB, 5000, 3399);

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

    if (enc != nullptr) {
        for (uint8_t i = 0; i < 3; i++)
            encoder_init(i, enc[i]);
        encodersEnabled = true;
    }

    const uint32_t currentTime = micros();
    motor1 = {pw1, 0, 0, 0, currentTime, 0, {}};
    motor2 = {pw2, 0, 0, 0, currentTime, 1, {}};
    motor3 = {pw3, 0, 0, 0, currentTime, 2, {}};
}

template<bool stage>
void FORCE_INLINE writeMotorSpeed(const MotorPwm& motor, const int speedA, const int speedB)
{
    if constexpr (stage) {
        pwm_stage(&motor.inA, speedA);
        pwm_stage(&motor.inB, speedB);
    } else
    {
        pwm_write(&motor.inA, speedA);
        pwm_write(&motor.inB, speedB);
    }
}

template<bool stage>
void MotorDriver::setMotorSpeed(const MotorPwm& motor, const float targetSpeed) const
{
    // Float noise can produce tiny non-zero commands (e.g. -0.00 in logs).
    // Treat a small band around zero as stop to avoid commanding MIN_SPEED.
    constexpr float kStopDeadband = 0.05f;
    if (fabsf(targetSpeed) <= kStopDeadband)
    {
        writeMotorSpeed<stage>(motor, 0, 0);
        return;
    }

    const float speed = fabsf(targetSpeed) * speedRange.scale + speedRange.min;

    if (targetSpeed < 0) {
        writeMotorSpeed<stage>(motor, 0, static_cast<int>(speed));
        return;
    }

    writeMotorSpeed<stage>(motor, static_cast<int>(speed), 0);
}

constexpr float timePer100 = 30000; // Time required to go from speed 0 to speed 100 in ms

// New implementation using Hermite smoothstep
float getSmoothFunction(const float begin, const float target, const float totalSpeed, const uint32_t time)
{
    const float difference = fabsf(begin - totalSpeed);
    const auto floatTime = static_cast<float>(time);
    
    if (floatTime > difference * timePer100)
        return target;

    const float t = floatTime / (difference * timePer100); // Normalize time to [0, 1]
    const float smoothStep = t * t * (3 - 2 * t); // Hermite smoothstep function
    return begin + smoothStep * (target - begin);
}

template<bool stage>
void MotorDriver::updateMotor(Motor &motor) const
{
    const uint32_t timeSinceBeginSmooth = micros() - motor.beginTimeMs;
    const float setpoint = getSmoothFunction(motor.beginSpeed, motor.targetSpeed, motor.totalSpeed, timeSinceBeginSmooth);
    motor.motor.currentSpeed = setpoint;

    if (!encodersEnabled || !encoder_is_active(motor.encoderIndex)) {
        setMotorSpeed<stage>(motor.motor, setpoint);
        return;
    }

    constexpr float kStopDeadband = 0.05f;
    if (fabsf(setpoint) <= kStopDeadband) {
        motor.pi.integral = 0.0f;
        setMotorSpeed<stage>(motor.motor, 0.0f);
        return;
    }

    encoder_update_speed(motor.encoderIndex);

    const float ticksPerPercent = maxTicksPerSec[motor.encoderIndex] / 100.0f;
    const float measuredSpeed = encoder_get_speed(motor.encoderIndex) / ticksPerPercent;
    const float error = setpoint - measuredSpeed;

    motor.pi.integral = clampf(motor.pi.integral + error, -piIntegralMax, piIntegralMax);
    const float correction = kP * error + kI * motor.pi.integral;

    const float output = clampf(setpoint + correction, -100.0f, 100.0f);
    setMotorSpeed<stage>(motor.motor, output);
}

void MotorDriver::updateAllMotors() {
    updateMotor<false>(motor1);
    updateMotor<false>(motor2);
    updateMotor<false>(motor3);
}

void MotorDriver::syncUpdateMotor(Motor& motor) const
{
    const uint32_t timeSinceBeginSmooth = micros() - motor.beginTimeMs;
    const float setpoint = getSmoothFunction(motor.beginSpeed, motor.targetSpeed, motor.totalSpeed, timeSinceBeginSmooth);
    motor.motor.currentSpeed = setpoint;

    if (!encodersEnabled || !encoder_is_active(motor.encoderIndex)) {
        setMotorSpeed<true>(motor.motor, setpoint);
        return;
    }

    constexpr float kStopDeadband = 0.05f;
    if (fabsf(setpoint) <= kStopDeadband) {
        motor.pi.integral = 0.0f;
        setMotorSpeed<true>(motor.motor, 0.0f);
        return;
    }

    encoder_update_speed(motor.encoderIndex);

    const float ticksPerPercent = maxTicksPerSec[motor.encoderIndex] / 100.0f;
    const float measuredSpeed = encoder_get_speed(motor.encoderIndex) / ticksPerPercent;
    const float error = setpoint - measuredSpeed;

    motor.pi.integral = clampf(motor.pi.integral + error, -piIntegralMax, piIntegralMax);
    const float correction = kP * error + kI * motor.pi.integral;

    const float output = clampf(setpoint + correction, -100.0f, 100.0f);
    setMotorSpeed<true>(motor.motor, output);
}

void MotorDriver::syncUpdateAllMotors() {
    syncUpdateMotor(motor1);
    syncUpdateMotor(motor2);
    syncUpdateMotor(motor3);
    pwm_commit();
}

void MotorDriver::drive(Motor& motor, const float speed, const float totalSpeed) {
    DBG_PRINTLN("Motor drive: beginSpeed=" + String(motor.beginSpeed) + ", targetSpeed=" + String(motor.targetSpeed) + ", totalSpeed=" + String(motor.totalSpeed));
    if (motor.targetSpeed == speed && motor.totalSpeed == totalSpeed)
        return;
    motor.beginSpeed = motor.motor.currentSpeed;
    motor.targetSpeed = speed;
    motor.totalSpeed = totalSpeed;
    motor.beginTimeMs = micros();

}

void MotorDriver::driveDegrees(const float degrees, const float scale, const float rotation) {
    driveRadians(Math::degreesToRadians(degrees), scale, rotation);
}

constexpr float SIN_60 = 0.8660254037844f;

// `radians` is expected to be in radians; `driveDegrees` converts before calling this.
void MotorDriver::driveRadians(const float radians, const float scale, const float rotation) {
    const float rotationScale = fmaxf(scale, fabsf(rotation)) / 100.0f;
    const float scaledRotation = rotation * rotationScale;

    float sinRadians, cosRadians;
    cordic_sin_cos(radians, &sinRadians, &cosRadians);

    float m1Speed = (0.5f * sinRadians - SIN_60 * cosRadians) * scale + scaledRotation;
    float m2Speed = -sinRadians * scale + scaledRotation;
    float m3Speed = (0.5f * sinRadians + SIN_60 * cosRadians) * scale + scaledRotation;

    m1Speed = clampf(m1Speed, -100.0f, 100.0f);
    m2Speed = clampf(m2Speed, -100.0f, 100.0f);
    m3Speed = clampf(m3Speed, -100.0f, 100.0f);

    drive(this->motor1, m1Speed, scale);
    drive(this->motor2, m2Speed, scale);
    drive(this->motor3, m3Speed, scale);

    DBG_PRINTLN("");
}

void MotorDriver::driveVector(const VectorXY vector, const float rotation) {
    float angleRad, magnitude;
    cordic_atan2_mod(vector.y, vector.x, &angleRad, &magnitude);

    driveRadians(angleRad, magnitude, rotation);
}

void MotorDriver::driveMotorsDirect(float m1Speed, float m2Speed, float m3Speed) {
    m1Speed = clampf(m1Speed, -100.0f, 100.0f);
    m2Speed = clampf(m2Speed, -100.0f, 100.0f);
    m3Speed = clampf(m3Speed, -100.0f, 100.0f);

    auto set = [](Motor& m, const float speed) {
        m.beginSpeed = speed;
        m.targetSpeed = speed;
        m.totalSpeed = fabsf(speed);
        m.motor.currentSpeed = speed;
        m.beginTimeMs = micros();
    };
    set(motor1, m1Speed);
    set(motor2, m2Speed);
    set(motor3, m3Speed);
}
