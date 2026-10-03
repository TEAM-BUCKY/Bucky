#include "MotorDriver.h"
#include "debug.h"
#include "hardware/io/cordic/cordic.h"
#include <cmath>

#include "helpers/Math.h"
#include "optimizations/logic.h"

constexpr float STOP_DEADBAND = 0.05f;

constexpr uint32_t MOTOR_PWM_FREQ_HZ = 5000;
constexpr uint32_t MOTOR_PWM_RESOLUTION = 3399;

constexpr float timePer100 = 30000; // Time required to go from speed 0 to speed 100 in ms

static FORCE_INLINE float clampSpeed(const float speed) {
    return clamp(speed, -100.0f, 100.0f);
}

static void initPwm(PwmPin& pw, const PinName pin) {
    pw = pwm_pin_init(pin);
    pwm_init(&pw, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION);
    pwm_write(&pw, 0);
    pwm_sync_register(&pw);
}

void MotorDriver::init(const float minSpeed, const float maxSpeed, const EncoderPins enc[MOTOR_COUNT])
{
    changeSpeed(minSpeed, maxSpeed);

    const uint32_t currentTime = micros();
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor& m = motors[i];
        m = Motor{};
        m.beginTimeMs = currentTime;
        m.encoderIndex = i;

        DBG_PRINT_SUBJECT(DEBUG_SUBJ_MOTOR, "Initializing motor ");
        DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_MOTOR, i);

        initPwm(m.motor.inA, pins[i].inA);
        initPwm(m.motor.inB, pins[i].inB);
    }
    pwm_sync_timers();

    if (enc != nullptr) {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++)
            encoder_init(i, enc[i]);
        encodersEnabled = true;
    }
}

template<bool stage>
void FORCE_INLINE writeMotorSpeed(const MotorPwm& motor, const int speedA, const int speedB)
{
    if constexpr (stage) {
        pwm_stage(&motor.inA, speedA);
        pwm_stage(&motor.inB, speedB);
    } else {
        pwm_write(&motor.inA, speedA);
        pwm_write(&motor.inB, speedB);
    }
}

template<bool stage>
void MotorDriver::setMotorSpeed(const MotorPwm& motor, const float targetSpeed) const
{
    if (fabsf(targetSpeed) <= STOP_DEADBAND) {
        writeMotorSpeed<stage>(motor, 0, 0);
        return;
    }

    const auto speed = static_cast<int>(fabsf(targetSpeed) * speedRange.scale + speedRange.min);
    if (targetSpeed < 0)
        writeMotorSpeed<stage>(motor, 0, speed);
    else
        writeMotorSpeed<stage>(motor, speed, 0);
}

// Hermite smoothstep from begin to target over a time proportional to the speed change.
float getSmoothFunction(const float begin, const float target, const float totalSpeed, const uint32_t time)
{
    const float duration = fabsf(begin - totalSpeed) * timePer100;
    const auto floatTime = static_cast<float>(time);

    if (floatTime > duration)
        return target;

    const float t = floatTime / duration;
    const float smoothStep = t * t * (3 - 2 * t);
    return begin + smoothStep * (target - begin);
}

template<bool stage>
void MotorDriver::updateMotor(Motor& motor) const
{
    const uint32_t timeSinceBeginSmooth = micros() - motor.beginTimeMs;
    const float setpoint = getSmoothFunction(motor.beginSpeed, motor.targetSpeed, motor.totalSpeed, timeSinceBeginSmooth);
    motor.motor.currentSpeed = setpoint;

    if (!encodersEnabled || !encoder_is_active(motor.encoderIndex)) {
        setMotorSpeed<stage>(motor.motor, setpoint);
        return;
    }

    if (fabsf(setpoint) <= STOP_DEADBAND) {
        motor.pi.integral = 0.0f;
        setMotorSpeed<stage>(motor.motor, 0.0f);
        return;
    }

    encoder_update_speed(motor.encoderIndex);

    const float ticksPerPercent = maxTicksPerSec[motor.encoderIndex] / 100.0f;
    const float measuredSpeed = encoder_get_speed(motor.encoderIndex) / ticksPerPercent;
    const float error = setpoint - measuredSpeed;

    motor.pi.integral = clamp(motor.pi.integral + error, -piIntegralMax, piIntegralMax);
    const float correction = kP * error + kI * motor.pi.integral;

    setMotorSpeed<stage>(motor.motor, clampSpeed(setpoint + correction));
}

void MotorDriver::updateAllMotors() {
    for (Motor& m : motors) updateMotor<false>(m);
}

void MotorDriver::syncUpdateAllMotors() {
    for (Motor& m : motors) updateMotor<true>(m);
    pwm_commit();
}

void MotorDriver::drive(Motor& motor, const float speed, const float totalSpeed) {
    DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_MOTOR, "Motor drive: beginSpeed=" + String(motor.beginSpeed) + ", targetSpeed=" + String(motor.targetSpeed) + ", totalSpeed=" + String(motor.totalSpeed));
    if (motor.targetSpeed == speed && motor.totalSpeed == totalSpeed)
        return;
    motor.beginSpeed = motor.motor.currentSpeed;
    motor.targetSpeed = speed;
    motor.totalSpeed = totalSpeed;
    motor.beginTimeMs = micros();
}

void MotorDriver::wheelSpeeds(const float sinHeading, const float cosHeading, const float scale,
                              const float rotation, float out[MOTOR_COUNT]) {
    out[0] = (0.5f * sinHeading - SIN_60 * cosHeading) * scale + rotation;
    out[1] = -sinHeading * scale + rotation;
    out[2] = (0.5f * sinHeading + SIN_60 * cosHeading) * scale + rotation;
}

void MotorDriver::driveDegrees(const float degrees, const float scale, const float rotation) {
    driveRadians(Math::degreesToRadians(degrees), scale, rotation);
}

void MotorDriver::driveRadians(const float radians, const float scale, const float rotation) {
    const float rotationScale = fmaxf(scale, fabsf(rotation)) / 100.0f;

    float sinRadians, cosRadians;
    cordic_sin_cos(radians, &sinRadians, &cosRadians);

    float speeds[MOTOR_COUNT];
    wheelSpeeds(sinRadians, cosRadians, scale, rotation * rotationScale, speeds);

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
        drive(motors[i], clampSpeed(speeds[i]), scale);

    DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_MOTOR, "Drive: radians=" + String(radians) + ", scale=" + String(scale) + ", rotation=" + String(rotation));
}

void MotorDriver::driveVector(const VectorXY vector, const float rotation) {
    float angleRad, magnitude;
    cordic_atan2_mod(vector.y, vector.x, &angleRad, &magnitude);

    driveRadians(angleRad, magnitude, rotation);
}

void MotorDriver::driveMotorsDirect(const float m1Speed, const float m2Speed, const float m3Speed) {
    const float speeds[MOTOR_COUNT] = {m1Speed, m2Speed, m3Speed};
    const uint32_t now = micros();

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor& m = motors[i];
        const float speed = clampSpeed(speeds[i]);
        m.beginSpeed = speed;
        m.targetSpeed = speed;
        m.totalSpeed = fabsf(speed);
        m.motor.currentSpeed = speed;
        m.beginTimeMs = now;
    }
}

void MotorDriver::getEncoderSpeeds(float out[3]) const
{
    if (!encodersEnabled) {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++)
            out[i] = motors[i].motor.currentSpeed;
        return;
    }

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        encoder_update_speed(i);
        const float ticksPerPercent = maxTicksPerSec[i] / 100.0f;
        out[i] = encoder_get_speed(i) / ticksPerPercent;
    }
}

void MotorDriver::getEncoderTicks(uint16_t out[3]) const
{
    if (!encodersEnabled) {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++)
            out[i] = 0;
        return;
    }

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
        out[i] = encoder_get_ticks(i);
}

void MotorDriver::getEncoderValues(MotorEncoderValue out[3]) const
{
    if (!encodersEnabled) {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++)
            out[i] = {0, 0};
        return;
    }

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        encoder_update_speed(i);
        const float ticksPerPercent = maxTicksPerSec[i] / 100.0f;
        out[i].ticks = encoder_get_ticks(i);
        out[i].speed = encoder_get_speed(i) / ticksPerPercent;
    }
}
