#include "Motors.h"
#include <debug.h>
#include <hardware/io/cordic/cordic.h>
#include <cmath>

#include <hardware/motor/MotorDriver.h>
#include <helpers/math/Math.h>
#include <helpers/types.h>
#include <optimizations/logic.h>

constexpr float STOP_DEADBAND = 0.03f;

constexpr float ACCELERATION_TIME_US = 3000000.0f; // Time to ramp from speed 0 to speed 1

static void initPwm(PWMPin& pw, const PinName pin) {
    pw = pwm_pin_init(pin);
    pwm_init(&pw, DRIVER_PWM_FREQ_HZ, DRIVER_PWM_RESOLUTION);
    pwm_write(&pw, 0);
    pwm_sync_register(&pw);
}

// minSpeed/maxSpeed in [0, 1]; enc may be nullptr when no encoders are fitted.
void Motors::init(const Percentage minSpeed, const Percentage maxSpeed, const EncoderPins enc[MOTOR_COUNT])
{
    changeSpeed(minSpeed, maxSpeed);

    const uint32_t currentTime = micros();
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor& m = motors[i];
        m = Motor{};
        m.beginTimeUs = currentTime;
        m.encoderIndex = i;

        DBG_PRINT_SUBJECT(DEBUG_SUBJ_MOTOR, "Initializing motor ");
        DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_MOTOR, i);

        initPwm(m.motor.inA, pins[i].inA);
        initPwm(m.motor.inB, pins[i].inB);
    }
    pwm_sync_timers();

    if (enc != nullptr) {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++)
            encoder_init(i, &enc[i]);
        encodersEnabled = true;
    }
}

template<bool shouldStage>
void FORCE_INLINE writeMotorSpeed(const MotorPWM& motor, const int speedA, const int speedB)
{
    if constexpr (shouldStage) {
        pwm_stage(&motor.inA, speedA);
        pwm_stage(&motor.inB, speedB);
    } else {
        pwm_write(&motor.inA, speedA);
        pwm_write(&motor.inB, speedB);
    }
}

template<bool shouldStage>
void Motors::setMotorSpeed(const MotorPWM& motor, const PercentageSigned targetSpeed) const
{
    if (fabsf(targetSpeed) <= STOP_DEADBAND) {
        writeMotorSpeed<shouldStage>(motor, 0, 0);
        return;
    }

    const auto speed = static_cast<int>(fabsf(targetSpeed) * speedRange.scale + speedRange.min);
    if (targetSpeed < 0)
        writeMotorSpeed<shouldStage>(motor, 0, speed);
    else
        writeMotorSpeed<shouldStage>(motor, speed, 0);
}

// Hermite smoothstep from begin to target over a time proportional to the speed change.
// All wheels of one drive() share the duration (|begin - totalSpeed|), so the direction holds while ramping.
float getSmoothFunction(const float begin, const float target, const float totalSpeed, const uint32_t timeUs)
{
    const float duration = fabsf(begin - totalSpeed) * ACCELERATION_TIME_US;
    const auto floatTime = static_cast<float>(timeUs);

    if (floatTime >= duration)
        return target;

    const float t = floatTime / duration;
    const float smoothStep = t * t * (3 - 2 * t);
    return begin + smoothStep * (target - begin);
}

template<bool shouldStage>
void Motors::updateMotor(Motor& motor) const
{
    const uint32_t timeSinceBeginSmooth = micros() - motor.beginTimeUs;
    const float setpoint = getSmoothFunction(motor.beginSpeed, motor.targetSpeed, motor.totalSpeed, timeSinceBeginSmooth);
    motor.motor.currentSpeed = setpoint;

    // Speeds are kinematic (and so are the encoders); only the H-bridge sees the wiring.
    const float polarity = this->polarity[motor.encoderIndex];

    if (!encodersEnabled || !encoder_is_active(motor.encoderIndex)) {
        setMotorSpeed<shouldStage>(motor.motor, polarity * setpoint);
        return;
    }

    if (fabsf(setpoint) <= STOP_DEADBAND) {
        motor.pi.integral = 0.0f;
        setMotorSpeed<shouldStage>(motor.motor, 0.0f);
        return;
    }

    encoder_update_speed(motor.encoderIndex);

    const float measuredSpeed = encoder_get_speed(motor.encoderIndex) / maxTicksPerSec[motor.encoderIndex];
    const float error = setpoint - measuredSpeed;

    motor.pi.integral = clamp(motor.pi.integral + error, -piIntegralMax, piIntegralMax);
    const float correction = kP * error + kI * motor.pi.integral;

    setMotorSpeed<shouldStage>(motor.motor, polarity * (setpoint + correction));
}

void Motors::updateAllMotors() {

    for (Motor& m : motors) updateMotor<false>(m);
}

void Motors::syncUpdateAllMotors() {

    for (Motor& m : motors) updateMotor<true>(m);
    pwm_commit();
}

void Motors::drive(Motor& motor, const PercentageSigned speed, const Percentage totalSpeed) {
    if (motor.targetSpeed == speed && motor.totalSpeed == totalSpeed)
        return;
    motor.beginSpeed = motor.motor.currentSpeed;
    motor.targetSpeed = speed;
    motor.totalSpeed = totalSpeed;
    motor.beginTimeUs = micros();
    DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_MOTOR, "Motor drive: beginSpeed=" + String(motor.beginSpeed) + ", targetSpeed=" + String(motor.targetSpeed) + ", totalSpeed=" + String(motor.totalSpeed));
}

// sin(d - w) = sin(d)cos(w) - cos(d)sin(w), with the wheel angles' sin/cos written out.
void Motors::wheelSpeeds(const float sinHeading, const float cosHeading, const float scale,
                              const float rotation, float out[MOTOR_COUNT]) {
    out[0] = (0.5f * sinHeading - SIN_60_F * cosHeading) * scale + rotation;   // sin(d - 60°)
    out[1] = -sinHeading * scale + rotation;                                     // sin(d - 180°)
    out[2] = (0.5f * sinHeading + SIN_60_F * cosHeading) * scale + rotation;   // sin(d - 300°)
}

// Clockwise from +y (forward): 0 = forward, 90 = right, 180 = backward, 270 = left.
void Motors::driveDegrees(const float degrees, const Percentage scale, const PercentageSigned rotation) {
    driveRadians(Math::degreesToRadians(degrees), scale, rotation);
}

// Clockwise from +y (forward): 0 = forward, π/2 = right, π = backward, 3π/2 = left.
void Motors::driveRadians(const float radians, const Percentage scale, const PercentageSigned rotation) {
    const Percentage rotationScale = fmaxf(scale, fabsf(rotation)) / 1.0f;

    float sinRadians, cosRadians;
    cordic_sin_cos(radians, &sinRadians, &cosRadians);

    float speeds[MOTOR_COUNT];
    wheelSpeeds(sinRadians, cosRadians, scale, rotation * rotationScale, speeds);

    // Translation + rotation can ask more than 1 of a wheel; scale all down so the direction holds.
    float peak = 1.0f;
    for (const float s : speeds) peak = fmaxf(peak, fabsf(s));

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
        drive(motors[i], speeds[i] / peak, scale);

    DBG_PRINTLN_SUBJECT(DEBUG_SUBJ_MOTOR, "Drive: radians=" + String(radians) + ", scale=" + String(scale) + ", rotation=" + String(rotation));
}

// |vector| is the speed (clamped to 1); rotation in [-1, 1].
void Motors::driveVector(const VectorXY vector, const PercentageSigned rotation) {

    float angleRad, magnitude;
    cordic_atan2_mod(vector.x, vector.y, &angleRad, &magnitude);

    driveRadians(angleRad, magnitude, rotation);
}

void Motors::driveMotorsDirect(const float m1Speed, const float m2Speed, const float m3Speed) {
    const float speeds[MOTOR_COUNT] = {m1Speed, m2Speed, m3Speed};
    const uint32_t now = micros();

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        Motor& m = motors[i];
        const float speed = PercentageSigned(speeds[i]);
        m.beginSpeed = speed;
        m.targetSpeed = speed;
        m.totalSpeed = fabsf(speed);
        m.motor.currentSpeed = speed;
        m.beginTimeUs = now;
    }
}

void Motors::getEncoderSpeeds(float out[3]) const
{
    if (!encodersEnabled) {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++)
            out[i] = motors[i].motor.currentSpeed;
        return;
    }

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        encoder_update_speed(i);
        out[i] = encoder_get_speed(i) / maxTicksPerSec[i];
    }
}

void Motors::getEncoderTicks(uint16_t out[3]) const
{
    if (!encodersEnabled) {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++)
            out[i] = 0;
        return;
    }

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
        out[i] = encoder_get_ticks(i);
}

void Motors::getEncoderValues(MotorEncoderValue out[3]) const
{
    if (!encodersEnabled) {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++)
            out[i] = {0, 0};
        return;
    }

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        encoder_update_speed(i);
        out[i].ticks = encoder_get_ticks(i);
        out[i].speed = encoder_get_speed(i) / maxTicksPerSec[i];
    }
}
