#ifndef BUCKY_MOTORDRIVER_H
#define BUCKY_MOTORDRIVER_H

#include <Arduino.h>

#include <hardware/motor/MotorDriver.h>
#include <hardware/io/gpio/pwm.h>
#include <hardware/io/encoder/Encoder.h>
#include <helpers/types.h>

// Pins may carry an _ALTn suffix to select the timer (e.g. PB_14_ALT2 = TIM12).
struct MotorPin {
    PinName inA;
    PinName inB;
};

struct MotorPWM {
    PWMPin inA{};
    PWMPin inB{};
    float currentSpeed = 0;
};

struct MotorPI {
    float integral = 0.0f;
};

struct MotorEncoderValue
{
    uint16_t ticks = 0;
    float speed = 0;
};

struct Motor {
    MotorPWM motor;
    float beginSpeed = 0;
    float targetSpeed = 0;
    float totalSpeed = 0;
    uint32_t beginTimeUs = 0;
    uint8_t encoderIndex = 0;
    MotorPI pi;
};

struct VectorXY
{
    float x;
    float y;
};

// PWM counts: |speed| in (0, 1] maps to min + |speed| * (max - min).
struct SpeedRange {
    float min;
    float max;
    float scale = 1;   // max - min
};

class Motors {
public:
    static constexpr uint8_t MOTOR_COUNT = 3;

    // Wheel positions in driveDegrees' angle convention (wheel A = M1, B = M2, C = M3).
    static constexpr float WHEEL_ANGLES_DEG[MOTOR_COUNT] = {60.0f, 180.0f, 300.0f};

    // Per wheel: sin(d - wheelAngle) * scale + rotation, with d the driving direction.
    static void wheelSpeeds(float sinHeading, float cosHeading, float scale, float rotation,
                            float out[MOTOR_COUNT]);

private:
    MotorPin pins[MOTOR_COUNT];
    Motor motors[MOTOR_COUNT];

    SpeedRange speedRange = {0.0f, DRIVER_PWM_RESOLUTION * 1.0f, DRIVER_PWM_RESOLUTION * 1.0f};
    float polarity[MOTOR_COUNT] = {1.0f, 1.0f, 1.0f};

    bool encodersEnabled = false;
    float kP = 0.5f;
    float kI = 0.05f;
    float piIntegralMax = 0.3f;
    float maxTicksPerSec[MOTOR_COUNT] = {1200.0f, 1200.0f, 1200.0f};

    template<bool stage>
    void setMotorSpeed(const MotorPWM& motor, PercentageSigned targetSpeed) const;
    template<bool stage>
    void updateMotor(Motor& motor) const;
    static void drive(Motor& motor, PercentageSigned speed, Percentage totalSpeed);

public:
    Motors(const MotorPin m1, const MotorPin m2, const MotorPin m3) : pins{m1, m2, m3} {}

    void init(Percentage minSpeed = 0.0f, Percentage maxSpeed = 1.0f,
              const EncoderPins enc[MOTOR_COUNT] = nullptr);

    // -1 for a motor wired reversed, so +speed turns every wheel the same way (see WHEEL_ANGLES_DEG).
    void setPolarity(const float p[MOTOR_COUNT]) {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) polarity[i] = p[i] < 0 ? -1.0f : 1.0f;
    }

    void updateAllMotors();
    void syncUpdateAllMotors();

    void driveDegrees(float degrees, Percentage scale = 1.0f, PercentageSigned rotation = 0.0f);
    void driveRadians(float radians, Percentage scale = 1.0f, PercentageSigned rotation = 0.0f);
    void driveVector(VectorXY vector, PercentageSigned rotation = 0.0f);
    void driveMotorsDirect(float m1Speed, float m2Speed, float m3Speed);

    void changeSpeed(const float minSpeed = 0.0f, const float maxSpeed = 1.0f) {
        const float absoluteMinSpeed = minSpeed * DRIVER_PWM_RESOLUTION;
        const float absoluteMaxSpeed = maxSpeed * DRIVER_PWM_RESOLUTION;
        speedRange = {absoluteMinSpeed, absoluteMaxSpeed, absoluteMaxSpeed - absoluteMinSpeed};
    }

    [[nodiscard]] SpeedRange getSpeedRange() const
    {
        return this->speedRange;
    }

    void setPIGains(const float kp, const float ki, const float iMax) {
        this->kP = kp;
        this->kI = ki;
        this->piIntegralMax = iMax;
    }

    void setMaxTicksPerSec(const float tps) {
        for (float& max : maxTicksPerSec) max = tps;
    }

    void setMaxTicksPerSec(const uint8_t motor, const float tps) {
        if (motor < MOTOR_COUNT) maxTicksPerSec[motor] = tps;
    }

    void getEncoderSpeeds(float out[MOTOR_COUNT]) const;
    void getEncoderTicks(uint16_t out[MOTOR_COUNT]) const;
    void getEncoderValues(MotorEncoderValue out[MOTOR_COUNT]) const;
};

#endif //BUCKY_MOTORDRIVER_H