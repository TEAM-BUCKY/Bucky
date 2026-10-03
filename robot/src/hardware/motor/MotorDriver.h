#ifndef BUCKY_MOTORDRIVER_H
#define BUCKY_MOTORDRIVER_H

#include <Arduino.h>
#include "hardware/io/gpio/pwm.h"
#include "hardware/io/encoder/Encoder.h"

#define MIN_SPEED 1300
#define MAX_SPEED 3399

// Pins may carry an _ALTn suffix to select the timer (e.g. PB_14_ALT2 = TIM12).
struct MotorPin {
    PinName inA;
    PinName inB;
};

struct MotorPwm {
    PwmPin inA{};
    PwmPin inB{};
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
    MotorPwm motor;
    float beginSpeed = 0;
    float targetSpeed = 0;
    float totalSpeed = 0;
    uint32_t beginTimeMs = 0;
    uint8_t encoderIndex = 0;
    MotorPI pi;
};

struct VectorXY
{
    float x;
    float y;
};

struct SpeedRange {
    float min;
    float max;
    float scale = 1;
};

class MotorDriver {
public:
    static constexpr uint8_t MOTOR_COUNT = 3;
    static constexpr float SIN_60 = 0.8660254037844f;

    // Omni-wheel inverse kinematics: wheel speeds (in % before clamping) for a
    // heading given as sin/cos, a translation scale and a rotation term.
    static void wheelSpeeds(float sinHeading, float cosHeading, float scale, float rotation,
                            float out[MOTOR_COUNT]);

private:
    MotorPin pins[MOTOR_COUNT];
    Motor motors[MOTOR_COUNT];

    SpeedRange speedRange = {MIN_SPEED, MAX_SPEED, (MAX_SPEED - MIN_SPEED) / 100.0f};

    bool encodersEnabled = false;
    float kP = 0.5f;
    float kI = 0.05f;
    float piIntegralMax = 30.0f;
    float maxTicksPerSec[MOTOR_COUNT] = {1200.0f, 1200.0f, 1200.0f};

    template<bool stage>
    void setMotorSpeed(const MotorPwm& motor, float targetSpeed) const;
    template<bool stage>
    void updateMotor(Motor& motor) const;
    static void drive(Motor& motor, float speed, float totalSpeed);

public:
    MotorDriver(const MotorPin m1, const MotorPin m2, const MotorPin m3) : pins{m1, m2, m3} {}

    void init(float minSpeed = MIN_SPEED, float maxSpeed = MAX_SPEED,
              const EncoderPins enc[MOTOR_COUNT] = nullptr);

    void updateAllMotors();
    void syncUpdateAllMotors();

    void driveDegrees(float degrees, float scale = 100, float rotation = 0);
    void driveRadians(float radians, float scale = 100, float rotation = 0);
    void driveVector(VectorXY vector, float rotation = 0);
    void driveMotorsDirect(float m1Speed, float m2Speed, float m3Speed);

    void changeSpeed(const float minSpeed = MIN_SPEED, const float maxSpeed = MAX_SPEED) {
        speedRange = {minSpeed, maxSpeed, (maxSpeed - minSpeed) / 100.0f};
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