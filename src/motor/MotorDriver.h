#ifndef BUCKY_MOTORDRIVER_H
#define BUCKY_MOTORDRIVER_H

#include <Arduino.h>
#include "io/gpio/pwm.h"
#include "io/encoder/Encoder.h"

inline constexpr int MIN_SPEED = 1300;
inline constexpr int MAX_SPEED = 2500;

struct MotorPin {
    int inA;
    int inB;
};

struct MotorPwm {
    PwmPin inA{};
    PwmPin inB{};
    float currentSpeed = 0;
};

struct MotorPI {
    float integral = 0.0f;
    uint32_t lastUpdateUs = 0;   // micros() at last PI update, 0 = uninit
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
    MotorPin m1;
    MotorPin m2;
    MotorPin m3;

    MotorPwm pw1;
    MotorPwm pw2;
    MotorPwm pw3;

    Motor motor1;
    Motor motor2;
    Motor motor3;

    SpeedRange speedRange = {MIN_SPEED, MAX_SPEED, (MAX_SPEED - MIN_SPEED) / 100.0f};

    bool encodersEnabled = false;
    float kP = 0.5f;
    float kI = 0.05f;
    float piIntegralMax = 30.0f;
    float maxTicksPerSec[3] = {1200.0f, 1200.0f, 1200.0f};
    // Inverse of (maxTicksPerSec / 100). Cached so the per-tick PI loop can
    // multiply instead of dividing. Kept in sync with maxTicksPerSec by the
    // setMaxTicksPerSec() setters.
    float invTicksPerPercent[3] = {100.0f / 1200.0f, 100.0f / 1200.0f, 100.0f / 1200.0f};

    float dirScale[12]     = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    float dirOffsetDeg[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    bool  dirCalEnabled    = false;

    template<bool stage>
    void setMotorSpeed(const MotorPwm& motor, float targetSpeed) const;
    template<bool stage>
    void updateMotor(Motor& motor) const;
    static void drive(Motor& motor, float speed, float totalSpeed);

    void syncUpdateMotor(Motor& motor) const;

public:
    MotorDriver(const MotorPin m1, const MotorPin m2, const MotorPin m3) : m1(m1), m2(m2), m3(m3) {};

    void init(float minSpeed = MIN_SPEED, float maxSpeed = MAX_SPEED,
              const EncoderPins enc[3] = nullptr);

    void updateAllMotors();
    void syncUpdateAllMotors();

    void driveDegrees(float degrees, float scale = 100, float rotation = 0);
    void driveRadians(float radians, float scale = 100, float rotation = 0);
    void driveVector(VectorXY vector, float rotation = 0);
    void driveMotorsDirect(float m1Speed, float m2Speed, float m3Speed);

    void changeSpeed(const float minSpeed = MIN_SPEED, const float maxSpeed = MAX_SPEED) {
        this->speedRange.min = minSpeed;
        this->speedRange.max = maxSpeed;
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
        // Reject implausible values; leave the existing defaults in place so the
        // PI loop's ticksPerPercent = tps/100 can never be zero or negative.
        if (tps < 1.0f || tps > 100000.0f) return;
        const float inv = 100.0f / tps;
        maxTicksPerSec[0] = maxTicksPerSec[1] = maxTicksPerSec[2] = tps;
        invTicksPerPercent[0] = invTicksPerPercent[1] = invTicksPerPercent[2] = inv;
    }

    void setMaxTicksPerSec(const uint8_t motor, const float tps) {
        if (motor >= 3) return;
        if (tps < 1.0f || tps > 100000.0f) return;
        maxTicksPerSec[motor] = tps;
        invTicksPerPercent[motor] = 100.0f / tps;
    }

    void setDirectionCalibration(const float scale[12], const float offsetDeg[12]);
    void enableDirectionCalibration(const bool on) { dirCalEnabled = on; }
};

#endif //BUCKY_MOTORDRIVER_H
