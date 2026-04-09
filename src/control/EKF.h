#ifndef BUCKY_EKF_H
#define BUCKY_EKF_H

#include <cstdint>

#include "IRBallTracker.h"
#include "helpers/Math.h"

class EKF
{
    public:
        struct State {
            float x;
            float y;
            float vx;
            float vy;
        };

        EKF() { reset(); }

        void reset(float x = 0.0f, float y = 0.0f, float vx = 0.0f, float vy = 0.0f);
        void setProcessNoise(float accelStdDevCmS2);
        void setMeasurementNoise(float rangeStdDevCm, float bearingStdDevDeg);

        void predict(float dt);
        void updatePolar(float rangeCm, float bearingDeg, float confidence = 1.0f);
        void updateObservation(const IRBallObservation& observation);

        [[nodiscard]] FORCE_INLINE State getState() const { return {state_[0], state_[1], state_[2], state_[3]}; }
        [[nodiscard]] FORCE_INLINE float getX() const { return state_[0]; }
        [[nodiscard]] FORCE_INLINE float getY() const { return state_[1]; }
        [[nodiscard]] FORCE_INLINE float getVx() const { return state_[2]; }
        [[nodiscard]] FORCE_INLINE float getVy() const { return state_[3]; }
        [[nodiscard]] FORCE_INLINE float getRange() const { return sqrtf(state_[0] * state_[0] + state_[1] * state_[1]); }
        [[nodiscard]] FORCE_INLINE float getBearing() const { return Math::radiansToDegrees(atan2f(state_[1], state_[0])); }
        [[nodiscard]] FORCE_INLINE bool isInitialized() const { return initialized_; }

    private:
        Math::Vec4 state_;
        Math::Mat44 covariance_;

        float processAccelStdDev_ = 80.0f;
        float rangeStdDevCm_ = 20.0f;
        float bearingStdDevDeg_ = 8.0f;
        bool initialized_ = false;

        float lastRangeCm_ = 0.0f;
        float lastBearingDeg_ = 0.0f;

        void applyLinearUpdate(const Math::Mat24& H, const Math::Vec2& residual, const Math::Mat22& R);
        void updateRangeMeasurement(float rangeCm, float noiseCm, float confidence);
        void updateBearingMeasurement(float bearingDeg, float noiseDeg, float confidence);
        void symmetrizeCovariance();
};

extern EKF ekf;

#endif // BUCKY_EKF_H
