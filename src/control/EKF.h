#ifndef BUCKY_EKF_H
#define BUCKY_EKF_H

#include <cstdint>

#include "IRBallTracker.h"

class EKF
{
    public:
        struct State {
            float x;
            float y;
            float vx;
            float vy;
        };

        EKF();

        void reset(float x = 0.0f, float y = 0.0f, float vx = 0.0f, float vy = 0.0f);
        void setProcessNoise(float accelStdDevCmS2);
        void setMeasurementNoise(float rangeStdDevCm, float bearingStdDevDeg);

        void predict(float dt);
        void updatePolar(float rangeCm, float bearingDeg, float confidence = 1.0f);
        void updateObservation(const IRBallObservation& observation);

        [[nodiscard]] State getState() const;
        [[nodiscard]] float getX() const;
        [[nodiscard]] float getY() const;
        [[nodiscard]] float getVx() const;
        [[nodiscard]] float getVy() const;
        [[nodiscard]] float getRange() const;
        [[nodiscard]] float getBearing() const;
        [[nodiscard]] bool isInitialized() const;

    private:
        float state_[4];
        float covariance_[4][4];

        float processAccelStdDev_ = 80.0f;
        float rangeStdDevCm_ = 20.0f;
        float bearingStdDevDeg_ = 8.0f;
        bool initialized_ = false;

        float lastRangeCm_ = 0.0f;
        float lastBearingDeg_ = 0.0f;

        static float wrapRadians(float radians);
        static float radiansToDegrees(float radians);
        static float degreesToRadians(float degrees);

        void applyLinearUpdate(const float H[2][4], const float residual[2], const float R[2][2]);
        void updateRangeMeasurement(float rangeCm, float noiseCm, float confidence);
        void updateBearingMeasurement(float bearingDeg, float noiseDeg, float confidence);
        void symmetrizeCovariance();
};

extern EKF ekf;

#endif // BUCKY_EKF_H
