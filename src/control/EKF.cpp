#include "EKF.h"

#include <Arduino.h>
#include <cmath>

namespace
{
    constexpr float PI_F = 3.14159265358979323846f;

    float clampConfidence(const float value)
    {
        return value < 0.1f ? 0.1f : (value > 1.0f ? 1.0f : value);
    }

    void zeroMatrix4(float m[4][4])
    {
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                m[i][j] = 0.0f;
            }
        }
    }

    void identityMatrix4(float m[4][4])
    {
        zeroMatrix4(m);
        for (int i = 0; i < 4; ++i)
        {
            m[i][i] = 1.0f;
        }
    }

    void zeroMatrix2(float m[2][2])
    {
        for (int i = 0; i < 2; ++i)
        {
            for (int j = 0; j < 2; ++j)
            {
                m[i][j] = 0.0f;
            }
        }
    }

    void identityMatrix2(float m[2][2])
    {
        zeroMatrix2(m);
        m[0][0] = 1.0f;
        m[1][1] = 1.0f;
    }
} // namespace

EKF ekf;

EKF::EKF()
{
    reset();
}

void EKF::reset(const float x, const float y, const float vx, const float vy)
{
    state_[0] = x;
    state_[1] = y;
    state_[2] = vx;
    state_[3] = vy;

    zeroMatrix4(covariance_);
    covariance_[0][0] = 10000.0f;
    covariance_[1][1] = 10000.0f;
    covariance_[2][2] = 2500.0f;
    covariance_[3][3] = 2500.0f;

    initialized_ = true;
    lastRangeCm_ = 0.0f;
    lastBearingDeg_ = 0.0f;
}

void EKF::setProcessNoise(const float accelStdDevCmS2)
{
    if (accelStdDevCmS2 > 0.0f)
    {
        processAccelStdDev_ = accelStdDevCmS2;
    }
}

void EKF::setMeasurementNoise(const float rangeStdDevCm, const float bearingStdDevDeg)
{
    if (rangeStdDevCm > 0.0f)
        rangeStdDevCm_ = rangeStdDevCm;
    if (bearingStdDevDeg > 0.0f)
        bearingStdDevDeg_ = bearingStdDevDeg;
}

float EKF::wrapRadians(float radians)
{
    while (radians > PI_F) radians -= 2.0f * PI_F;
    while (radians < -PI_F) radians += 2.0f * PI_F;
    return radians;
}

float EKF::radiansToDegrees(const float radians)
{
    return radians * (180.0f / PI_F);
}

float EKF::degreesToRadians(const float degrees)
{
    return degrees * (PI_F / 180.0f);
}

void EKF::predict(const float dt)
{
    if (!(dt > 0.0f))
        return;

    state_[0] += state_[2] * dt;
    state_[1] += state_[3] * dt;

    float F[4][4];
    identityMatrix4(F);
    F[0][2] = dt;
    F[1][3] = dt;

    float FP[4][4] = {};
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            float acc = 0.0f;
            for (int k = 0; k < 4; ++k)
            {
                acc += F[i][k] * covariance_[k][j];
            }
            FP[i][j] = acc;
        }
    }

    float newP[4][4] = {};
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            float acc = 0.0f;
            for (int k = 0; k < 4; ++k)
            {
                acc += FP[i][k] * F[j][k];
            }
            newP[i][j] = acc;
        }
    }

    const float q = processAccelStdDev_ * processAccelStdDev_;
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;

    newP[0][0] += 0.25f * dt4 * q;
    newP[0][2] += 0.5f * dt3 * q;
    newP[1][1] += 0.25f * dt4 * q;
    newP[1][3] += 0.5f * dt3 * q;
    newP[2][0] += 0.5f * dt3 * q;
    newP[2][2] += dt2 * q;
    newP[3][1] += 0.5f * dt3 * q;
    newP[3][3] += dt2 * q;

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            covariance_[i][j] = newP[i][j];
        }
    }

    symmetrizeCovariance();
}

void EKF::updateObservation(const IRBallObservation& observation)
{
    if (!observation.valid)
    {
        return;
    }

    updatePolar(observation.rangeCm, observation.bearingDeg, observation.confidence);
}

void EKF::updatePolar(const float rangeCm, const float bearingDeg, const float confidence)
{
    const float c = clampConfidence(confidence);

    const float r2 = state_[0] * state_[0] + state_[1] * state_[1];
    if (rangeCm > 0.0f && r2 <= 1.0e-6f)
    {
        const float bearingRad = degreesToRadians(bearingDeg);
        state_[0] = rangeCm * cosf(bearingRad);
        state_[1] = rangeCm * sinf(bearingRad);
        state_[2] = 0.0f;
        state_[3] = 0.0f;

        covariance_[0][0] = rangeStdDevCm_ * rangeStdDevCm_;
        covariance_[1][1] = rangeStdDevCm_ * rangeStdDevCm_;
        covariance_[2][2] = 2500.0f;
        covariance_[3][3] = 2500.0f;
    }

    if (rangeCm > 0.0f)
    {
        updateRangeMeasurement(rangeCm, rangeStdDevCm_ / c, c);
        lastRangeCm_ = rangeCm;
    }

    updateBearingMeasurement(bearingDeg, bearingStdDevDeg_ / c, c);
    lastBearingDeg_ = bearingDeg;
    initialized_ = true;
}

void EKF::updateRangeMeasurement(const float rangeCm, const float noiseCm, const float confidence)
{
    (void)confidence;
    const float x = state_[0];
    const float y = state_[1];
    const float r = sqrtf(x * x + y * y);
    if (!(r > 1.0e-6f))
        return;

    const float H[2][4] = {
        {x / r, y / r, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f}
    };
    const float residual[2] = {rangeCm - r, 0.0f};
    float R[2][2];

    identityMatrix2(R);
    R[0][0] = noiseCm * noiseCm;
    R[1][1] = 1.0e12f;
    applyLinearUpdate(H, residual, R);
}

void EKF::updateBearingMeasurement(const float bearingDeg, const float noiseDeg, const float confidence)
{
    (void)confidence;

    const float x = state_[0];
    const float y = state_[1];
    const float r2 = x * x + y * y;

    if (!(r2 > 1.0e-6f))
        return;

    const float predictedBearing = atan2f(y, x);
    const float measuredBearing = degreesToRadians(bearingDeg);
    const float residualAngle = wrapRadians(measuredBearing - predictedBearing);

    const float H[2][4] = {
        {-y / r2, x / r2, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f}
    };
    const float residual[2] = {residualAngle, 0.0f};

    float R[2][2];
    identityMatrix2(R);
    const float bearingNoiseRad = degreesToRadians(noiseDeg);
    R[0][0] = bearingNoiseRad * bearingNoiseRad;
    R[1][1] = 1.0e12f;

    applyLinearUpdate(H, residual, R);
}

void EKF::applyLinearUpdate(const float H[2][4], const float residual[2], const float R[2][2])
{
    float PHt[4][2] = {};
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            float acc = 0.0f;
            for (int k = 0; k < 4; ++k)
            {
                acc += covariance_[i][k] * H[j][k];
            }
            PHt[i][j] = acc;
        }
    }

    float S[2][2] = {};
    for (int i = 0; i < 2; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            float acc = 0.0f;
            for (int k = 0; k < 4; ++k)
            {
                acc += H[i][k] * PHt[k][j];
            }
            S[i][j] = acc + R[i][j];
        }
    }

    const float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    if (fabsf(det) < 1.0e-9f)
        return;

    const float invDet = 1.0f / det;
    const float SInv[2][2] = {
        {S[1][1] * invDet, -S[0][1] * invDet},
        {-S[1][0] * invDet, S[0][0] * invDet}
    };

    float K[4][2] = {};
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            float acc = 0.0f;
            for (int k = 0; k < 2; ++k)
            {
                acc += PHt[i][k] * SInv[k][j];
            }
            K[i][j] = acc;
        }
    }

    for (int i = 0; i < 4; ++i)
        state_[i] += K[i][0] * residual[0] + K[i][1] * residual[1];

    float KH[4][4] = {};
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            float acc = 0.0f;
            for (int k = 0; k < 2; ++k)
            {
                acc += K[i][k] * H[k][j];
            }
            KH[i][j] = acc;
        }
    }

    float IminusKH[4][4];
    identityMatrix4(IminusKH);
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            IminusKH[i][j] -= KH[i][j];
        }
    }

    float newP[4][4] = {};
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            float acc = 0.0f;
            for (int k = 0; k < 4; ++k)
            {
                acc += IminusKH[i][k] * covariance_[k][j];
            }
            newP[i][j] = acc;
        }
    }

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            covariance_[i][j] = newP[i][j];
        }
    }

    symmetrizeCovariance();
}

EKF::State EKF::getState() const
{
    return {state_[0], state_[1], state_[2], state_[3]};
}

float EKF::getX() const
{
    return state_[0];
}

float EKF::getY() const
{
    return state_[1];
}

float EKF::getVx() const
{
    return state_[2];
}

float EKF::getVy() const
{
    return state_[3];
}

float EKF::getRange() const
{
    return sqrtf(state_[0] * state_[0] + state_[1] * state_[1]);
}

float EKF::getBearing() const
{
    return radiansToDegrees(atan2f(state_[1], state_[0]));
}

bool EKF::isInitialized() const
{
    return initialized_;
}

void EKF::symmetrizeCovariance()
{
    for (int i = 0; i < 4; ++i)
    {
        for (int j = i + 1; j < 4; ++j)
        {
            const float avg = 0.5f * (covariance_[i][j] + covariance_[j][i]);
            covariance_[i][j] = avg;
            covariance_[j][i] = avg;
        }
    }
}
