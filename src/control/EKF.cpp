#include "EKF.h"

#include <Arduino.h>
#include <cmath>

#include "helpers/MatrixMultiplyArm.h"

namespace
{
    FORCE_INLINE float clampConfidence(const float value)
    {
        return value < 0.1f ? 0.1f : (value > 1.0f ? 1.0f : value);
    }
} // namespace

EKF ekf;

void EKF::reset(const float x, const float y, const float vx, const float vy)
{
    // Reset the filter state and uncertainty.
    state_ = Math::vec4(x, y, vx, vy);
    covariance_ = Math::diag4(10000.0f, 10000.0f, 2500.0f, 2500.0f);

    initialized_ = true;
    lastRangeCm_ = 0.0f;
    lastBearingDeg_ = 0.0f;
}

void EKF::setProcessNoise(const float accelStdDevCmS2)
{
    if (accelStdDevCmS2 > 0.0f)
        processAccelStdDev_ = accelStdDevCmS2;
}

void EKF::setMeasurementNoise(const float rangeStdDevCm, const float bearingStdDevDeg)
{
    if (rangeStdDevCm > 0.0f)
        rangeStdDevCm_ = rangeStdDevCm;
    if (bearingStdDevDeg > 0.0f)
        bearingStdDevDeg_ = bearingStdDevDeg;
}

void EKF::predict(const float dt)
{
    if (!(dt > 0.0f))
        return;

    state_[0] += state_[2] * dt;
    state_[1] += state_[3] * dt;

    Math::Mat44 F = Math::identity4();
    F[0][2] = dt;
    F[1][3] = dt;

    Math::Mat44 FP = {};
    ArmMatrix::multiply4x4(F.data(), covariance_.data(), FP.data());

    Math::Mat44 newP = {};
    ArmMatrix::multiply4x4ByTransposed(FP.data(), F.data(), newP.data());

    const float q = processAccelStdDev_ * processAccelStdDev_;
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;

    const Math::Mat44 Q = Math::mat4(
        0.25f * dt4 * q, 0.0f,           0.5f * dt3 * q, 0.0f,
        0.0f,            0.25f * dt4 * q, 0.0f,           0.5f * dt3 * q,
        0.5f * dt3 * q,  0.0f,            dt2 * q,        0.0f,
        0.0f,            0.5f * dt3 * q,  0.0f,           dt2 * q
    );
    Math::addMatrix4(newP, Q);

    covariance_ = newP;

    symmetrizeCovariance();
}

void EKF::updateObservation(const IRBallObservation& observation)
{
    if (!observation.valid)
        return;

    updatePolar(observation.rangeCm, observation.bearingDeg, observation.confidence);
}

void EKF::updatePolar(const float rangeCm, const float bearingDeg, const float confidence)
{
    const float c = clampConfidence(confidence);

    const float r2 = state_[0] * state_[0] + state_[1] * state_[1];
    if (rangeCm > 0.0f && r2 <= 1.0e-6f)
    {
        // Bootstrap the state from the first valid polar measurement.
        const float bearingRad = Math::degreesToRadians(bearingDeg);
        state_ = Math::vec4(rangeCm * cosf(bearingRad), rangeCm * sinf(bearingRad), 0.0f, 0.0f);

        const float rangeVariance = rangeStdDevCm_ * rangeStdDevCm_;
        covariance_ = Math::diag4(rangeVariance, rangeVariance, 2500.0f, 2500.0f);
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

    // Use the range residual to correct position.
    const Math::Mat24 H = Math::mat2x4(
        x / r, y / r, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f
    );
    const Math::Vec2 residual = Math::vec2(rangeCm - r, 0.0f);
    const Math::Mat22 R = Math::diag2(noiseCm * noiseCm, 1.0e12f);
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
    const float measuredBearing = Math::degreesToRadians(bearingDeg);
    const float residualAngle = Math::wrapRadians(measuredBearing - predictedBearing);

    // Use the bearing residual to correct position.
    const Math::Mat24 H = Math::mat2x4(
        -y / r2, x / r2, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f
    );
    const Math::Vec2 residual = Math::vec2(residualAngle, 0.0f);

    const float bearingNoiseRad = Math::degreesToRadians(noiseDeg);
    const Math::Mat22 R = Math::diag2(bearingNoiseRad * bearingNoiseRad, 1.0e12f);

    applyLinearUpdate(H, residual, R);
}

void EKF::applyLinearUpdate(const Math::Mat24& H, const Math::Vec2& residual, const Math::Mat22& R)
{
    Math::Mat42 PHt = {};
    ArmMatrix::multiply4x2ByTransposed(covariance_.data(), H.data(), PHt.data());

    Math::Mat22 S = {};
    ArmMatrix::multiply2x2(H.data(), PHt.data(), S.data());
    Math::addMatrix2(S, R);

    const float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    if (fabsf(det) < 1.0e-9f)
        return;

    const float invDet = 1.0f / det;
    const Math::Mat22 SInv = Math::mat2(
        S[1][1] * invDet, -S[0][1] * invDet,
        -S[1][0] * invDet, S[0][0] * invDet
    );

    // K maps the residual into a state correction.
    Math::Mat42 K = {};
    ArmMatrix::multiply4x2(PHt.data(), SInv.data(), K.data());

    // StateDelta is the correction applied to the estimate.
    Math::Vec4 stateDelta = {};
    ArmMatrix::multiply4x2Vector(K.data(), residual.data(), stateDelta.data());
    Math::addVector4(state_, stateDelta);

    // KH is the Kalman gain applied to the measurement model.
    Math::Mat44 KH = {};
    ArmMatrix::multiply4x4From4x2And2x4(K.data(), H.data(), KH.data());

    // IminusKH updates the covariance after the correction.
    Math::Mat44 IminusKH = {};
    Math::subtractFromIdentity4(IminusKH, KH);

    // newP stores the updated covariance.
    Math::Mat44 newP = {};
    ArmMatrix::multiply4x4(IminusKH.data(), covariance_.data(), newP.data());

    covariance_ = newP;

    symmetrizeCovariance();
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
