#include "SelfLocalizationEKF.h"

#include <cmath>

#include "../ekf/EKFCore.h"

namespace
{
    constexpr float kCompassNoiseIdle = 0.0049f;
    constexpr float kCompassNoiseBusy = 0.0305f;
    constexpr float kSensorOffsets[4] = {0.0f, 0.5f * PI_F, PI_F, -0.5f * PI_F};

    constexpr float kFieldXMin = -1.2f;
    constexpr float kFieldXMax = 1.2f;
    constexpr float kFieldYMin = -0.9f;
    constexpr float kFieldYMax = 0.9f;

    FORCE_INLINE float wrapAngle(const float angle)
    {
        return Math::wrapRadians(angle);
    }

    void selfloc_clamp_to_field(SelfLocalizationFilter* filter)
    {
        filter->x[0] = clampf(filter->x[0], -1.15f, 1.15f);
        filter->x[1] = clampf(filter->x[1], -0.85f, 0.85f);
        filter->x[2] = wrapAngle(filter->x[2]);
    }
}

void selfloc_reset(SelfLocalizationFilter* filter, const float x, const float y, const float theta)
{
    if (filter == nullptr)
        return;

    filter->x[0] = x;
    filter->x[1] = y;
    filter->x[2] = wrapAngle(theta);
    filter->x[3] = 0.0f;
    filter->x[4] = 0.0f;
    filter->x[5] = 0.0f;

    for (auto & r : filter->P)
        for (float & c : r)
            c = 0.0f;

    filter->P[0][0] = 0.06f;
    filter->P[1][1] = 0.06f;
    filter->P[2][2] = 0.08f;
    filter->P[3][3] = 0.25f;
    filter->P[4][4] = 0.25f;
    filter->P[5][5] = 0.25f;
}

void selfloc_predict(SelfLocalizationFilter* filter, const float dt, const float vxBody, const float vyBody, const float omega)
{
    if (filter == nullptr || dt <= 0.0f)
        return;

    filter->x[3] = vxBody;
    filter->x[4] = vyBody;
    filter->x[5] = omega;

    const float theta = filter->x[2];
    const float c = cosf(theta);
    const float s = sinf(theta);

    filter->x[0] += (vxBody * c - vyBody * s) * dt;
    filter->x[1] += (vxBody * s + vyBody * c) * dt;
    filter->x[2] = wrapAngle(filter->x[2] + omega * dt);

    float F[36] = {};
    for (int i = 0; i < 6; ++i)
    {
        F[i * 6 + i] = 1.0f;
    }

    F[0 * 6 + 2] = (-vxBody * s - vyBody * c) * dt;
    F[0 * 6 + 3] = c * dt;
    F[0 * 6 + 4] = -s * dt;
    F[1 * 6 + 2] = (vxBody * c - vyBody * s) * dt;
    F[1 * 6 + 3] = s * dt;
    F[1 * 6 + 4] = c * dt;
    F[2 * 6 + 5] = dt;

    const float speed = sqrtf(vxBody * vxBody + vyBody * vyBody);
    const float sigmaX = 0.03f * speed * dt + 0.0008f;
    const float sigmaY = sigmaX;
    const float sigmaTheta = 0.05f * fabsf(omega) * dt + 0.0008f;

    const float qDiag[6] = {
        sigmaX * sigmaX,
        sigmaY * sigmaY,
        sigmaTheta * sigmaTheta,
        0.01f,
        0.01f,
        0.04f,
    };

    EKFCore::predictCovariance(&filter->P[0][0], F, qDiag, 6);
    selfloc_clamp_to_field(filter);
}

void selfloc_update_compass(SelfLocalizationFilter* filter, const float thetaMeasuredRad, const float totalPwmDuty)
{
    if (filter == nullptr)
        return;

    const float H[6] = {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f};
    const float innovation = wrapAngle(thetaMeasuredRad - filter->x[2]);
    const float R = (totalPwmDuty > 1.5f) ? kCompassNoiseBusy : kCompassNoiseIdle;
    EKFCore::updateScalar(filter->x, &filter->P[0][0], H, innovation, R, 6);
    filter->x[2] = wrapAngle(filter->x[2]);
    selfloc_clamp_to_field(filter);
}

float selfloc_expected_wall_distance(const SelfLocalizationFilter* filter, const float beamAngleRad)
{
    if (filter == nullptr)
        return 0.0f;


    const float c = cosf(beamAngleRad);
    const float s = sinf(beamAngleRad);
    float minD = 10.0f;

    if (fabsf(c) > 1.0e-5f)
    {
        const float dxPos = (kFieldXMax - filter->x[0]) / c;
        const float dxNeg = (kFieldXMin - filter->x[0]) / c;
        if (dxPos > 0.0f && dxPos < minD) minD = dxPos;
        if (dxNeg > 0.0f && dxNeg < minD) minD = dxNeg;
    }

    if (fabsf(s) > 1.0e-5f)
    {
        const float dyPos = (kFieldYMax - filter->x[1]) / s;
        const float dyNeg = (kFieldYMin - filter->x[1]) / s;
        if (dyPos > 0.0f && dyPos < minD) minD = dyPos;
        if (dyNeg > 0.0f && dyNeg < minD) minD = dyNeg;
    }

    if (minD > 9.0f)
        minD = 0.0f;

    return minD;
}

SonarUpdateResult selfloc_update_sonar(SelfLocalizationFilter* filter, const uint8_t sensorIdx, const float distanceM)
{
    SonarUpdateResult result = {};

    if (filter == nullptr)
        return result;

    if (sensorIdx >= 4 || distanceM < 0.03f || distanceM > 2.5f)
        return result;

    const float alpha = wrapAngle(filter->x[2] + kSensorOffsets[sensorIdx]);
    const float expected = selfloc_expected_wall_distance(filter, alpha);
    if (expected <= 0.0f)
        return result;

    const float residual = distanceM - expected;

    if (fabsf(residual) > 0.12f)
    {
        result.anomaly_detected = true;
        result.obstacle_x = filter->x[0] + distanceM * cosf(alpha);
        result.obstacle_y = filter->x[1] + distanceM * sinf(alpha);
        return result;
    }

    constexpr float eps = 0.01f;
    // Estimate Jacobian numerically for x, y, and theta.
    const float xOrig = filter->x[0];
    const float yOrig = filter->x[1];
    const float tOrig = filter->x[2];

    filter->x[0] = xOrig + eps;
    const float dPx = selfloc_expected_wall_distance(filter, alpha);
    filter->x[0] = xOrig - eps;
    const float dNx = selfloc_expected_wall_distance(filter, alpha);
    filter->x[0] = xOrig;

    filter->x[1] = yOrig + eps;
    const float dPy = selfloc_expected_wall_distance(filter, alpha);
    filter->x[1] = yOrig - eps;
    const float dNy = selfloc_expected_wall_distance(filter, alpha);
    filter->x[1] = yOrig;

    filter->x[2] = wrapAngle(tOrig + eps);
    const float dPt = selfloc_expected_wall_distance(filter, wrapAngle(filter->x[2] + kSensorOffsets[sensorIdx]));
    filter->x[2] = wrapAngle(tOrig - eps);
    const float dNt = selfloc_expected_wall_distance(filter, wrapAngle(filter->x[2] + kSensorOffsets[sensorIdx]));
    filter->x[2] = tOrig;

    const float H[6] = {
        (dPx - dNx) / (2.0f * eps),
        (dPy - dNy) / (2.0f * eps),
        (dPt - dNt) / (2.0f * eps),
        0.0f,
        0.0f,
        0.0f,
    };

    EKFCore::updateScalar(filter->x, &filter->P[0][0], H, residual, 0.0009f, 6);
    selfloc_clamp_to_field(filter);
    result.used_for_localization = true;
    return result;
}

SelfLocState selfloc_get_state(const SelfLocalizationFilter* filter)
{
    SelfLocState out = {};
    if (filter == nullptr)
        return out;

    out.x = filter->x[0];
    out.y = filter->x[1];
    out.theta = filter->x[2];
    out.vx = filter->x[3];
    out.vy = filter->x[4];
    out.omega = filter->x[5];
    out.P_xy = filter->P[0][0] + filter->P[1][1];
    return out;
}
