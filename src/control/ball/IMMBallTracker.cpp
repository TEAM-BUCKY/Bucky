#include "IMMBallTracker.h"

#include <cmath>

#include "../ekf/EKFCore.h"

namespace
{
    constexpr float kTransitionBase[3][3] = {
        {0.990f, 0.005f, 0.005f},
        {0.010f, 0.985f, 0.005f},
        {0.010f, 0.005f, 0.985f},
    };

    constexpr float kCaptureOffsetM = 0.11f;
    constexpr float kFrictionMu = 1.5f;
    constexpr float kRestitution = 0.60f;

    constexpr float kFieldXMin = -1.2f;
    constexpr float kFieldXMax = 1.2f;
    constexpr float kFieldYMin = -0.9f;
    constexpr float kFieldYMax = 0.9f;

    void mult4x4(const float a[4][4], const float b[4][4], float out[4][4])
    {
        for (int r = 0; r < 4; ++r)
        {
            for (int c = 0; c < 4; ++c)
            {
                float acc = 0.0f;
                for (int k = 0; k < 4; ++k)
                {
                    acc += a[r][k] * b[k][c];
                }
                out[r][c] = acc;
            }
        }
    }

    void mult4x4T(const float a[4][4], const float b[4][4], float out[4][4])
    {
        for (int r = 0; r < 4; ++r)
        {
            for (int c = 0; c < 4; ++c)
            {
                float acc = 0.0f;
                for (int k = 0; k < 4; ++k)
                {
                    acc += a[r][k] * b[c][k];
                }
                out[r][c] = acc;
            }
        }
    }
}

void IMMBallTracker::copy4x4(float dst[4][4], const float src[4][4])
{
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            dst[r][c] = src[r][c];
}

void IMMBallTracker::identity4x4(float out[4][4])
{
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            out[r][c] = (r == c) ? 1.0f : 0.0f;
}

void IMMBallTracker::addDiag4(float m[4][4], const float d0, const float d1, const float d2, const float d3)
{
    m[0][0] += d0;
    m[1][1] += d1;
    m[2][2] += d2;
    m[3][3] += d3;
}

void IMMBallTracker::reset(const float bx, const float by)
{
    for (auto & [x, P] : mode_)
    {
        x[0] = bx;
        x[1] = by;
        x[2] = 0.0f;
        x[3] = 0.0f;
        identity4x4(P);
        addDiag4(P, 0.04f, 0.04f, 0.5f, 0.5f);
    }

    mu_[0] = 1.0f;
    mu_[1] = 0.0f;
    mu_[2] = 0.0f;
    possessionHint_ = BALL_MODE_FREE;
    initialized_ = true;
    out_ = {};
    out_.mu[0] = 1.0f;
}

void IMMBallTracker::setPossessionHint(const BallMode mode)
{
    possessionHint_ = mode;
}

void IMMBallTracker::predictFree(ModeFilter& f, const float dt) const
{
    const float damp = fmaxf(0.0f, 1.0f - kFrictionMu * dt);

    f.x[0] += f.x[2] * dt;
    f.x[1] += f.x[3] * dt;
    f.x[2] *= damp;
    f.x[3] *= damp;

    if (f.x[0] > kFieldXMax)
    {
        f.x[0] = 2.0f * kFieldXMax - f.x[0];
        f.x[2] *= -kRestitution;
    }
    if (f.x[0] < kFieldXMin)
    {
        f.x[0] = 2.0f * kFieldXMin - f.x[0];
        f.x[2] *= -kRestitution;
    }
    if (f.x[1] > kFieldYMax)
    {
        f.x[1] = 2.0f * kFieldYMax - f.x[1];
        f.x[3] *= -kRestitution;
    }
    if (f.x[1] < kFieldYMin)
    {
        f.x[1] = 2.0f * kFieldYMin - f.x[1];
        f.x[3] *= -kRestitution;
    }

    const float F[4][4] = {
        {1.0f, 0.0f, dt, 0.0f},
        {0.0f, 1.0f, 0.0f, dt},
        {0.0f, 0.0f, damp, 0.0f},
        {0.0f, 0.0f, 0.0f, damp},
    };

    float FP[4][4] = {};
    float newP[4][4] = {};
    mult4x4(F, f.P, FP);
    mult4x4T(FP, F, newP);

    const float qScale = fmaxf(dt, 0.001f);
    addDiag4(newP, 0.001f * qScale, 0.001f * qScale, 2.0f * qScale, 2.0f * qScale);
    copy4x4(f.P, newP);
}

void IMMBallTracker::predictFriendly(ModeFilter& f, const float dt, const SelfLocState& self) const
{
    (void)dt;
    const float bx = self.x + kCaptureOffsetM * cosf(self.theta);
    const float by = self.y + kCaptureOffsetM * sinf(self.theta);

    float vxField = 0.0f;
    float vyField = 0.0f;
    bodyToField(self.vx, self.vy, self.theta, &vxField, &vyField);

    f.x[0] = bx;
    f.x[1] = by;
    f.x[2] = vxField;
    f.x[3] = vyField;

    addDiag4(f.P, 0.0001f * dt, 0.0001f * dt, 0.01f * dt, 0.01f * dt);
}

void IMMBallTracker::predictEnemy(ModeFilter& f, const float dt, const EnemyState& enemy) const
{
    const float heading = (fabsf(enemy.vx) + fabsf(enemy.vy) > 0.05f) ? atan2f(enemy.vy, enemy.vx) : 0.0f;
    f.x[0] = enemy.x + kCaptureOffsetM * cosf(heading);
    f.x[1] = enemy.y + kCaptureOffsetM * sinf(heading);
    f.x[2] = enemy.vx;
    f.x[3] = enemy.vy;

    addDiag4(f.P, 0.005f * dt, 0.005f * dt, 0.5f * dt, 0.5f * dt);
}

void IMMBallTracker::step(const float dt,
                          const bool hasMeasurement,
                          const float measX,
                          const float measY,
                          const float measRange,
                          const SelfLocState& self,
                          const EnemyState& enemy,
                          const uint32_t nowMs)
{
    if (!initialized_)
        reset(measX, measY);

    float transition[3][3] = {};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            transition[i][j] = kTransitionBase[i][j];

    if (possessionHint_ == BALL_MODE_FRIENDLY)
    {
        transition[BALL_MODE_FREE][BALL_MODE_ENEMY] = 0.05f;
        transition[BALL_MODE_FREE][BALL_MODE_FREE] = 0.945f;
    }

    float cBar[3] = {};
    for (int j = 0; j < 3; ++j)
    {
        for (int i = 0; i < 3; ++i)
            cBar[j] += transition[i][j] * mu_[i];

        cBar[j] = fmaxf(cBar[j], 1.0e-6f);
    }

    ModeFilter mixed[3] = {};
    for (int j = 0; j < 3; ++j)
    {
        for (int i = 0; i < 3; ++i)
        {
            const float w = transition[i][j] * mu_[i] / cBar[j];
            for (int k = 0; k < 4; ++k)
                mixed[j].x[k] += w * mode_[i].x[k];
        }

        for (int i = 0; i < 3; ++i)
        {
            const float w = transition[i][j] * mu_[i] / cBar[j];
            float dx[4] = {};

            for (int k = 0; k < 4; ++k)
                dx[k] = mode_[i].x[k] - mixed[j].x[k];

            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c)
                    mixed[j].P[r][c] += w * (mode_[i].P[r][c] + dx[r] * dx[c]);
        }
    }

    for (int j = 0; j < 3; ++j)
        mode_[j] = mixed[j];

    predictFree(mode_[BALL_MODE_FREE], dt);
    predictFriendly(mode_[BALL_MODE_FRIENDLY], dt, self);
    predictEnemy(mode_[BALL_MODE_ENEMY], dt, enemy);

    float likelihood[3] = {1.0f, 1.0f, 1.0f};
    float mahal[3] = {0.0f, 0.0f, 0.0f};
    float innovationMag = 0.0f;

    const float sigma = 0.03f + 0.08f * fmaxf(0.0f, measRange);
    const float R = sigma * sigma;

    if (hasMeasurement)
    {
        lastSeenMs_ = nowMs;

        bool anyAccepted = false;
        for (int j = 0; j < 3; ++j)
        {
            const bool accepted = EKFCore::updatePosition2Of4(mode_[j].x, mode_[j].P, measX, measY, R, 9.21f,
                                                              &mahal[j], &likelihood[j]);
            anyAccepted = anyAccepted || accepted;
        }

        if (!anyAccepted)
        {
            for (int j = 0; j < 3; ++j)
            {
                mode_[j].x[0] = measX;
                mode_[j].x[1] = measY;
                mode_[j].x[2] = 0.0f;
                mode_[j].x[3] = 0.0f;
                identity4x4(mode_[j].P);
                addDiag4(mode_[j].P, 0.12f, 0.12f, 1.2f, 1.2f);
                likelihood[j] = 0.2f;
                mahal[j] = 9.21f;
            }
        }

        float denom = 0.0f;
        for (int j = 0; j < 3; ++j)
            denom += cBar[j] * likelihood[j];
        denom = fmaxf(denom, 1.0e-8f);

        for (int j = 0; j < 3; ++j)
            mu_[j] = (cBar[j] * likelihood[j]) / denom;

        int best = 0;
        if (mu_[1] > mu_[best]) best = 1;
        if (mu_[2] > mu_[best]) best = 2;
        innovationMag = sqrtf(fmaxf(0.0f, mahal[best]));
        out_.visible = 1;
        out_.lost_ms = 0;
    }
    else
    {
        out_.visible = 0;
        out_.lost_ms = static_cast<uint16_t>(nowMs - lastSeenMs_);

        const float inflate = (out_.lost_ms > 300U) ? 10.0f : 3.0f;
        for (auto & [x, P] : mode_)
            addDiag4(P, inflate * 0.0005f, inflate * 0.0005f, inflate * 0.02f, inflate * 0.02f);
    }

    combineOutput(innovationMag);
}

void IMMBallTracker::combineOutput(const float innovationMag)
{
    float x[4] = {};
    for (int j = 0; j < 3; ++j)
        for (int k = 0; k < 4; ++k)
            x[k] += mu_[j] * mode_[j].x[k];

    float P[4][4] = {};
    for (int j = 0; j < 3; ++j)
    {
        float dx[4] = {};
        for (int k = 0; k < 4; ++k)
            dx[k] = mode_[j].x[k] - x[k];

        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                P[r][c] += mu_[j] * (mode_[j].P[r][c] + dx[r] * dx[c]);
    }

    out_.bx = x[0];
    out_.by = x[1];
    out_.bvx = x[2];
    out_.bvy = x[3];
    out_.mu[0] = mu_[0];
    out_.mu[1] = mu_[1];
    out_.mu[2] = mu_[2];
    out_.innovation_mag = innovationMag;
    out_.P[0][0] = P[0][0];
    out_.P[0][1] = P[0][1];
    out_.P[1][0] = P[1][0];
    out_.P[1][1] = P[1][1];
}

