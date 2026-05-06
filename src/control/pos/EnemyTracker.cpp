#include <control/pos/EnemyTracker.h>
#include <helpers/Math.h>
#include <io/cordic/cordic.h>

#include <cmath>

constexpr float kSensorOffsets[4] = {0.0f, 0.5f * PI_F, PI_F, -0.5f * PI_F};

void EnemyTracker::update(const float mx, const float my, const uint32_t nowMs)
{
    if (state_.confidence > 0.1f)
    {
        const float dt = static_cast<float>(nowMs - state_.last_update_ms) * 0.001f;
        const float newX = state_.x + kAlpha * (mx - state_.x);
        const float newY = state_.y + kAlpha * (my - state_.y);

        if (dt > 0.01f && dt < 1.0f)
        {
            state_.vx = (newX - state_.x) / dt;
            state_.vy = (newY - state_.y) / dt;
        }

        state_.x = newX;
        state_.y = newY;
    }
    else
    {
        state_.x = mx;
        state_.y = my;
        state_.vx = 0.0f;
        state_.vy = 0.0f;
    }

    state_.confidence = 1.0f;
    state_.last_update_ms = nowMs;
}

void EnemyTracker::decay(const uint32_t nowMs)
{
    const float elapsed = static_cast<float>(nowMs - state_.last_update_ms) * 0.001f;
    constexpr float timeoutS = static_cast<float>(kTimeoutMs) * 0.001f;
    state_.confidence = fmaxf(0.0f, 1.0f - elapsed / timeoutS);
}

void EnemyTracker::reportFromSonar(const SelfLocState& self, const uint8_t sensorIdx, const float distanceM, const uint32_t nowMs)
{
    if (sensorIdx >= 4 || distanceM < 0.03f || distanceM > 2.5f)
    {
        return;
    }

    const float alpha = self.theta + kSensorOffsets[sensorIdx];
    float s, c;
    cordic_sin_cos(alpha, &s, &c);
    const float mx = self.x + distanceM * c;
    const float my = self.y + distanceM * s;

    update(mx, my, nowMs);
}

