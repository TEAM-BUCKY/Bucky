#ifndef BUCKY_ENEMYTRACKER_H
#define BUCKY_ENEMYTRACKER_H

#include <cstdint>

#include <control/pos/SelfLocalizationEKF.h>

struct EnemyState {
    float x = 0.0f;
    float y = 0.0f;
    float vx = 0.0f;
    float vy = 0.0f;
    float confidence = 0.0f;
    uint32_t last_update_ms = 0;
};

class EnemyTracker
{
public:
    void reset() { state_ = {}; }
    void update(float mx, float my, uint32_t nowMs);
    void decay(uint32_t nowMs);
    void reportFromSonar(const SelfLocState& self, uint8_t sensorIdx, float distanceM, uint32_t nowMs);

    [[nodiscard]] EnemyState getState() const { return state_; }

private:
    EnemyState state_ = {};
    static constexpr float kAlpha = 0.35f;
    static constexpr uint32_t kTimeoutMs = 500;
};

#endif // BUCKY_ENEMYTRACKER_H

