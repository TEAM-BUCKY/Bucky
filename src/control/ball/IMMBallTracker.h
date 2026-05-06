#ifndef BUCKY_IMMBALLTRACKER_H
#define BUCKY_IMMBALLTRACKER_H

#include <cstdint>

#include "../pos/EnemyTracker.h"
#include "field/DigitalField.h"

struct BallEstimate {
    float bx = 0.0f;
    float by = 0.0f;
    float bvx = 0.0f;
    float bvy = 0.0f;
    float P[4][4] = {};
    float mu[3] = {1.0f, 0.0f, 0.0f};
    float innovation_mag = 0.0f;
    uint8_t visible = 0;
    uint16_t lost_ms = 0;
};

class IMMBallTracker
{
public:
    void reset(float bx = 0.0f, float by = 0.0f);
    void setPossessionHint(BallMode mode);
    void step(float dt,
              bool hasMeasurement,
              float measX,
              float measY,
              float measRange,
              const SelfLocState& self,
              const EnemyState& enemy,
              uint32_t nowMs);

    // Returning by const reference avoids an 80-byte copy (incl. 4×4 P) on
    // every brain tick; callers only read a few scalars.
    [[nodiscard]] const BallEstimate& getEstimate() const { return out_; }

private:
    struct ModeFilter {
        float x[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        float P[4][4] = {};
    };

    ModeFilter mode_[3] = {};
    float mu_[3] = {1.0f, 0.0f, 0.0f};
    BallMode possessionHint_ = BALL_MODE_FREE;
    uint32_t lastSeenMs_ = 0;
    bool initialized_ = false;
    BallEstimate out_ = {};

    static void copy4x4(float dst[4][4], const float src[4][4]);
    static void identity4x4(float out[4][4]);
    static void addDiag4(float m[4][4], float d0, float d1, float d2, float d3);

    void predictFree(ModeFilter& f, float dt) const;
    void predictFriendly(ModeFilter& f, float dt, const SelfLocState& self) const;
    void predictEnemy(ModeFilter& f, float dt, const EnemyState& enemy) const;
    void combineOutput(float innovationMag);
};

#endif // BUCKY_IMMBALLTRACKER_H

