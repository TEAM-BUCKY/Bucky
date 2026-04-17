#ifndef BUCKY_ROBOTBRAIN_H
#define BUCKY_ROBOTBRAIN_H

#ifndef SONAR_COUNT
#define SONAR_COUNT 4
#endif

#include "control/pos/SelfLocalizationEKF.h"
#include "control/pos/EnemyTracker.h"
#include "control/ball/IMMBallTracker.h"
#include "strategy/StrategyFSM.h"
#include "field/DigitalField.h"

// Sensor measurements passed into the brain each tick.
// Hardware main.cpp and the simulation both fill this struct
// from their respective sensor sources.
struct BrainSensors {
    float dt;
    uint32_t now_ms;

    // Compass
    bool compass_ready;
    float compass_heading_rad;

    // Sonar (4 sensors: front, right, back, left)
    bool sonar_ready;
    float sonar_distance_m[4];

    // Ball observation (already in field frame)
    bool ball_visible;
    float ball_field_x;
    float ball_field_y;
    float ball_range_m;

    // Possession hint from dribbler contact sensor / simulation capture flag
    BallMode possession_hint;

    // External stuck detection (hardware: encoder watchdog; sim: ground truth
    // or left false). Propagates directly into StrategyFSM.
    bool stuck_detected;
};

// Drive command output from the brain.
struct BrainCommand {
    VectorXY drive;
    float rotation;
    GameState_t state;
};

// The shared algorithm pipeline.  Both the real main.cpp and the
// simulation instantiate one of these and call tick() each cycle.
class RobotBrain {
public:
    void reset(float x0 = 0.0f, float y0 = -0.2f, float theta0 = 0.0f);
    BrainCommand tick(const BrainSensors& sensors);

    const DigitalField& field() const { return field_; }
    GameState_t gameState() const { return gameState_; }

    // Expose for simulation heading override
    SelfLocalizationFilter& selfLoc() { return selfLoc_; }

private:
    static constexpr float kCmdToMps  = 0.012f;
    static constexpr float kCmdToRadS = 0.02f;

    SelfLocalizationFilter selfLoc_ = {};
    IMMBallTracker ballImm_;
    EnemyTracker enemyTracker_;
    StrategyFSM strategy_;
    DigitalField field_ = {};

    VectorXY lastDrive_ = {0.0f, 0.0f};
    float lastRotation_ = 0.0f;
    GameState_t gameState_ = STATE_FIND_BALL;
};

#endif // BUCKY_ROBOTBRAIN_H
