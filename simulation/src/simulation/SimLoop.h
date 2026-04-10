#ifndef BUCKY_SIMLOOP_H
#define BUCKY_SIMLOOP_H

#include "GroundTruth.h"
#include "SensorSim.h"

#include "control/pos/SelfLocalizationEKF.h"
#include "control/pos/EnemyTracker.h"
#include "control/ball/IMMBallTracker.h"
#include "control/ball/IRBallProcessor.h"
#include "strategy/StrategyFSM.h"
#include "field/DigitalField.h"

class SimLoop {
public:
    SimLoop();

    void reset();
    void step(float dt);

    GroundTruth& groundTruth() { return truth_; }
    const GroundTruth& groundTruth() const { return truth_; }
    const DigitalField& belief() const { return field_; }
    GameState_t gameState() const { return gameState_; }
    const char* gameStateName() const;

    SensorSim& sensorSim() { return sensorSim_; }
    const SensorSim& sensorSim() const { return sensorSim_; }

    bool paused = false;
    int speedMultiplier = 1;

private:
    GroundTruth truth_;
    SensorSim sensorSim_;

    // Robot algorithm instances (actual code from Bucky/src/)
    SelfLocalizationFilter selfLoc_ = {};
    IMMBallTracker ballImm_;
    EnemyTracker enemyTracker_;
    StrategyFSM strategy_;
    DigitalField field_ = {};

    VectorXY driveCmd_ = {0.0f, 0.0f};
    float rotationCmd_ = 0.0f;
    GameState_t gameState_ = STATE_FIND_BALL;
};

#endif // BUCKY_SIMLOOP_H
