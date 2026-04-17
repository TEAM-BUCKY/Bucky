#ifndef BUCKY_SIMLOOP_H
#define BUCKY_SIMLOOP_H

#include "GroundTruth.h"
#include "SensorSim.h"
#include "robot/RobotBrain.h"

class SimLoop {
public:
    SimLoop();

    void reset();
    void step(float dt);

    GroundTruth& groundTruth() { return truth_; }
    const GroundTruth& groundTruth() const { return truth_; }
    const DigitalField& belief() const { return brain_.field(); }
    GameState_t gameState() const { return brain_.gameState(); }
    const char* gameStateName() const;

    SensorSim& sensorSim() { return sensorSim_; }
    const SensorSim& sensorSim() const { return sensorSim_; }

    bool paused = false;
    int speedMultiplier = 1;

private:
    GroundTruth truth_;
    SensorSim sensorSim_;
    RobotBrain brain_;

    VectorXY driveCmd_ = {0.0f, 0.0f};
    float rotationCmd_ = 0.0f;
};

#endif // BUCKY_SIMLOOP_H
