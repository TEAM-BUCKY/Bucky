#ifndef BUCKY_STRATEGYFSM_H
#define BUCKY_STRATEGYFSM_H

#include "field/DigitalField.h"
#include "motor/MotorDriver.h"

enum GameState_t : uint8_t {
    STATE_FIND_BALL = 0,
    STATE_CHASE_BALL,
    STATE_ORBIT_BALL,
    STATE_DRIBBLE,
    STATE_SHOOT,
    STATE_DEFEND,
    STATE_INTERCEPT,
    STATE_RETURN_POSITION,
    STATE_LINE_AVOID,
    STATE_STUCK_RECOVERY,
};

struct StrategyCommand {
    VectorXY drive = {0.0f, 0.0f};
    float rotation = 0.0f;
    GameState_t state = STATE_FIND_BALL;
};

class StrategyFSM
{
public:
    StrategyCommand update(const DigitalField& field, GameState_t current, bool lineDetected, bool stuckDetected);

private:
    static void moveToFieldPoint(const DigitalField& field, float tx, float ty, float speed,
                                 float* vxBody, float* vyBody);
    static void computeOrbitVelocity(const DigitalField& field, float* vxBody, float* vyBody);
};

#endif // BUCKY_STRATEGYFSM_H

