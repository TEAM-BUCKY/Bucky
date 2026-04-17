#ifndef BUCKY_STRATEGYFSM_H
#define BUCKY_STRATEGYFSM_H

#include "field/DigitalField.h"
#include "drive/drive.h"

#ifndef BUCKY_MOTORDRIVER_H
#include "motor/MotorDriver.h"
#endif

enum GameState_t : uint8_t {
    STATE_FIND_BALL = 0,
    STATE_CHASE_BALL,
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
    static StrategyCommand update(const DigitalField& field, GameState_t current,
                                  bool lineDetected, bool stuckDetected);

private:
    // Convert field-frame target to body-frame mm and call drive_to_waypoint.
    static void driveToFieldPoint(const DigitalField& field, float tx, float ty,
                                  float speedMmS, float* vxBody, float* vyBody);
    // Convert ball to body-frame mm and call drive_to_point.
    static void driveToBall(const DigitalField& field, float speedMmS,
                            float* vxBody, float* vyBody);
    static float computeGoalHeadingRate(const DigitalField& field, float gain, float maxRate);
};

#endif // BUCKY_STRATEGYFSM_H
