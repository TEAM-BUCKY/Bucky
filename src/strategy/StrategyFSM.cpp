#include "StrategyFSM.h"

#include <cmath>

// Command scale: cmd 100 → 100 * 0.012 = 1.2 m/s = 1200 mm/s.
// So mm/s → cmd = mm/s / 12.0, and cmd → mm/s = cmd * 12.0.
static constexpr float kMmSToCmd = 1.0f / 12.0f;
static constexpr float kCmdToMmS = 12.0f;

// Shared drive config — tuned once, used by all states.
static const DriveConfig kDriveCfg = drive_default_config();

namespace
{
float wrapAnglePi(float a)
{
    while (a > PI_F) a -= 2.0f * PI_F;
    while (a < -PI_F) a += 2.0f * PI_F;
    return a;
}
} // namespace

void StrategyFSM::driveToFieldPoint(const DigitalField& field,
                                    const float tx, const float ty,
                                    const float speedMmS,
                                    float* vxBody, float* vyBody)
{
    // Convert field-frame target to body-frame, in mm.
    const float dx = tx - field.self.x;
    const float dy = ty - field.self.y;
    float bx, by;
    fieldToBody(dx, dy, field.self.theta, &bx, &by);
    bx *= 1000.0f;  // m → mm
    by *= 1000.0f;

    // Use a temporary config with the requested speed.
    DriveConfig cfg = kDriveCfg;
    cfg.max_speed = speedMmS;

    DriveCmd cmd = drive_to_waypoint(bx, by, field.sonar_mm, &cfg);

    // Convert mm/s output back to command scale (0..100).
    *vxBody = cmd.vx * kMmSToCmd;
    *vyBody = cmd.vy * kMmSToCmd;
}

void StrategyFSM::driveToBall(const DigitalField& field,
                               const float speedMmS,
                               float* vxBody, float* vyBody)
{
    // Convert ball position from field frame to body frame, in mm.
    const float dx = field.ball.bx - field.self.x;
    const float dy = field.ball.by - field.self.y;
    float bx, by;
    fieldToBody(dx, dy, field.self.theta, &bx, &by);
    bx *= 1000.0f;
    by *= 1000.0f;

    DriveConfig cfg = kDriveCfg;
    cfg.max_speed = speedMmS;

    DriveCmd cmd = drive_to_point(bx, by, field.sonar_mm, &cfg);

    *vxBody = cmd.vx * kMmSToCmd;
    *vyBody = cmd.vy * kMmSToCmd;
}

float StrategyFSM::computeGoalHeadingRate(const DigitalField& field,
                                           const float gain,
                                           const float maxRate)
{
    constexpr float kGoalX = 0.0f;
    constexpr float kGoalY = 0.9f;
    const float gx = kGoalX - field.self.x;
    const float gy = kGoalY - field.self.y;
    const float goalYaw = atan2f(gy, gx);

    const float forwardYaw = (PI_F * 0.5f) - field.self.theta;
    const float err = wrapAnglePi(goalYaw - forwardYaw);
    return clampf(gain * err, -maxRate, maxRate);
}

StrategyCommand StrategyFSM::update(const DigitalField& field,
                                    const GameState_t current,
                                    const bool lineDetected,
                                    const bool stuckDetected)
{
    StrategyCommand out = {};
    out.state = current;

    // Direct proximity check: ball close and in front of cage → we have it,
    // regardless of what the IMM mode probability says.
    const float bdx = field.ball.bx - field.self.x;
    const float bdy = field.ball.by - field.self.y;
    const float ballDist = hypotf(bdx, bdy);
    float bxBody = 0.0f, byBody = 0.0f;
    fieldToBody(bdx, bdy, field.self.theta, &bxBody, &byBody);
    const bool ballInCage = (ballDist < 0.15f && byBody > 0.0f
                             && fabsf(bxBody) < 0.05f);

    const bool hasBall = (field.ball.mu[BALL_MODE_FRIENDLY] > 0.6f) || ballInCage;

    if (lineDetected)
        out.state = STATE_LINE_AVOID;
    else if (stuckDetected)
        out.state = STATE_STUCK_RECOVERY;
    else if (field.ball.mu[BALL_MODE_ENEMY] > 0.6f)
        out.state = STATE_DEFEND;
    else if (hasBall)
    {
        const float ballDistToGoal = hypotf(0.0f - field.ball.bx, 0.9f - field.ball.by);
        if (ballDistToGoal < 0.6f) out.state = STATE_SHOOT;
        else out.state = STATE_DRIBBLE;
    }
    else if (!field.ball.visible)
        out.state = (current == STATE_FIND_BALL) ? STATE_FIND_BALL : STATE_RETURN_POSITION;
    else
    {
        out.state = STATE_CHASE_BALL;
    }

    float vxBody = 0.0f;
    float vyBody = 0.0f;
    float rot = 0.0f;

    switch (out.state)
    {
        case STATE_FIND_BALL:
            rot = 25.0f;
            break;

        case STATE_CHASE_BALL:
            // cmd 80 → 80 * 12 = 960 mm/s
            driveToBall(field, 80.0f * kCmdToMmS, &vxBody, &vyBody);
            rot = computeGoalHeadingRate(field, 20.0f, 30.0f);
            break;

        case STATE_DRIBBLE:
            driveToFieldPoint(field, 0.0f, 0.9f, 65.0f * kCmdToMmS, &vxBody, &vyBody);
            rot = computeGoalHeadingRate(field, 25.0f, 30.0f);
            break;

        case STATE_SHOOT:
            driveToFieldPoint(field, 0.0f, 0.9f, 100.0f * kCmdToMmS, &vxBody, &vyBody);
            rot = computeGoalHeadingRate(field, 30.0f, 35.0f);
            break;

        case STATE_DEFEND: {
            constexpr float goalX = 0.0f;
            constexpr float goalY = -0.9f;
            const float gx = field.ball.bx - goalX;
            const float gy = field.ball.by - goalY;
            const float gd = fmaxf(0.05f, hypotf(gx, gy));
            float tx = goalX + (gx / gd) * fminf(0.30f, gd * 0.4f);
            const float ty = goalY + (gy / gd) * fminf(0.30f, gd * 0.4f);
            tx = clampf(tx, -0.4f, 0.4f);
            driveToFieldPoint(field, tx, ty, 60.0f * kCmdToMmS, &vxBody, &vyBody);
            break;
        }

        case STATE_INTERCEPT:
            driveToFieldPoint(field,
                              field.ball.bx + field.ball.bvx * 0.20f,
                              field.ball.by + field.ball.bvy * 0.20f,
                              80.0f * kCmdToMmS, &vxBody, &vyBody);
            break;

        case STATE_RETURN_POSITION:
            driveToFieldPoint(field, 0.0f, -0.3f, 50.0f * kCmdToMmS, &vxBody, &vyBody);
            break;

        case STATE_LINE_AVOID:
            vxBody = -20.0f;
            vyBody = -50.0f;
            rot = 0.0f;
            break;

        case STATE_STUCK_RECOVERY:
            vxBody = 0.0f;
            vyBody = -70.0f;
            rot = 25.0f;
            break;

        default:
            break;
    }

    out.drive = {clampf(vxBody, -100.0f, 100.0f), clampf(vyBody, -100.0f, 100.0f)};
    out.rotation = clampf(rot, -35.0f, 35.0f);
    return out;
}
