#include "StrategyFSM.h"

#include <cmath>

void StrategyFSM::moveToFieldPoint(const DigitalField& field,
                                   const float tx,
                                   const float ty,
                                   const float speed,
                                   float* vxBody,
                                   float* vyBody)
{
    const float dx = tx - field.self.x;
    const float dy = ty - field.self.y;
    const float dist = sqrtf(dx * dx + dy * dy);
    if (dist < 1.0e-4f)
    {
        *vxBody = 0.0f;
        *vyBody = 0.0f;
        return;
    }

    const float vFieldX = (dx / dist) * speed;
    const float vFieldY = (dy / dist) * speed;
    fieldToBody(vFieldX, vFieldY, field.self.theta, vxBody, vyBody);
}

void StrategyFSM::computeOrbitVelocity(const DigitalField& field, float* vxBody, float* vyBody)
{
    const float dx = field.ball.bx - field.self.x;
    const float dy = field.ball.by - field.self.y;
    const float dist = sqrtf(dx * dx + dy * dy);

    const float ballToGoal = atan2f(0.9f - field.ball.by, 0.0f - field.ball.bx);
    const float robotToBall = atan2f(dy, dx);
    const float idealApproach = Math::wrapRadians(ballToGoal + PI_F);
    const float err = Math::wrapRadians(robotToBall - idealApproach);
    const float orbitSign = (err >= 0.0f) ? 1.0f : -1.0f;

    const float tangentialWeight = fminf(fabsf(err), 1.0f);
    const float radialWeight = 1.0f - tangentialWeight;

    const float tangAngle = robotToBall + orbitSign * (0.5f * PI_F);
    const float moveAngle = atan2f(
        tangentialWeight * sinf(tangAngle) + radialWeight * sinf(robotToBall),
        tangentialWeight * cosf(tangAngle) + radialWeight * cosf(robotToBall));

    const float speed = fminf(dist * 220.0f, 70.0f);
    const float vFieldX = speed * cosf(moveAngle);
    const float vFieldY = speed * sinf(moveAngle);
    fieldToBody(vFieldX, vFieldY, field.self.theta, vxBody, vyBody);
}

StrategyCommand StrategyFSM::update(const DigitalField& field,
                                    const GameState_t current,
                                    const bool lineDetected,
                                    const bool stuckDetected)
{
    StrategyCommand out = {};
    out.state = current;

    const float dx = field.ball.bx - field.self.x;
    const float dy = field.ball.by - field.self.y;
    const float ballDist = sqrtf(dx * dx + dy * dy);
    const float ballToGoal = atan2f(0.9f - field.ball.by, -field.ball.bx);
    const float myToBall = atan2f(dy, dx);
    const float approachError = fabsf(Math::wrapRadians(myToBall - ballToGoal - PI_F));

    if (lineDetected)
        out.state = STATE_LINE_AVOID;
    else if (stuckDetected)
        out.state = STATE_STUCK_RECOVERY;
    else if (field.ball.mu[BALL_MODE_ENEMY] > 0.6f)
        out.state = STATE_DEFEND;
    else if (field.ball.mu[BALL_MODE_FRIENDLY] > 0.6f)
    {
        const float ballDistToGoal = hypotf(0.0f - field.ball.bx, 0.9f - field.ball.by);
        if (approachError < 0.35f && ballDistToGoal < 0.6f) out.state = STATE_SHOOT;
        else if (approachError < 0.5f) out.state = STATE_DRIBBLE;
        else out.state = STATE_ORBIT_BALL;
    }
    else if (!field.ball.visible)
        out.state = (current == STATE_FIND_BALL) ? STATE_FIND_BALL : STATE_RETURN_POSITION;
    else if (ballDist > 0.5f)
        out.state = STATE_CHASE_BALL;
    else if (approachError > 0.6f)
        out.state = STATE_ORBIT_BALL;
    else
        out.state = STATE_CHASE_BALL;

    float vxBody = 0.0f;
    float vyBody = 0.0f;
    float rot = 0.0f;

    switch (out.state)
    {
        case STATE_FIND_BALL:
            rot = 25.0f;
            break;

        case STATE_CHASE_BALL:
            moveToFieldPoint(field, field.ball.bx, field.ball.by, 80.0f, &vxBody, &vyBody);
            rot = clampf(Math::radiansToDegrees(Math::wrapRadians(myToBall - field.self.theta)) * 0.35f, -25.0f, 25.0f);
            break;

        case STATE_ORBIT_BALL:
            computeOrbitVelocity(field, &vxBody, &vyBody);
            rot = clampf(Math::radiansToDegrees(Math::wrapRadians(myToBall - field.self.theta)) * 0.30f, -20.0f, 20.0f);
            break;

        case STATE_DRIBBLE:
            moveToFieldPoint(field, 0.0f, 0.9f, 65.0f, &vxBody, &vyBody);
            rot = 0.0f;
            break;

        case STATE_SHOOT:
            moveToFieldPoint(field, 0.0f, 0.9f, 100.0f, &vxBody, &vyBody);
            rot = 0.0f;
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
            moveToFieldPoint(field, tx, ty, 60.0f, &vxBody, &vyBody);
            rot = clampf(Math::radiansToDegrees(Math::wrapRadians(myToBall - field.self.theta)) * 0.4f, -20.0f, 20.0f);
            break;
        }

        case STATE_INTERCEPT:
            moveToFieldPoint(field, field.ball.bx + field.ball.bvx * 0.20f, field.ball.by + field.ball.bvy * 0.20f,
                             80.0f, &vxBody, &vyBody);
            break;

        case STATE_RETURN_POSITION:
            moveToFieldPoint(field, 0.0f, -0.3f, 50.0f, &vxBody, &vyBody);
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
    }

    out.drive = {clampf(vxBody, -100.0f, 100.0f), clampf(vyBody, -100.0f, 100.0f)};
    out.rotation = clampf(rot, -35.0f, 35.0f);
    return out;
}

