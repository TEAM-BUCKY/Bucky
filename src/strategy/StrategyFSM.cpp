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

void StrategyFSM::computeParabolicApproach(const DigitalField& field,
                                           const float speed,
                                           float* vxBody,
                                           float* vyBody)
{
    // Parabolic approach: the robot curves behind the ball and scoops it
    // into the front cage in one smooth motion.
    //
    // The velocity field is shaped so that:
    //   - Lateral (vx): steers the robot toward the ball's x position
    //   - Forward (vy): ramps up quadratically with lateral alignment
    //
    // When far off laterally: mostly sideways movement (getting aligned)
    // When aligned: mostly forward movement (approaching from behind)
    // The resulting path traces a parabola: y ∝ x²

    const float dx = field.ball.bx - field.self.x;
    const float dy = field.ball.by - field.self.y;
    const float absDx = fabsf(dx);

    // Lateral alignment: 1.0 when perfectly aligned, 0.0 at ≥50cm offset
    constexpr float kAlignDist = 0.5f;
    const float alignment = fmaxf(0.0f, 1.0f - absDx / kAlignDist);

    float vFieldX, vFieldY;

    if (dy > -0.05f)
    {
        // Ball is ahead or roughly at the same y: parabolic approach
        // Lateral: proportional control toward ball's x
        vFieldX = clampf(dx * 200.0f, -speed, speed);

        // Forward: quadratic ramp with alignment = parabolic path shape
        // alignment² ensures the robot only drives forward when laterally aligned
        vFieldY = speed * alignment * alignment;

        // Ensure some forward progress toward the ball even when offset
        vFieldY = fmaxf(vFieldY, fminf(dy * 40.0f, speed * 0.3f));
    }
    else
    {
        // Ball is behind: drive backward to get behind it, then curve
        vFieldX = clampf(dx * 150.0f, -speed * 0.7f, speed * 0.7f);
        vFieldY = clampf(dy * 80.0f, -speed, 0.0f);
    }

    // Normalize to speed limit
    const float vMag = sqrtf(vFieldX * vFieldX + vFieldY * vFieldY);
    if (vMag > speed)
    {
        vFieldX *= speed / vMag;
        vFieldY *= speed / vMag;
    }

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

    if (lineDetected)
        out.state = STATE_LINE_AVOID;
    else if (stuckDetected)
        out.state = STATE_STUCK_RECOVERY;
    else if (field.ball.mu[BALL_MODE_ENEMY] > 0.6f)
        out.state = STATE_DEFEND;
    else if (field.ball.mu[BALL_MODE_FRIENDLY] > 0.6f)
    {
        const float ballDistToGoal = hypotf(0.0f - field.ball.bx, 0.9f - field.ball.by);
        if (ballDistToGoal < 0.6f) out.state = STATE_SHOOT;
        else out.state = STATE_DRIBBLE;
    }
    else if (!field.ball.visible)
        out.state = (current == STATE_FIND_BALL) ? STATE_FIND_BALL : STATE_RETURN_POSITION;
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
            computeParabolicApproach(field, 80.0f, &vxBody, &vyBody);
            break;

        case STATE_ORBIT_BALL:
            computeParabolicApproach(field, 70.0f, &vxBody, &vyBody);
            break;

        case STATE_DRIBBLE:
            moveToFieldPoint(field, 0.0f, 0.9f, 65.0f, &vxBody, &vyBody);
            break;

        case STATE_SHOOT:
            moveToFieldPoint(field, 0.0f, 0.9f, 100.0f, &vxBody, &vyBody);
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
