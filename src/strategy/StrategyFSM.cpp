#include "StrategyFSM.h"

#include <cmath>

#include "io/cordic/cordic.h"
#include "helpers/Math.h"

// Command scale: cmd 100 → 100 * 0.012 = 1.2 m/s = 1200 mm/s.
// So mm/s → cmd = mm/s / 12.0, and cmd → mm/s = cmd * 12.0.
static constexpr float kMmSToCmd = 1.0f / 12.0f;
static constexpr float kCmdToMmS = 12.0f;

// Friendly goal (target) position in field frame — hoisted to file scope so
// the compiler can keep the constants in flash and all uses agree.
static constexpr float kGoalX = 0.0f;
static constexpr float kGoalY = 0.9f;

// Shared drive config — default tuning loaded once at init, `max_speed` is
// overwritten per state in-place to avoid the 40 B stack copy that used to
// happen on every driveToBall / driveToFieldPoint call. Safe because the
// brain runs single-threaded and the mutation is scoped to one call.
static DriveConfig sDriveCfg = drive_default_config();

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

    // Mutate shared config's max_speed in place — avoids copying the whole
    // struct every call. Safe: brain runs single-threaded.
    sDriveCfg.max_speed = speedMmS;
    DriveCmd cmd = drive_to_waypoint(bx, by, field.sonar_mm, &sDriveCfg);

    // Convert mm/s output back to command scale (0..100).
    *vxBody = cmd.vx * kMmSToCmd;
    *vyBody = cmd.vy * kMmSToCmd;
}

void StrategyFSM::driveToBall(const DigitalField& field,
                               const float speedMmS,
                               float* vxBody, float* vyBody)
{
    // Body-frame delta already computed once per tick in RobotBrain::tick.
    const float bx = field.ball.bxBody * 1000.0f;  // m → mm
    const float by = field.ball.byBody * 1000.0f;

    sDriveCfg.max_speed = speedMmS;
    DriveCmd cmd = drive_to_point(bx, by, field.sonar_mm, &sDriveCfg);

    *vxBody = cmd.vx * kMmSToCmd;
    *vyBody = cmd.vy * kMmSToCmd;
}

float StrategyFSM::computeGoalHeadingRate(const DigitalField& field,
                                           const float gain,
                                           const float maxRate)
{
    const float gx = kGoalX - field.self.x;
    const float gy = kGoalY - field.self.y;
    const float goalYaw = cordic_atan2(gy, gx);

    const float forwardYaw = (PI_F * 0.5f) - field.self.theta;
    const float err = Math::wrapRadians(goalYaw - forwardYaw);
    return clampf(gain * err, -maxRate, maxRate);
}

StrategyCommand StrategyFSM::update(const DigitalField& field,
                                    const GameState_t current,
                                    const bool lineDetected,
                                    const bool stuckDetected,
                                    const bool s0Locked)
{
    StrategyCommand out = {};
    out.state = current;

    // Direct proximity check: ball close and in front of cage → we have it,
    // regardless of what the IMM mode probability says. Body-frame delta and
    // range are already cached by RobotBrain::tick.
    const bool ballInCage = (field.ball.distM < 0.15f
                             && field.ball.byBody > 0.0f
                             && fabsf(field.ball.bxBody) < 0.05f);

    const bool hasBall = (field.ball.mu[BALL_MODE_FRIENDLY] > 0.6f) || ballInCage;

    if (lineDetected)
        out.state = STATE_LINE_AVOID;
    else if (stuckDetected)
        out.state = STATE_STUCK_RECOVERY;
    else if (field.ball.mu[BALL_MODE_ENEMY] > 0.6f)
        out.state = STATE_DEFEND;
    else if (hasBall)
    {
        const float ballDistToGoal = hypotf(kGoalX - field.ball.bx, kGoalY - field.ball.by);
        if (ballDistToGoal < 0.6f) out.state = STATE_SHOOT;
        else out.state = STATE_DRIBBLE;
    }
    else if (!field.ball.visible)
    {
        // Ball-lost policy splits on whether the one-shot S0 lock has fired.
        // Pre-lock: stay in FIND_BALL so the IR ring's first confident reading
        // can be trusted as "the sensor pointing at the ball is the front".
        // Post-lock: retreat to the defensive home point as before.
        if (!s0Locked)
            out.state = STATE_FIND_BALL;
        else
            out.state = STATE_RETURN_POSITION;
    }
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
            // Pre-S0-lock bootstrap: the IR channel offset hasn't been
            // confirmed yet, so we drive straight forward and wait for the
            // first confident reading. Compass PD holds heading at 0°, so
            // body +Y is physical forward. Once IRBallProcessor rotates its
            // offset to lock the first-firing sensor as index 0, the FSM
            // transitions out and this state is unreachable until reboot.
            vxBody = 0.0f;
            vyBody = 30.0f;
            rot    = 0.0f;
            break;

        case STATE_CHASE_BALL:
            // cmd 80 → 80 * 12 = 960 mm/s. Heading is held at 0° by the outer
            // compass PD — omni-drive handles the lateral component, no yaw
            // command needed here.
            driveToBall(field, 80.0f * kCmdToMmS, &vxBody, &vyBody);
            break;

        case STATE_DRIBBLE:
            driveToFieldPoint(field, kGoalX, kGoalY, 65.0f * kCmdToMmS, &vxBody, &vyBody);
            break;

        case STATE_SHOOT:
            driveToFieldPoint(field, kGoalX, kGoalY, 100.0f * kCmdToMmS, &vxBody, &vyBody);
            break;

        case STATE_DEFEND: {
            constexpr float goalX = 0.0f;
            constexpr float goalY = -0.9f;
            // Block along goal→ball by default. When the enemy tracker is
            // confident, block along goal→enemy instead — the enemy's shot
            // line matters more than wherever the loose ball currently sits.
            float refX = field.ball.bx;
            float refY = field.ball.by;
            if (field.enemy[0].confidence > 0.5f) {
                refX = field.enemy[0].x;
                refY = field.enemy[0].y;
            }
            const float gx = refX - goalX;
            const float gy = refY - goalY;
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
            // Back up gently, hold heading. Old values (-70 Y + 25 rot) were
            // a spin-and-run maneuver that combined badly with the outer
            // compass hold — translation scrub overwhelmed the rotation PD
            // and the robot corkscrewed into the wall. A calm retreat lets
            // the outer loop keep heading at 0° while the drive module
            // disengages from the obstacle.
            vxBody = 0.0f;
            vyBody = -30.0f;
            rot    = 0.0f;
            break;

        default:
            break;
    }

    out.drive = {clampf(vxBody, -100.0f, 100.0f), clampf(vyBody, -100.0f, 100.0f)};
    out.rotation = clampf(rot, -35.0f, 35.0f);
    return out;
}
