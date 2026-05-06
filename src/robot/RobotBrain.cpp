#include "RobotBrain.h"

#include <cmath>

#include "helpers/Math.h"

void RobotBrain::reset(const float x0, const float y0, const float theta0)
{
    selfloc_reset(&selfLoc_, x0, y0, theta0);
    ballImm_.reset();
    enemyTracker_.reset();
    field_ = {};
    lastDrive_ = {0.0f, 0.0f};
    lastRotation_ = 0.0f;
    gameState_ = STATE_FIND_BALL;
}

BrainCommand RobotBrain::tick(const BrainSensors& s)
{
    // --- Predict self-localization from previous drive commands ---
    selfloc_predict(&selfLoc_, s.dt,
                    lastDrive_.x * kCmdToMps,
                    lastDrive_.y * kCmdToMps,
                    lastRotation_ * kCmdToRadS);

    // --- Compass update ---
    // Sum |drive|+|rotation| from the previous tick as the "motors busy"
    // signal for the EKF's noise-switch (SelfLocalizationEKF.cpp busy/idle R).
    if (s.compass_ready) {
        const float totalPwmDuty = fabsf(lastDrive_.x) + fabsf(lastDrive_.y) + fabsf(lastRotation_);
        selfloc_update_compass(&selfLoc_, s.compass_heading_rad, totalPwmDuty);
    }

    // --- Sonar update + enemy detection ---
    if (s.sonar_ready)
    {
        for (uint8_t i = 0; i < SONAR_COUNT; ++i)
        {
            // Skip invalid readings (-1 sentinel from main.cpp) — otherwise the
            // EKF wastes a Jacobian step on garbage and may spuriously flag an
            // enemy at the sensor origin.
            if (s.sonar_distance_m[i] <= 0.0f)
                continue;
            const SonarUpdateResult result = selfloc_update_sonar(&selfLoc_, i,
                                                            s.sonar_distance_m[i]);
            if (result.anomaly_detected)
                enemyTracker_.update(result.obstacle_x, result.obstacle_y, s.now_ms);
        }
        enemyTracker_.decay(s.now_ms);
    }

    // --- Get estimated states ---
    const SelfLocState selfState = selfloc_get_state(&selfLoc_);
    const EnemyState enemyState = enemyTracker_.getState();

    // --- Ball tracker ---
    ballImm_.setPossessionHint(s.possession_hint);
    ballImm_.step(s.dt, s.ball_visible,
                  s.ball_field_x, s.ball_field_y, s.ball_range_m,
                  selfState, enemyState, s.now_ms);

    const BallEstimate& est = ballImm_.getEstimate();

    // --- Populate DigitalField ---
    field_.self.x      = selfState.x;
    field_.self.y      = selfState.y;
    field_.self.theta  = selfState.theta;
    field_.self.vx     = selfState.vx;
    field_.self.vy     = selfState.vy;
    field_.self.omega  = selfState.omega;
    field_.self.P_xy   = selfState.P_xy;

    field_.ball.bx             = est.bx;
    field_.ball.by             = est.by;
    field_.ball.bvx            = est.bvx;
    field_.ball.bvy            = est.bvy;
    field_.ball.mu[0]          = est.mu[0];
    field_.ball.mu[1]          = est.mu[1];
    field_.ball.mu[2]          = est.mu[2];
    field_.ball.P_xy           = est.P[0][0] + est.P[1][1];
    field_.ball.innovation_mag = est.innovation_mag;
    field_.ball.visible        = est.visible;
    field_.ball.lost_ms        = est.lost_ms;

    // Derive ball-in-body-frame + range once per tick so strategy, drive
    // helpers, and main.cpp's possession-hint block don't each repeat the
    // same hypotf + cordic_sin_cos against the same self/ball delta.
    digitalFieldRefreshBallCache(field_);

    field_.enemy[0].x          = enemyState.x;
    field_.enemy[0].y          = enemyState.y;
    field_.enemy[0].vx         = enemyState.vx;
    field_.enemy[0].vy         = enemyState.vy;
    field_.enemy[0].confidence = enemyState.confidence;
    field_.timestamp_ms        = s.now_ms;

    // Store latest sonar distances (convert m → mm for drive module). Invalid
    // slots map to the 2400 mm "no wall seen" sentinel defined in DigitalField
    // so hard-stop / wall-avoidance treat a dead sensor as "far = safe"
    // instead of "wall at 0 mm".
    if (s.sonar_ready)
        for (uint8_t i = 0; i < SONAR_COUNT; ++i)
            field_.sonar_mm[i] = (s.sonar_distance_m[i] > 0.0f)
                                     ? s.sonar_distance_m[i] * 1000.0f
                                     : 2400.0f;

    // --- Strategy ---
    auto [drive, rotation, state] = StrategyFSM::update(field_, gameState_,
                                                        false, s.stuck_detected,
                                                        s.ir_s0_locked);
    gameState_    = state;
    lastDrive_    = drive;
    lastRotation_ = rotation;

    return {drive, rotation, state};
}
