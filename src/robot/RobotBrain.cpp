#include "RobotBrain.h"
#include "helpers/Math.h"

void RobotBrain::reset(float x0, float y0, float theta0)
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
    if (s.compass_ready)
        selfloc_update_compass(&selfLoc_, s.compass_heading_rad);

    // --- Sonar update + enemy detection ---
    if (s.sonar_ready)
    {
        for (int i = 0; i < SONAR_COUNT; ++i)
        {
            SonarUpdateResult result = selfloc_update_sonar(&selfLoc_, i,
                                                            s.sonar_distance_m[i]);
            if (result.anomaly_detected)
                enemyTracker_.update(result.obstacle_x, result.obstacle_y, s.now_ms);
        }
        enemyTracker_.decay(s.now_ms);
    }

    // --- Get estimated states ---
    SelfLocState selfState = selfloc_get_state(&selfLoc_);
    EnemyState enemyState = enemyTracker_.getState();

    // --- Ball tracker ---
    ballImm_.setPossessionHint(s.possession_hint);
    ballImm_.step(s.dt, s.ball_visible,
                  s.ball_field_x, s.ball_field_y, s.ball_range_m,
                  selfState, enemyState, s.now_ms);

    auto [bx, by, bvx, bvy, P, mu, innovation_mag, visible, lost_ms] =
        ballImm_.getEstimate();

    // --- Populate DigitalField ---
    field_.self.x      = selfState.x;
    field_.self.y      = selfState.y;
    field_.self.theta  = selfState.theta;
    field_.self.vx     = selfState.vx;
    field_.self.vy     = selfState.vy;
    field_.self.omega  = selfState.omega;
    field_.self.P_xy   = selfState.P_xy;

    field_.ball.bx             = bx;
    field_.ball.by             = by;
    field_.ball.bvx            = bvx;
    field_.ball.bvy            = bvy;
    field_.ball.mu[0]          = mu[0];
    field_.ball.mu[1]          = mu[1];
    field_.ball.mu[2]          = mu[2];
    field_.ball.P_xy           = P[0][0] + P[1][1];
    field_.ball.innovation_mag = innovation_mag;
    field_.ball.visible        = visible;
    field_.ball.lost_ms        = lost_ms;

    field_.enemy[0].x          = enemyState.x;
    field_.enemy[0].y          = enemyState.y;
    field_.enemy[0].vx         = enemyState.vx;
    field_.enemy[0].vy         = enemyState.vy;
    field_.enemy[0].confidence = enemyState.confidence;
    field_.timestamp_ms        = s.now_ms;

    // Store latest sonar distances (convert m → mm for drive module).
    if (s.sonar_ready)
        for (int i = 0; i < SONAR_COUNT; ++i)
            field_.sonar_mm[i] = s.sonar_distance_m[i] * 1000.0f;

    // --- Strategy ---
    auto [drive, rotation, state] = strategy_.update(field_, gameState_,
                                                     false, s.stuck_detected);
    gameState_    = state;
    lastDrive_    = drive;
    lastRotation_ = rotation;

    return {drive, rotation, state};
}
