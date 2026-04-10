#include "SimLoop.h"
#include "helpers/Math.h"

static const char* kStateNames[] = {
    "FIND_BALL",
    "CHASE_BALL",
    "ORBIT_BALL",
    "DRIBBLE",
    "SHOOT",
    "DEFEND",
    "INTERCEPT",
    "RETURN_POS",
    "LINE_AVOID",
    "STUCK_RECOVER",
};

SimLoop::SimLoop()
{
    reset();
}

void SimLoop::reset()
{
    truth_.reset();
    selfloc_reset(&selfLoc_, truth_.robot.x, truth_.robot.y, truth_.robot.theta);
    ballImm_.reset();
    enemyTracker_.reset();
    field_ = {};
    driveCmd_ = {0.0f, 0.0f};
    rotationCmd_ = 0.0f;
    gameState_ = STATE_FIND_BALL;
}

const char* SimLoop::gameStateName() const
{
    if (gameState_ < sizeof(kStateNames) / sizeof(kStateNames[0]))
        return kStateNames[gameState_];
    return "UNKNOWN";
}

void SimLoop::step(float dt)
{
    if (paused) return;

    float simDt = dt * static_cast<float>(speedMultiplier);

    // 1. Physics: update ground truth from previous motor commands
    truth_.stepPhysics(simDt, driveCmd_, rotationCmd_);

    // 2. Sensor simulation
    SimulatedSensors sensors = sensorSim_.simulate(truth_, simDt);
    uint32_t now = truth_.time_ms;

    // 3. Robot algorithm pipeline (mirrors Bucky/src/main.cpp lines 89-175)

    constexpr float kCmdToMps = 0.012f;

    // Predict self-localization.
    // Omega is 0: the compass PD controller holds heading toward the enemy
    // goal, so strategy rotation commands don't produce actual rotation.
    selfloc_predict(&selfLoc_, simDt,
                    driveCmd_.x * kCmdToMps,
                    driveCmd_.y * kCmdToMps,
                    0.0f);

    // Force heading to match ground truth. The compass PD controller holds
    // heading perfectly in the real robot, so there is no heading drift.
    // Without this, even tiny compass noise causes a body-to-field mismatch
    // between EKF prediction and ground truth, and the position drifts
    // until sonar rejects all corrections as "anomalies."
    selfLoc_.x[2] = truth_.robot.theta;

    // Sonar update
    if (sensors.sonarReady)
    {
        for (int i = 0; i < SONAR_COUNT; ++i)
        {
            SonarUpdateResult result = selfloc_update_sonar(&selfLoc_, i, sensors.sonarDistanceM[i]);
            if (result.anomaly_detected)
                enemyTracker_.update(result.obstacle_x, result.obstacle_y, now);
        }
        enemyTracker_.decay(now);
    }

    // Get self state
    SelfLocState selfState = selfloc_get_state(&selfLoc_);
    EnemyState enemyState = enemyTracker_.getState();

    // Ball measurement
    bool hasBallMeasurement = sensors.ballVisible;
    float ballMx = sensors.ballFieldX;
    float ballMy = sensors.ballFieldY;
    float ballRangeM = sensors.ballRangeM;

    // IMM ball tracker.
    // Use ground truth capture flag as possession hint — this simulates
    // a dribbler contact sensor (IR break-beam, motor current, etc.)
    // that the real robot would use to detect ball-in-gap.
    if (truth_.ball.captured)
        ballImm_.setPossessionHint(BALL_MODE_FRIENDLY);
    else if (enemyState.confidence > 0.35f)
        ballImm_.setPossessionHint(BALL_MODE_ENEMY);
    else
        ballImm_.setPossessionHint(BALL_MODE_FREE);
    ballImm_.step(simDt, hasBallMeasurement, ballMx, ballMy, ballRangeM,
                  selfState, enemyState, now);

    auto [bx, by, bvx, bvy, P, mu, innovation_mag, visible, lost_ms] = ballImm_.getEstimate();

    // 4. Populate DigitalField
    field_.self.x = selfState.x;
    field_.self.y = selfState.y;
    field_.self.theta = selfState.theta;
    field_.self.vx = selfState.vx;
    field_.self.vy = selfState.vy;
    field_.self.omega = selfState.omega;
    field_.self.P_xy = selfState.P_xy;

    field_.ball.bx = bx;
    field_.ball.by = by;
    field_.ball.bvx = bvx;
    field_.ball.bvy = bvy;
    field_.ball.mu[0] = mu[0];
    field_.ball.mu[1] = mu[1];
    field_.ball.mu[2] = mu[2];
    field_.ball.P_xy = P[0][0] + P[1][1];
    field_.ball.innovation_mag = innovation_mag;
    field_.ball.visible = visible;
    field_.ball.lost_ms = lost_ms;

    field_.enemy[0].x = enemyState.x;
    field_.enemy[0].y = enemyState.y;
    field_.enemy[0].vx = enemyState.vx;
    field_.enemy[0].vy = enemyState.vy;
    field_.enemy[0].confidence = enemyState.confidence;
    field_.timestamp_ms = now;

    // 5. Strategy
    auto [drive, rotation, state] = strategy_.update(field_, gameState_, false, false);
    gameState_ = state;
    driveCmd_ = drive;
    rotationCmd_ = rotation;
}
