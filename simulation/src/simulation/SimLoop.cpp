#include "SimLoop.h"

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
    brain_.reset(truth_.robot.x, truth_.robot.y, truth_.robot.theta);
    driveCmd_ = {0.0f, 0.0f};
    rotationCmd_ = 0.0f;
}

const char* SimLoop::gameStateName() const
{
    if (brain_.gameState() < sizeof(kStateNames) / sizeof(kStateNames[0]))
        return kStateNames[brain_.gameState()];
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

    // 3. Build BrainSensors from simulated data
    BrainSensors bs = {};
    bs.dt     = simDt;
    bs.now_ms = truth_.time_ms;

    bs.compass_ready      = sensors.compassReady;
    bs.compass_heading_rad = sensors.compassRad;

    bs.sonar_ready = sensors.sonarReady;
    for (int i = 0; i < 4; ++i)
        bs.sonar_distance_m[i] = sensors.sonarDistanceM[i];

    bs.ball_visible = sensors.ballVisible;
    bs.ball_field_x = sensors.ballFieldX;
    bs.ball_field_y = sensors.ballFieldY;
    bs.ball_range_m = sensors.ballRangeM;

    // Sim has no DMA channel rotation to calibrate for, so the IR-boot
    // workaround is always considered locked.
    bs.ir_s0_locked = true;

    // Use ground truth capture flag as possession hint — simulates
    // a dribbler contact sensor (IR break-beam, motor current, etc.)
    if (truth_.ball.captured)
        bs.possession_hint = BALL_MODE_FRIENDLY;
    else if (brain_.field().enemy[0].confidence > 0.35f)
        bs.possession_hint = BALL_MODE_ENEMY;
    else
        bs.possession_hint = BALL_MODE_FREE;

    // 4. Force heading to match ground truth.  The compass PD controller
    //    holds heading perfectly on the real robot, so there is no heading
    //    drift.  Without this, compass noise causes body-to-field mismatch.
    brain_.selfLoc().x[2] = truth_.robot.theta;

    // 5. Run the shared algorithm pipeline
    auto [drive, rotation, state] = brain_.tick(bs);
    driveCmd_    = drive;
    rotationCmd_ = rotation;
}
