#include "EKF.h"

#include <cmath>

static constexpr float G_TO_CM_S2 = 981.0f;
static constexpr float MIN_BALL_DISTANCE = 1.0f; // cm, avoid singular Jacobian when ball is on the robot

void EKF::init(const float robotX, const float robotY, const float robotTheta, const float positionStd, const float thetaStd)
{
    x = {};
    x[X] = robotX;
    x[Y] = robotY;
    x[THETA] = Math::wrapRadians(robotTheta);

    stateCovariance = {};
    stateCovariance[X][X] = positionStd * positionStd;
    stateCovariance[Y][Y] = positionStd * positionStd;
    stateCovariance[THETA][THETA] = thetaStd * thetaStd;
    stateCovariance[OMEGA][OMEGA] = noise.omegaInit * noise.omegaInit;
    stateCovariance[VX][VX] = noise.encoderSpeed * noise.encoderSpeed;
    stateCovariance[VY][VY] = noise.encoderSpeed * noise.encoderSpeed;
    // Ball unknown until first IR reading
    stateCovariance[BX][BX] = FIELD_WIDTH * FIELD_WIDTH;
    stateCovariance[BY][BY] = FIELD_HEIGHT * FIELD_HEIGHT;
    stateCovariance[BVX][BVX] = noise.ballSpeedInit * noise.ballSpeedInit;
    stateCovariance[BVY][BVY] = noise.ballSpeedInit * noise.ballSpeedInit;
    ballInitialized = false;

    for (std::size_t i = 0; i < SENSOR_COUNT; ++i)
    {
        consecutiveRejects[i] = 0;
        rejectCount[i] = 0;
    }

    syncState();
}


void EKF::predict(const float accelX_g, const float accelY_g, const float dt)
{
    const float s = std::sin(x[THETA]);
    const float c = std::cos(x[THETA]);
    const float omega = x[OMEGA];

    // Add back the centripetal accel of the off-centre IMU.
    const float w2 = omega * omega;
    const float ax = accelX_g * G_TO_CM_S2 + w2 * model.imuOffsetX;
    const float ay = accelY_g * G_TO_CM_S2 + w2 * model.imuOffsetY;

    // Acceleration in field frame (theta clockwise, robot forward = field (sin, cos))
    const float afx = c * ax + s * ay;
    const float afy = -s * ax + c * ay;
    // d(a_f)/domega through the centripetal term
    const float dafxdw = 2.0f * omega * (c * model.imuOffsetX + s * model.imuOffsetY);
    const float dafydw = 2.0f * omega * (-s * model.imuOffsetX + c * model.imuOffsetY);

    const float halfDt2 = 0.5f * dt * dt;
    const float ballDecay = model.ballFrictionTau > 0.0f ? std::exp(-dt / model.ballFrictionTau) : 1.0f;

    x[X] += x[VX] * dt + halfDt2 * afx;
    x[Y] += x[VY] * dt + halfDt2 * afy;
    x[VX] += afx * dt;
    x[VY] += afy * dt;
    x[THETA] = Math::wrapRadians(x[THETA] + omega * dt);
    x[BX] += x[BVX] * dt;
    x[BY] += x[BVY] * dt;
    x[BVX] *= ballDecay;
    x[BVY] *= ballDecay;

    // Jacobian F = df/dx. d(afx)/dtheta = afy, d(afy)/dtheta = -afx
    Math::Matrix<N, N> F = {};
    for (std::size_t i = 0; i < N; ++i) F[i][i] = 1.0f;
    F[X][VX] = dt;
    F[Y][VY] = dt;
    F[X][THETA] = halfDt2 * afy;
    F[Y][THETA] = -halfDt2 * afx;
    F[VX][THETA] = afy * dt;
    F[VY][THETA] = -afx * dt;
    F[X][OMEGA] = halfDt2 * dafxdw;
    F[Y][OMEGA] = halfDt2 * dafydw;
    F[VX][OMEGA] = dafxdw * dt;
    F[VY][OMEGA] = dafydw * dt;
    F[THETA][OMEGA] = dt;
    F[BX][BVX] = dt;
    F[BY][BVY] = dt;
    F[BVX][BVX] = ballDecay;
    F[BVY][BVY] = ballDecay;

    Math::Matrix<N, N> FP;
    Math::multiplyMatrix(F, stateCovariance, FP);
    Math::multiplyMatrixByTransposed(FP, F, stateCovariance);

    // Q: accel noise through G = [1/2 dt^2, dt] per axis.
    const float accelStd = noise.accel * (1.0f + noise.accelSpinGain * std::fabs(omega));
    const float qa = accelStd * accelStd;
    const float qPP = halfDt2 * halfDt2 * qa;
    const float qPV = halfDt2 * dt * qa;
    const float qVV = dt * dt * qa;
    stateCovariance[X][X] += qPP;
    stateCovariance[Y][Y] += qPP;
    stateCovariance[X][VX] += qPV;
    stateCovariance[VX][X] += qPV;
    stateCovariance[Y][VY] += qPV;
    stateCovariance[VY][Y] += qPV;
    stateCovariance[VX][VX] += qVV;
    stateCovariance[VY][VY] += qVV;

    // Heading: same white-noise acceleration model on (theta, omega), plus a small random walk floor
    const float qw = noise.angularAccel * noise.angularAccel;
    stateCovariance[THETA][THETA] += halfDt2 * halfDt2 * qw + noise.thetaDrift * noise.thetaDrift * dt;
    stateCovariance[THETA][OMEGA] += halfDt2 * dt * qw;
    stateCovariance[OMEGA][THETA] += halfDt2 * dt * qw;
    stateCovariance[OMEGA][OMEGA] += dt * dt * qw;

    // Ball: same white-noise acceleration model, with ballAccel
    const float qb = noise.ballAccel * noise.ballAccel;
    const float qbPP = halfDt2 * halfDt2 * qb;
    const float qbPV = halfDt2 * dt * qb;
    const float qbVV = dt * dt * qb;
    stateCovariance[BX][BX] += qbPP;
    stateCovariance[BY][BY] += qbPP;
    stateCovariance[BX][BVX] += qbPV;
    stateCovariance[BVX][BX] += qbPV;
    stateCovariance[BY][BVY] += qbPV;
    stateCovariance[BVY][BY] += qbPV;
    stateCovariance[BVX][BVX] += qbVV;
    stateCovariance[BVY][BVY] += qbVV;

    syncState();
}


bool EKF::updateScalar(const Math::Vector<N>& h, const float innovation, const float variance, const Sensor sensor)
{
    Math::Vector<N> PhT;
    Math::multiplyMatrixVector(stateCovariance, h, PhT); // P symmetric, so P h^T == (h P)^T

    float s = variance;
    for (std::size_t i = 0; i < N; ++i)
        s += h[i] * PhT[i];
    if (s <= 0.0f) return false;

    // Innovation gate. Encoders are ungated (slip biases them); accept again after maxRejects.
    if (sensor != ENCODER && innovation * innovation > model.gateChi2 * s)
    {
        ++rejectCount[sensor];
        if (consecutiveRejects[sensor] < model.maxRejects)
        {
            ++consecutiveRejects[sensor];
            return false;
        }
    }
    consecutiveRejects[sensor] = 0;

    const float invS = 1.0f / s;
    float gain[N];
    for (std::size_t r = 0; r < N; ++r)
        gain[r] = PhT[r] * invS;

    for (std::size_t r = 0; r < N; ++r)
        x[r] += gain[r] * innovation;
    x[THETA] = Math::wrapRadians(x[THETA]);

    // P <- P - g (h P), and h P == PhT^T
    for (std::size_t r = 0; r < N; ++r)
        for (std::size_t col = 0; col < N; ++col)
            stateCovariance[r][col] -= gain[r] * PhT[col];

    // Keep P symmetric against float drift
    for (std::size_t r = 0; r < N; ++r)
        for (std::size_t col = r + 1; col < N; ++col)
        {
            const float avg = 0.5f * (stateCovariance[r][col] + stateCovariance[col][r]);
            stateCovariance[r][col] = avg;
            stateCovariance[col][r] = avg;
        }
    return true;
}


bool EKF::updateIdentity(const std::size_t i, const float innovation, const float variance, const Sensor sensor)
{
    Math::Vector<N> h = {};
    h[i] = 1.0f;
    return updateScalar(h, innovation, variance, sensor);
}


void EKF::updatePosition(const float robotX, const float robotY)
{
    const float r = noise.sonarPos * noise.sonarPos;
    updateIdentity(X, robotX - x[X], r, SONAR_X);
    updateIdentity(Y, robotY - x[Y], r, SONAR_Y);
    syncState();
}


void EKF::updateHeading(const float robotTheta)
{
    updateIdentity(THETA, Math::wrapRadians(robotTheta - x[THETA]), noise.compassTheta * noise.compassTheta, COMPASS);
    syncState();
}


void EKF::updateSpeed(const float speedX, const float speedY, const float omega)
{
    // h(x) = R(theta)^T v_field, each row relinearized after the previous one.
    const float r = noise.encoderSpeed * noise.encoderSpeed;
    {
        const float s = std::sin(x[THETA]);
        const float c = std::cos(x[THETA]);
        Math::Vector<N> h = {};
        h[VX] = c;
        h[VY] = -s;
        h[THETA] = -s * x[VX] - c * x[VY];
        updateScalar(h, speedX - (c * x[VX] - s * x[VY]), r, ENCODER);
    }
    {
        const float s = std::sin(x[THETA]);
        const float c = std::cos(x[THETA]);
        Math::Vector<N> h = {};
        h[VX] = s;
        h[VY] = c;
        h[THETA] = c * x[VX] - s * x[VY];
        updateScalar(h, speedY - (s * x[VX] + c * x[VY]), r, ENCODER);
    }
    updateIdentity(OMEGA, omega - x[OMEGA], noise.encoderOmega * noise.encoderOmega, ENCODER);
    syncState();
}


void EKF::updateBall(const float angle, const float distance)
{
    const float distanceStd = std::fmax(noise.irDistance, noise.irDistanceRel * distance);

    if (!ballInitialized)
    {
        // First sighting: place the ball directly, linearizing around a guess far away is useless
        const float fieldAngle = x[THETA] + angle;
        x[BX] = x[X] + distance * std::sin(fieldAngle);
        x[BY] = x[Y] + distance * std::cos(fieldAngle);
        x[BVX] = 0.0f;
        x[BVY] = 0.0f;
        const float posVar = distanceStd * distanceStd + distance * distance * noise.irAngle * noise.irAngle;
        constexpr std::size_t ballIndices[] = {BX, BY, BVX, BVY};
        for (const std::size_t b : ballIndices)
            for (std::size_t i = 0; i < N; ++i)
                stateCovariance[b][i] = stateCovariance[i][b] = 0.0f;
        stateCovariance[BX][BX] = posVar + stateCovariance[X][X];
        stateCovariance[BY][BY] = posVar + stateCovariance[Y][Y];
        stateCovariance[BVX][BVX] = noise.ballSpeedInit * noise.ballSpeedInit;
        stateCovariance[BVY][BVY] = noise.ballSpeedInit * noise.ballSpeedInit;
        ballInitialized = true;
        syncState();
        return;
    }

    // h(x) = [atan2(dx, dy) - theta, sqrt(dx^2 + dy^2)],  d = ball - robot (angle clockwise from +y)
    const float dx = x[BX] - x[X];
    const float dy = x[BY] - x[Y];
    const float d = std::fmax(std::sqrt(dx * dx + dy * dy), MIN_BALL_DISTANCE);
    const float d2 = d * d;

    {
        Math::Vector<N> h = {};
        h[X] = -dy / d2;
        h[Y] = dx / d2;
        h[THETA] = -1.0f;
        h[BX] = dy / d2;
        h[BY] = -dx / d2;
        const float predicted = std::atan2(dx, dy) - x[THETA];
        updateScalar(h, Math::wrapRadians(angle - predicted), noise.irAngle * noise.irAngle, IR_ANGLE);
    }

    // Distance row, relinearized around the angle-corrected state
    {
        const float ex = x[BX] - x[X];
        const float ey = x[BY] - x[Y];
        const float e = std::fmax(std::sqrt(ex * ex + ey * ey), MIN_BALL_DISTANCE);

        Math::Vector<N> h = {};
        h[X] = -ex / e;
        h[Y] = -ey / e;
        h[BX] = ex / e;
        h[BY] = ey / e;
        updateScalar(h, distance - e, distanceStd * distanceStd, IR_DISTANCE);
    }

    syncState();
}


void EKF::syncState()
{
    state.robotX = x[X];
    state.robotY = x[Y];
    state.robotTheta = x[THETA];
    state.robotOmega = x[OMEGA];
    state.speedX = x[VX];
    state.speedY = x[VY];
    state.ballX = x[BX];
    state.ballY = x[BY];
    state.ballSpeedX = x[BVX];
    state.ballSpeedY = x[BVY];
}


void EKF::writeTo(DigitalField& field) const
{
    field.robot.x = state.robotX;
    field.robot.y = state.robotY;
    field.ball.x = state.ballX;
    field.ball.y = state.ballY;
}
