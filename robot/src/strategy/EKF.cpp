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
    stateCovariance[VX][VX] = noise.encoderSpeed * noise.encoderSpeed;
    stateCovariance[VY][VY] = noise.encoderSpeed * noise.encoderSpeed;
    // Ball unknown until first IR reading
    stateCovariance[BX][BX] = FIELD_WIDTH * FIELD_WIDTH;
    stateCovariance[BY][BY] = FIELD_HEIGHT * FIELD_HEIGHT;
    stateCovariance[BVX][BVX] = noise.ballSpeedInit * noise.ballSpeedInit;
    stateCovariance[BVY][BVY] = noise.ballSpeedInit * noise.ballSpeedInit;
    ballInitialized = false;

    syncState();
}


void EKF::predict(const float accelX_g, const float accelY_g, const float dt)
{
    // x_k = f(x_k-1, u_k)
    // p_k = F p_k-1 F^T + Q
    const float s = std::sin(x[THETA]);
    const float c = std::cos(x[THETA]);
    const float ax = accelX_g * G_TO_CM_S2;
    const float ay = accelY_g * G_TO_CM_S2;

    // Acceleration in field frame (theta clockwise, robot forward = field (sin, cos))
    const float afx = c * ax + s * ay;
    const float afy = -s * ax + c * ay;

    const float halfDt2 = 0.5f * dt * dt;

    x[X] += x[VX] * dt + halfDt2 * afx;
    x[Y] += x[VY] * dt + halfDt2 * afy;
    x[VX] += afx * dt;
    x[VY] += afy * dt;
    x[BX] += x[BVX] * dt;
    x[BY] += x[BVY] * dt;

    // Jacobian F = df/dx. d(afx)/dtheta = afy, d(afy)/dtheta = -afx
    Math::Matrix<N, N> F = {};
    for (std::size_t i = 0; i < N; ++i) F[i][i] = 1.0f;
    F[X][VX] = dt;
    F[Y][VY] = dt;
    F[X][THETA] = halfDt2 * afy;
    F[Y][THETA] = -halfDt2 * afx;
    F[VX][THETA] = afy * dt;
    F[VY][THETA] = -afx * dt;
    F[BX][BVX] = dt;
    F[BY][BVY] = dt;

    Math::Matrix<N, N> FP;
    Math::multiplyMatrix(F, stateCovariance, FP);
    Math::multiplyMatrixByTransposed(FP, F, stateCovariance);

    // Q: accel noise enters through G = [1/2 dt^2, dt] per axis (field frame, isotropic)
    const float qa = noise.accel * noise.accel;
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
    stateCovariance[THETA][THETA] += noise.thetaDrift * noise.thetaDrift * dt;

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


void EKF::updateScalar(const Math::Vector<N>& h, const float innovation, const float variance)
{
    // g_k = p_k h^T / (h p_k h^T + r)
    // x_k <- x_k + g_k (z_k - h(x_k))
    // p_k <- (1 - g_k h) p_k
    Math::Vector<N> PhT;
    Math::multiplyMatrixVector(stateCovariance, h, PhT); // P symmetric, so P h^T == (h P)^T

    float s = variance;
    for (std::size_t i = 0; i < N; ++i)
        s += h[i] * PhT[i];
    if (s <= 0.0f) return;

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
}


void EKF::updateIdentity(const std::size_t i, const float innovation, const float variance)
{
    Math::Vector<N> h = {};
    h[i] = 1.0f;
    updateScalar(h, innovation, variance);
}


void EKF::updatePosition(const float robotX, const float robotY)
{
    const float r = noise.sonarPos * noise.sonarPos;
    updateIdentity(X, robotX - x[X], r);
    updateIdentity(Y, robotY - x[Y], r);
    syncState();
}


void EKF::updateHeading(const float robotTheta)
{
    updateIdentity(THETA, Math::wrapRadians(robotTheta - x[THETA]), noise.compassTheta * noise.compassTheta);
    syncState();
}


void EKF::updateSpeed(const float speedX, const float speedY)
{
    const float r = noise.encoderSpeed * noise.encoderSpeed;
    updateIdentity(VX, speedX - x[VX], r);
    updateIdentity(VY, speedY - x[VY], r);
    syncState();
}


void EKF::updateBall(const float angle, const float distance)
{
    if (!ballInitialized)
    {
        // First sighting: place the ball directly, linearizing around a guess far away is useless
        const float fieldAngle = x[THETA] + angle;
        x[BX] = x[X] + distance * std::sin(fieldAngle);
        x[BY] = x[Y] + distance * std::cos(fieldAngle);
        x[BVX] = 0.0f;
        x[BVY] = 0.0f;
        const float posVar = noise.irDistance * noise.irDistance + distance * distance * noise.irAngle * noise.irAngle;
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

    // Angle row
    {
        Math::Vector<N> h = {};
        h[X] = -dy / d2;
        h[Y] = dx / d2;
        h[THETA] = -1.0f;
        h[BX] = dy / d2;
        h[BY] = -dx / d2;
        const float predicted = std::atan2(dx, dy) - x[THETA];
        updateScalar(h, Math::wrapRadians(angle - predicted), noise.irAngle * noise.irAngle);
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
        updateScalar(h, distance - e, noise.irDistance * noise.irDistance);
    }

    syncState();
}


void EKF::syncState()
{
    state.robotX = x[X];
    state.robotY = x[Y];
    state.robotTheta = x[THETA];
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
