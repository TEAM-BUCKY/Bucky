#ifndef BUCKY_GROUNDTRUTH_H
#define BUCKY_GROUNDTRUTH_H

#include <cstdint>
#include <cmath>
#include "field/DigitalField.h"

struct GroundTruth {
    struct {
        float x = 0.0f, y = -0.3f, theta = 0.0f;
        float vx = 0.0f, vy = 0.0f, omega = 0.0f;
    } robot;

    struct {
        float x = 0.4f, y = 0.3f;
        float vx = 0.0f, vy = 0.0f;
        bool captured = false;
    } ball;

    struct EnemyGT {
        float x = 0.0f, y = 0.0f;
        float vx = 0.0f, vy = 0.0f;
        bool active = false;
    } enemies[2];

    uint32_t time_ms = 0;

    // Field bounds (from DigitalField defaults)
    static constexpr float X_MIN = -1.2f;
    static constexpr float X_MAX =  1.2f;
    static constexpr float Y_MIN = -0.9f;
    static constexpr float Y_MAX =  0.9f;

    static constexpr float BALL_FRICTION = 1.5f;
    static constexpr float BALL_RESTITUTION = 0.6f;
    static constexpr float ROBOT_RADIUS = 0.09f;
    static constexpr float BALL_RADIUS = 0.021f;
    static constexpr float ENEMY_RADIUS = 0.09f;

    // Dribble gap at the front of the robot.
    static constexpr float GAP_DEPTH = 0.035f;
    static constexpr float GAP_HALF_WIDTH = 0.04f;

    // Robot always faces the enemy goal (positive Y).
    static constexpr float HEADING_TOWARD_ENEMY_GOAL = static_cast<float>(M_PI) * 0.5f;

    void reset()
    {
        robot = {0.0f, -0.3f, HEADING_TOWARD_ENEMY_GOAL, 0.0f, 0.0f, 0.0f};
        ball = {0.4f, 0.3f, 0.0f, 0.0f, false};
        enemies[0] = {0.0f, 0.5f, 0.0f, 0.0f, true};
        enemies[1] = {-0.3f, 0.6f, 0.0f, 0.0f, false};
        time_ms = 0;
    }

    void stepPhysics(float dt, const VectorXY& driveCmd, float /*rotationCmd*/)
    {
        // --- Robot kinematics from motor commands ---
        constexpr float kCmdToMps = 0.012f;

        float vxBody = driveCmd.x * kCmdToMps;
        float vyBody = driveCmd.y * kCmdToMps;

        robot.theta = HEADING_TOWARD_ENEMY_GOAL;
        robot.omega = 0.0f;

        float c = cosf(robot.theta);
        float s = sinf(robot.theta);
        robot.vx = vxBody * c - vyBody * s;
        robot.vy = vxBody * s + vyBody * c;

        robot.x += robot.vx * dt;
        robot.y += robot.vy * dt;

        // --- Robot-enemy collision ---
        for (int i = 0; i < 2; ++i)
        {
            if (!enemies[i].active) continue;
            float ex = robot.x - enemies[i].x;
            float ey = robot.y - enemies[i].y;
            float edist = sqrtf(ex * ex + ey * ey);
            float minDist = ROBOT_RADIUS + ENEMY_RADIUS;
            if (edist < minDist && edist > 0.001f)
            {
                // Push robot out of enemy
                float nx = ex / edist;
                float ny = ey / edist;
                robot.x = enemies[i].x + nx * (minDist + 0.005f);
                robot.y = enemies[i].y + ny * (minDist + 0.005f);
            }
        }

        // Clamp robot to field
        robot.x = fmaxf(X_MIN + 0.05f, fminf(X_MAX - 0.05f, robot.x));
        robot.y = fmaxf(Y_MIN + 0.05f, fminf(Y_MAX - 0.05f, robot.y));

        // --- Ball physics ---
        if (ball.captured)
        {
            // Ball rides in the dribble gap
            float frontX = robot.x + (ROBOT_RADIUS - GAP_DEPTH + BALL_RADIUS) * cosf(robot.theta);
            float frontY = robot.y + (ROBOT_RADIUS - GAP_DEPTH + BALL_RADIUS) * sinf(robot.theta);

            // Release if the ball would go past a wall boundary
            if (frontX > X_MAX || frontX < X_MIN || frontY > Y_MAX || frontY < Y_MIN)
            {
                ball.captured = false;
                // Ball bounces off the wall
                if (frontY > Y_MAX) { ball.y = Y_MAX - BALL_RADIUS; ball.vy = -fabsf(ball.vy) * BALL_RESTITUTION; }
                if (frontY < Y_MIN) { ball.y = Y_MIN + BALL_RADIUS; ball.vy = fabsf(ball.vy) * BALL_RESTITUTION; }
                if (frontX > X_MAX) { ball.x = X_MAX - BALL_RADIUS; ball.vx = -fabsf(ball.vx) * BALL_RESTITUTION; }
                if (frontX < X_MIN) { ball.x = X_MIN + BALL_RADIUS; ball.vx = fabsf(ball.vx) * BALL_RESTITUTION; }
            }
            else
            {
                ball.x = frontX;
                ball.y = frontY;
                ball.vx = robot.vx;
                ball.vy = robot.vy;
            }

            // Release if the robot is moving backward
            float forwardSpeed = robot.vx * cosf(robot.theta) + robot.vy * sinf(robot.theta);
            if (forwardSpeed < -0.05f)
                ball.captured = false;
        }
        else
        {
            float damp = fmaxf(0.0f, 1.0f - BALL_FRICTION * dt);
            ball.x += ball.vx * dt;
            ball.y += ball.vy * dt;
            ball.vx *= damp;
            ball.vy *= damp;

            // Wall bounce
            if (ball.x > X_MAX) { ball.x = 2.0f * X_MAX - ball.x; ball.vx *= -BALL_RESTITUTION; }
            if (ball.x < X_MIN) { ball.x = 2.0f * X_MIN - ball.x; ball.vx *= -BALL_RESTITUTION; }
            if (ball.y > Y_MAX) { ball.y = 2.0f * Y_MAX - ball.y; ball.vy *= -BALL_RESTITUTION; }
            if (ball.y < Y_MIN) { ball.y = 2.0f * Y_MIN - ball.y; ball.vy *= -BALL_RESTITUTION; }
        }

        // --- Ball-enemy collision ---
        for (int i = 0; i < 2; ++i)
        {
            if (!enemies[i].active) continue;
            float bex = ball.x - enemies[i].x;
            float bey = ball.y - enemies[i].y;
            float bedist = sqrtf(bex * bex + bey * bey);
            float minBE = BALL_RADIUS + ENEMY_RADIUS;
            if (bedist < minBE && bedist > 0.001f)
            {
                if (ball.captured) ball.captured = false;
                float nx = bex / bedist;
                float ny = bey / bedist;
                ball.x = enemies[i].x + nx * (minBE + 0.005f);
                ball.y = enemies[i].y + ny * (minBE + 0.005f);
                ball.vx = nx * 0.3f;
                ball.vy = ny * 0.3f;
            }
        }

        // --- Dribble gap capture check ---
        float dbx = ball.x - robot.x;
        float dby = ball.y - robot.y;
        float bdist = sqrtf(dbx * dbx + dby * dby);

        if (!ball.captured && bdist < ROBOT_RADIUS + BALL_RADIUS + 0.01f && bdist > 0.001f)
        {
            float fwd = dbx * cosf(robot.theta) + dby * sinf(robot.theta);
            float lat = -dbx * sinf(robot.theta) + dby * cosf(robot.theta);

            if (fwd > 0.0f && fabsf(lat) < GAP_HALF_WIDTH)
            {
                ball.captured = true;
                ball.vx = robot.vx;
                ball.vy = robot.vy;
            }
            else
            {
                float nx = dbx / bdist;
                float ny = dby / bdist;
                ball.x = robot.x + nx * (ROBOT_RADIUS + BALL_RADIUS + 0.005f);
                ball.y = robot.y + ny * (ROBOT_RADIUS + BALL_RADIUS + 0.005f);
                float pushSpeed = sqrtf(robot.vx * robot.vx + robot.vy * robot.vy);
                ball.vx = robot.vx * 0.5f + nx * pushSpeed * 0.3f;
                ball.vy = robot.vy * 0.5f + ny * pushSpeed * 0.3f;
            }
        }

        time_ms += static_cast<uint32_t>(dt * 1000.0f);
    }
};

#endif // BUCKY_GROUNDTRUTH_H
