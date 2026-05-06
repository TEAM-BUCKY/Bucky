#ifndef BUCKY_DIGITALFIELD_H
#define BUCKY_DIGITALFIELD_H

#include <cmath>
#include <cstdint>

#include "helpers/Math.h"
#include "io/cordic/cordic.h"

enum BallMode : uint8_t {
    BALL_MODE_FREE = 0,
    BALL_MODE_FRIENDLY = 1,
    BALL_MODE_ENEMY = 2,
};

typedef struct DigitalField_s {
    struct {
        float x = 0.0f;
        float y = 0.0f;
        float theta = 0.0f;
        float vx = 0.0f;
        float vy = 0.0f;
        float omega = 0.0f;
        float P_xy = 0.0f;
    } self;

    struct {
        float bx = 0.0f;
        float by = 0.0f;
        float bvx = 0.0f;
        float bvy = 0.0f;
        float mu[3] = {1.0f, 0.0f, 0.0f};
        float P_xy = 0.0f;
        float innovation_mag = 0.0f;
        uint8_t visible = 0;
        uint16_t lost_ms = 0;
        // Derived each tick in RobotBrain::tick so strategy, main loop
        // possession-hint, and drive helpers can read them without redoing
        // hypotf + cordic_sin_cos for the same ball-to-self delta.
        float bxBody = 0.0f;   // body-frame ball delta x (m)
        float byBody = 0.0f;   // body-frame ball delta y (m)
        float distM  = 0.0f;   // hypotf(bdx, bdy) in m
    } ball;

    struct {
        float x = 0.0f;
        float y = 0.0f;
        float vx = 0.0f;
        float vy = 0.0f;
        float confidence = 0.0f;
    } enemy[2];

    struct {
        float x_min = -1.2f;
        float x_max = 1.2f;
        float y_min = -0.9f;
        float y_max = 0.9f;
        float goal_width = 0.40f;
    } field;

    struct {
        float x = 0.0f;
        float y = 0.0f;
        float theta = 0.0f;
        uint8_t role = 0;
        uint8_t state = 0;
        uint8_t valid = 0;
    } teammate;

    // Latest sonar wall distances in mm (front, right, back, left).
    // Set to a large value (e.g. 2400) when no reading is available.
    float sonar_mm[4] = {2400.0f, 2400.0f, 2400.0f, 2400.0f};

    uint32_t timestamp_ms = 0;
} DigitalField;

FORCE_INLINE float fieldClampX(const DigitalField& field, const float x)
{
    return clampf(x, field.field.x_min + 0.05f, field.field.x_max - 0.05f);
}

FORCE_INLINE float fieldClampY(const DigitalField& field, const float y)
{
    return clampf(y, field.field.y_min + 0.05f, field.field.y_max - 0.05f);
}

FORCE_INLINE void bodyToField(const float vxBody, const float vyBody, const float theta,
                              float* vxField, float* vyField)
{
    float s, c;
    cordic_sin_cos(theta, &s, &c);
    *vxField = vxBody * c - vyBody * s;
    *vyField = vxBody * s + vyBody * c;
}

FORCE_INLINE void fieldToBody(const float vxField, const float vyField, const float theta,
                              float* vxBody, float* vyBody)
{
    float s, c;
    cordic_sin_cos(theta, &s, &c);
    *vxBody = vxField * c + vyField * s;
    *vyBody = -vxField * s + vyField * c;
}

// Populate the per-tick ball derived-values cache (body-frame delta + range).
// Must be called after ball.bx/by and self.x/y/theta are set, before any
// consumer (StrategyFSM, possession-hint) reads the cache. Centralised so
// test fixtures and the real brain stay in sync.
FORCE_INLINE void digitalFieldRefreshBallCache(DigitalField& f)
{
    const float bdx = f.ball.bx - f.self.x;
    const float bdy = f.ball.by - f.self.y;
    f.ball.distM = hypotf(bdx, bdy);
    fieldToBody(bdx, bdy, f.self.theta, &f.ball.bxBody, &f.ball.byBody);
}

#endif // BUCKY_DIGITALFIELD_H

