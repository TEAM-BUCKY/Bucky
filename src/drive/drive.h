#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float vx;       /* mm/s, +X = rightward                      */
    float vy;       /* mm/s, +Y = forward (toward cage / goal)   */
    float speed;    /* mm/s, magnitude (for telemetry)            */
} DriveCmd;

typedef struct {
    float max_speed;       /* mm/s, maximum drive speed (e.g. 800)      */
    float capture_angle;   /* deg, cone where we drive straight (e.g. 12) */
    float behind_offset;   /* mm, virtual target offset (e.g. 150)      */
    float d_close;         /* mm, blend near threshold (e.g. 120)       */
    float d_far;           /* mm, blend far threshold (e.g. 500)        */
    float k_offset;        /* deg, max orbit offset (e.g. 70)           */
    float n_power;         /* exponent for offset curve (e.g. 1.3)      */
    float hyst_band;       /* deg, orbit direction hysteresis (e.g. 8)  */
    float wall_danger;     /* mm, wall avoidance activation dist (e.g. 200) */
    float wall_force;      /* 0..1, wall push strength (e.g. 0.5)       */
} DriveConfig;

/* Return a default config with recommended tuning values. */
DriveConfig drive_default_config(void);

/* Reset persistent state (call on game start, after pickup, etc.). */
void drive_reset(void);

/* Main entry point -- call at 1 kHz.
 * tx, ty:      target position in robot frame (mm), +Y = forward.
 * sonar_mm[4]: wall distances (front, right, back, left) in mm.
 * Returns:     velocity command in robot frame. */
DriveCmd drive_to_point(float tx, float ty,
                        const float sonar_mm[4],
                        const DriveConfig* cfg);

/* Variant for static waypoints -- no virtual target offset,
 * just the clamped orbit drive + wall avoidance. */
DriveCmd drive_to_waypoint(float tx, float ty,
                           const float sonar_mm[4],
                           const DriveConfig* cfg);

#ifdef __cplusplus
}
#endif

#endif /* DRIVE_H */
