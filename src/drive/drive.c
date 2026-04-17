/*
 * drive.c — Virtual-target drive-to-point module for Bucky
 *
 * Computes a translational velocity vector (vx, vy) that steers an
 * omni-directional, IMU-locked robot so that a target arrives centered
 * in the front-facing capture cage at (0, +R_robot).
 *
 * Pure math — no HAL, no RTOS, no dynamic allocation.
 */

#include "drive.h"
#include <math.h>

/* ------------------------------------------------------------------ */
/*  Constants                                                         */
/* ------------------------------------------------------------------ */

#define PI_F      3.14159265358979f
#define DEG2RAD   (PI_F / 180.0f)
#define RAD2DEG   (180.0f / PI_F)

/* EMA filter coefficient for bearing smoothing (0..1, lower = more smooth). */
#define ANGLE_EMA_ALPHA 0.35f

/* ------------------------------------------------------------------ */
/*  Helpers                                                           */
/* ------------------------------------------------------------------ */

static inline float clampf(float val, float lo, float hi)
{
    return fmaxf(lo, fminf(hi, val));
}

/* Hermite smoothstep — same formula used in MotorDriver for acceleration.
 * Returns 0 when x <= edge0, 1 when x >= edge1, smooth S-curve between. */
static inline float smoothstep(float x, float edge0, float edge1)
{
    float t = clampf((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

/* Wrap an angle in radians to (-pi, +pi]. */
static inline float wrap_pi(float a)
{
    while (a > PI_F)  a -= 2.0f * PI_F;
    while (a < -PI_F) a += 2.0f * PI_F;
    return a;
}

/* ------------------------------------------------------------------ */
/*  Persistent state (< 20 bytes, well within 128 B budget)           */
/* ------------------------------------------------------------------ */

static int8_t  orbit_dir;       /* +1 = CW (right), -1 = CCW (left), 0 = uncommitted */
static float   filtered_angle;  /* EMA-filtered bearing to virtual target (rad) */
static uint8_t filter_primed;   /* 0 = first sample not yet received */

/* ------------------------------------------------------------------ */
/*  Layer 1: Virtual target computation                               */
/* ------------------------------------------------------------------ */

/* Shift the real target backward (in -Y / away from goal) so the robot
 * arcs around the target and approaches from behind, letting the target
 * enter the front cage while the robot already faces the goal.
 *
 * The offset is blended out as the robot gets close, so final capture
 * drives straight at the real target. */
/* Shift the real target backward (in -Y / away from goal) so the robot
 * arcs around and approaches from behind.
 *
 * The offset depends on BOTH distance and bearing angle.  It only blends
 * out when the robot is close AND well-aligned with the cage.  If the
 * bearing is still large (robot approaching from the side), the offset
 * stays active even at close range, forcing the robot to keep orbiting
 * until it lines up with the cage opening. */
static void compute_virtual_target(float tx, float ty, float d,
                                   const DriveConfig* cfg,
                                   float* vt_x, float* vt_y)
{
    float bearing     = atan2f(tx, ty);  /* bearing to real target */
    float abs_bearing = fabsf(bearing);

    /* dist_blend:  0 when close, 1 when far */
    float dist_blend  = smoothstep(d, cfg->d_close, cfg->d_far);

    /* angle_blend: 0 when aligned (< capture_angle), 1 when misaligned (> 50°).
     * Keeps the offset active while the approach angle is too wide for the cage. */
    float capture_rad = cfg->capture_angle * DEG2RAD;
    float angle_blend = smoothstep(abs_bearing, capture_rad, 50.0f * DEG2RAD);

    /* Offset stays if EITHER far away OR misaligned. */
    float blend = fmaxf(dist_blend, angle_blend);

    /* Cap the offset so it never exceeds 60% of the distance to the real
     * target.  Without this, a close ball (e.g. 50 mm ahead) gets its
     * virtual target pushed 120 mm behind — past the robot — which flips
     * the bearing to ~150° and sends the orbit into reverse. */
    float raw_offset = cfg->behind_offset * blend;
    float max_offset = d * 0.6f;
    float eff_offset = fminf(raw_offset, max_offset);

    *vt_x = tx;
    *vt_y = ty - eff_offset;
}

/* ------------------------------------------------------------------ */
/*  Layer 2: Clamped offset orbit drive                               */
/* ------------------------------------------------------------------ */

/* Core steering logic.  Drives toward the (virtual) target using a
 * bearing-offset method with orbit commitment and hysteresis.
 *
 * Key insight: the 90-degree clamp on theta_drive means the robot NEVER
 * drives backward.  When the target is directly behind (theta ~= 180),
 * the clamped drive angle is 90 — pure sideways — which initiates the
 * orbit that brings the target into the front hemisphere.  Without this
 * clamp, the robot would drive backward toward the target and never
 * capture it in the front cage. */
static DriveCmd compute_orbit_drive(float vt_x, float vt_y, float d,
                                    const float sonar_mm[4],
                                    const DriveConfig* cfg)
{
    DriveCmd cmd = {0.0f, 0.0f, 0.0f};

    /* --- Bearing to virtual target ---
     * atan2(x, y) instead of atan2(y, x) because +Y is the forward axis. */
    float theta_raw = atan2f(vt_x, vt_y);

    /* --- EMA filter with wrap-around handling ---
     * If the angle jumps by more than pi between cycles (e.g. target
     * crosses the -Y axis), reset the filter instead of interpolating
     * across the discontinuity, which would produce a wild average. */
    if (!filter_primed) {
        filtered_angle = theta_raw;
        filter_primed = 1;
    } else {
        float diff = wrap_pi(theta_raw - filtered_angle);
        if (fabsf(diff) > PI_F * 0.95f) {
            /* Discontinuity — reset rather than blend across ±180°. */
            filtered_angle = theta_raw;
        } else {
            filtered_angle = wrap_pi(filtered_angle + ANGLE_EMA_ALPHA * diff);
        }
    }
    float theta = filtered_angle;

    float abs_theta     = fabsf(theta);
    float capture_rad   = cfg->capture_angle * DEG2RAD;
    float hyst_rad      = cfg->hyst_band * DEG2RAD;
    float k_offset_rad  = cfg->k_offset * DEG2RAD;

    /* --- Speed modulation ---
     * Ramp down on approach to prevent overshooting the capture zone. */
    float speed = cfg->max_speed;
    if (d < 250.0f) {
        speed = cfg->max_speed * (d / 250.0f);
    }
    /* Never stall completely — keep a minimum creep speed. */
    speed = fmaxf(speed, cfg->max_speed * 0.15f);
    /* Extra braking for the final alignment window. */
    if (abs_theta < 50.0f * DEG2RAD && d < 150.0f) {
        speed *= 0.8f;
    }

    /* --- Capture cone: drive straight when target is within cone --- */
    if (abs_theta < capture_rad) {
        /* Target inside the front cone — go straight, no orbit. */
        orbit_dir = 0;
        float inv_d = (d > 1e-4f) ? (1.0f / d) : 0.0f;
        cmd.vx    = speed * vt_x * inv_d;
        cmd.vy    = speed * vt_y * inv_d;
        cmd.speed = speed;
        return cmd;
    }

    /* --- Orbit direction commitment with hysteresis ---
     *
     * Once the robot commits to orbiting CW or CCW, it must not flip
     * on every cycle (which causes oscillation, especially when the
     * target is near ±180°).  The latch only flips when the bearing
     * crosses the *opposite* side by more than hyst_band degrees.
     *
     * Wall-aware tie-breaker: when the target is almost directly behind
     * (|theta| > 170°), choose the orbit direction that moves the robot
     * away from the nearest side wall, avoiding a wall jam. */
    if (orbit_dir == 0) {
        /* Uncommitted — pick initial direction. */
        if (abs_theta > 170.0f * DEG2RAD) {
            /* Near-180° ambiguity: use wall distances to break the tie. */
            float right_wall = sonar_mm[1];
            float left_wall  = sonar_mm[3];
            orbit_dir = (right_wall >= left_wall) ? +1 : -1;
        } else {
            orbit_dir = (theta > 0.0f) ? +1 : -1;
        }
    } else {
        /* Already committed — only flip if bearing crossed far enough. */
        if (orbit_dir == +1 && theta < -(capture_rad + hyst_rad)) {
            orbit_dir = -1;
        } else if (orbit_dir == -1 && theta > (capture_rad + hyst_rad)) {
            orbit_dir = +1;
        }
    }

    /* --- Offset computation ---
     * The offset grows with the absolute bearing angle using a power
     * curve: lazy at small misalignment, aggressive at large.
     * offset = k_offset * (|theta|/pi)^n_power                      */
    float frac   = abs_theta / PI_F;              /* 0..1 */
    float offset = k_offset_rad * powf(frac, cfg->n_power);

    /* --- Total drive angle ---
     * Add the offset in the orbit direction.  The 90° clamp ensures the
     * robot never drives backward — at worst it drives pure sideways,
     * which is exactly what we want for the 180° (ball-behind) case.  */
    float half_pi   = PI_F * 0.5f;
    float theta_mag = fminf(abs_theta + offset, half_pi);
    float theta_drive = (float)orbit_dir * theta_mag;

    /* --- Convert to velocity vector --- */
    cmd.vx    = speed * sinf(theta_drive);
    cmd.vy    = speed * cosf(theta_drive);
    cmd.speed = speed;
    return cmd;
}

/* ------------------------------------------------------------------ */
/*  Wall avoidance overlay                                            */
/* ------------------------------------------------------------------ */

/* After computing the orbit drive vector, blend in repulsive forces
 * from nearby walls.  Each sonar that reads below wall_danger mm
 * contributes a push in the inward direction (away from that wall).
 *
 * The result is renormalized so we never exceed max_speed. */
static void apply_wall_avoidance(float* vx, float* vy,
                                 const float sonar_mm[4],
                                 const DriveConfig* cfg,
                                 float max_speed)
{
    if (cfg->wall_danger < 1.0f) return; /* disabled */

    /* Wall normals point inward (away from wall).
     *   sonar[0] = front (+Y wall) → push -Y
     *   sonar[1] = right (+X wall) → push -X
     *   sonar[2] = back  (-Y wall) → push +Y
     *   sonar[3] = left  (-X wall) → push +X   */
    static const float nx[4] = { 0.0f, -1.0f,  0.0f, +1.0f};
    static const float ny[4] = {-1.0f,  0.0f, +1.0f,  0.0f};

    float push_x = 0.0f;
    float push_y = 0.0f;

    for (int i = 0; i < 4; i++) {
        if (sonar_mm[i] < cfg->wall_danger) {
            float strength = cfg->wall_force * max_speed
                           * (1.0f - sonar_mm[i] / cfg->wall_danger);
            push_x += strength * nx[i];
            push_y += strength * ny[i];
        }
    }

    *vx += push_x;
    *vy += push_y;

    /* Renormalize so we never exceed max_speed. */
    float mag = sqrtf(*vx * *vx + *vy * *vy);
    if (mag > max_speed) {
        float scale = max_speed / mag;
        *vx *= scale;
        *vy *= scale;
    }
}

/* ------------------------------------------------------------------ */
/*  Public API                                                        */
/* ------------------------------------------------------------------ */

DriveConfig drive_default_config(void)
{
    DriveConfig cfg;
    cfg.max_speed     = 800.0f;
    cfg.capture_angle = 12.0f;
    cfg.behind_offset = 120.0f;
    cfg.d_close       = 250.0f;
    cfg.d_far         = 600.0f;
    cfg.k_offset      = 70.0f;
    cfg.n_power       = 1.3f;
    cfg.hyst_band     = 8.0f;
    cfg.wall_danger   = 200.0f;
    cfg.wall_force    = 0.5f;
    return cfg;
}

void drive_reset(void)
{
    orbit_dir      = 0;
    filtered_angle = 0.0f;
    filter_primed  = 0;
}

DriveCmd drive_to_point(float tx, float ty,
                        const float sonar_mm[4],
                        const DriveConfig* cfg)
{
    float d = sqrtf(tx * tx + ty * ty);

    /* Layer 1 — virtual target: offset behind the real target so the
     * robot arcs around and captures from the front. */
    float vt_x, vt_y;
    compute_virtual_target(tx, ty, d, cfg, &vt_x, &vt_y);

    /* Layer 2 — clamped offset orbit: steer toward virtual target with
     * orbit commitment, hysteresis, and the critical 90° clamp. */
    DriveCmd cmd = compute_orbit_drive(vt_x, vt_y, d, sonar_mm, cfg);

    /* Wall avoidance overlay. */
    apply_wall_avoidance(&cmd.vx, &cmd.vy, sonar_mm, cfg, cfg->max_speed);

    cmd.speed = sqrtf(cmd.vx * cmd.vx + cmd.vy * cmd.vy);
    return cmd;
}

DriveCmd drive_to_waypoint(float tx, float ty,
                           const float sonar_mm[4],
                           const DriveConfig* cfg)
{
    float d = sqrtf(tx * tx + ty * ty);

    /* No virtual target offset — drive directly toward the waypoint.
     * The orbit logic still handles approach angles and speed ramp. */
    DriveCmd cmd = compute_orbit_drive(tx, ty, d, sonar_mm, cfg);

    /* Wall avoidance overlay. */
    apply_wall_avoidance(&cmd.vx, &cmd.vy, sonar_mm, cfg, cfg->max_speed);

    cmd.speed = sqrtf(cmd.vx * cmd.vx + cmd.vy * cmd.vy);
    return cmd;
}
