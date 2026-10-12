"""Steps between a firmware driver and the EKF that the firmware does not implement yet.

The EKF (robot/src/strategy/EKF.h) wants field-frame measurements; the drivers deliver raw
sensor quantities. Until the robot has its own code for these conversions, the lab and the tests
use the ones here — each is small and deliberately simple, so a C++ port is straightforward.

All in the firmware frame: cm, x right, y towards the opponent goal, angles clockwise from +y.
"""
from __future__ import annotations

import math

import numpy as np

from bucky.firmware import kinematics as kin
from bucky.firmware import sensors
from bucky.game.field import ARENA_HALF_X, ARENA_HALF_Y

#: Arena walls in the firmware frame (cm): x = lateral, y = goal axis.
WALL_X_CM = ARENA_HALF_Y * 100.0
WALL_Y_CM = ARENA_HALF_X * 100.0


def ir_bearing_range(frame16, *, baseline: float = 100.0, threshold_ratio: float = 0.08,
                     min_peak: float = 100.0, range_k: float = 700.0,
                     n_sensors: int = 12) -> tuple[float, float] | None:
    """testIRPositioning's algorithm: weighted circular centroid of the squared
    baseline-subtracted amplitudes above ``threshold_ratio`` of the peak; range = K / sqrt(peak).
    Returns (bearing rad, robot frame CW, range cm), or None when no ball is seen."""
    amp = np.clip(np.asarray(frame16[:n_sensors], float) - baseline, 0.0, None)
    peak = float(amp.max())
    if peak < min_peak:
        return None
    thr = peak * threshold_ratio
    w = np.where(amp > thr, (amp - thr) ** 2, 0.0)
    ang = np.radians(360.0 / n_sensors * np.arange(n_sensors))
    return math.atan2(float(w @ np.sin(ang)), float(w @ np.cos(ang))), range_k / math.sqrt(peak)


def encoder_field_velocity(tick_rates, theta: float,
                           enc: sensors.EncoderModel = sensors.EncoderModel()) -> np.ndarray:
    """Encoder speeds (ticks/s, as encoder_get_speed reports) → rim speeds → body twist
    (Motors::wheelSpeeds geometry) → field velocity (cm/s) at heading ``theta``."""
    tps = np.asarray(tick_rates, dtype=float)
    rim = tps / enc.ticks_per_rev * 2 * math.pi * enc.wheel_radius_m / np.asarray(enc.polarity)
    vx, vy, _ = kin.body_twist_from_wheels(rim)
    right = np.array([math.cos(theta), -math.sin(theta)])
    fwd = np.array([math.sin(theta), math.cos(theta)])
    return (vx * right + vy * fwd) * 100.0


def sonar_localize(distances_cm, valid, theta: float, guess_cm,
                   model: sensors.SonarModel = sensors.SonarModel(),
                   corner_margin_cm: float = 15.0,
                   iterations: int = 3) -> tuple[float, float] | None:
    """Robot position from the four sonar ranges, at heading ``theta``.

    Each valid ray is traced from ``guess_cm`` (e.g. the current EKF estimate) to find which
    wall it should hit; that wall then fixes one coordinate: x = ±W - r·u_x or y = ±L - r·u_y,
    with r the range from the robot centre. Rays that would land near a corner are skipped
    (which wall they hit is ambiguous). The answer is fed back as the next guess
    (``iterations`` times), so a guess some centimetres off still picks the right walls.
    Returns None unless both x and y were fixed."""
    pos = None
    guess = (float(guess_cm[0]), float(guess_cm[1]))
    for _ in range(max(1, iterations)):
        new = _sonar_fix(distances_cm, valid, theta, guess, model, corner_margin_cm)
        if new is None:
            return pos
        if pos is not None and math.dist(new, pos) < 1e-3:
            return new
        pos = guess = new
    return pos


def _sonar_fix(distances_cm, valid, theta, guess_cm, model, corner_margin_cm):
    gx, gy = float(guess_cm[0]), float(guess_cm[1])
    xs: list[float] = []
    ys: list[float] = []
    for a_deg, d, ok in zip(model.angles_cw_deg, distances_cm, valid):
        if not ok or not math.isfinite(d):
            continue
        a = theta + math.radians(a_deg)
        ux, uy = math.sin(a), math.cos(a)
        r = d + model.mount_radius_cm
        tx = (math.copysign(WALL_X_CM, ux) - gx) / ux if abs(ux) > 1e-6 else math.inf
        ty = (math.copysign(WALL_Y_CM, uy) - gy) / uy if abs(uy) > 1e-6 else math.inf
        if tx <= 0 and ty <= 0:
            continue
        if tx < ty:
            hit_y = gy + tx * uy
            if abs(hit_y) > WALL_Y_CM - corner_margin_cm:
                continue
            xs.append(math.copysign(WALL_X_CM, ux) - r * ux)
        else:
            hit_x = gx + ty * ux
            if abs(hit_x) > WALL_X_CM - corner_margin_cm:
                continue
            ys.append(math.copysign(WALL_Y_CM, uy) - r * uy)
    if not xs or not ys:
        return None
    return float(np.mean(xs)), float(np.mean(ys))
