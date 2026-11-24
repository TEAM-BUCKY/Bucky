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


def ir_bearing_range(frame16, **config) -> tuple[float, float] | None:
    """NumPy port of ``IRRing::estimate`` (robot/src/hardware/sensors/ir/IRRing.cpp), defaults =
    ``IRConfig`` (see :func:`ir_estimate`). Returns (bearing rad, robot frame CW, distance cm from
    the robot centre), or None when no ball is seen."""
    r = ir_estimate(frame16, **config)
    return (r["angle"], r["distance_cm"]) if r["seen"] else None


def _bisect(lo: float, hi: float, f, iterations: int = 16) -> float:
    """``Solvers::bisect``: root of f in [lo, hi]; without a sign change, the end with smaller |f|."""
    f_lo, f_hi = f(lo), f(hi)
    if (f_lo > 0) == (f_hi > 0):
        return lo if abs(f_lo) <= abs(f_hi) else hi
    for _ in range(iterations):
        mid = 0.5 * (lo + hi)
        f_mid = f(mid)
        if (f_mid > 0) == (f_lo > 0):
            lo, f_lo = mid, f_mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def ir_estimate(frame16, *, baseline: float = 4.0, range_k: float = 760.0, n_sensors: int = 12,
                mount_radius_cm: float = 9.5, height_cm: float = 4.3, tilt_rad: float = 0.0,
                cos_power: float = 20.0, clip_code: float = 2300.0, min_amp: float = 8.0,
                min_neighbour_amp: float = 2.0, max_cm: float = 300.0) -> dict:
    """``IRRing::estimate`` step for step: peak sensor, far-field bearing from the neighbour
    ratio of the cosⁿ response, range along that bearing on the far side of the fold (the near
    side's answer in ``near_distance_cm``), then
    Gauss–Newton on (bearing, centre distance) over the peak and its neighbours with each sensor's
    real 3D position (ring parallax and height)."""
    min_cos = 1e-3
    n = int(n_sensors)
    step = 2 * math.pi / n
    raw = np.asarray(frame16[:n], float)
    amp = np.maximum(raw - baseline, 0.0)
    clip = raw >= clip_code
    peak = int(np.argmax(amp))
    out = {"seen": False, "clipped": False, "near_fold": False, "angle": 0.0, "distance_cm": 0.0,
           "near_distance_cm": 0.0,
           "peak_sensor": peak, "peak_amp": float(amp[peak]), "contributors": 0}
    if amp[peak] < min_amp:
        return out
    out["seen"] = True
    out["clipped"] = bool(clip[peak])
    left, right = (peak - 1) % n, (peak + 1) % n
    R, H = mount_radius_cm, height_cm
    cos_t, sin_t = math.cos(tilt_rad), math.sin(tilt_rad)

    offset = 0.0
    if not clip[peak]:
        right_stronger = amp[right] >= amp[left]
        j = right if right_stronger else left
        if amp[j] >= min_neighbour_amp:
            rho = math.log(amp[peak] / amp[j]) / cos_power
            if rho <= 0:
                d = step / 2
            elif rho >= -math.log(math.cos(step)):
                d = 0.0
            else:
                d = step / 2 - rho / step
                for _ in range(4):
                    f = math.log(math.cos(d)) - math.log(math.cos(step - d)) - rho
                    d -= f / (-math.tan(d) - math.tan(step - d))
                    d = min(max(d, 0.0), step / 2)
            offset = d if right_stronger else -d
    elif amp[left] >= min_neighbour_amp and amp[right] >= min_neighbour_amp:
        target = math.log(amp[right] / amp[left]) / cos_power
        d = 0.0
        for _ in range(6):
            f = math.log(math.cos(step - d)) - math.log(math.cos(step + d)) - target
            d -= f / (math.tan(step - d) + math.tan(step + d))
            d = min(max(d, -step / 2), step / 2)
        offset = d
    peak_angle = step * peak
    bearing = peak_angle + offset

    def model(p, b, rho):           # IRModel::lnAmp; p = sensor direction (x right, y forward)
        dx, dy = rho * math.sin(b) - R * p[0], rho * math.cos(b) - R * p[1]
        dd = math.sqrt(dx * dx + dy * dy + H * H)
        c = max((cos_t * (dx * p[0] + dy * p[1]) + sin_t * H) / dd, min_cos)
        return 2 * math.log(range_k) - 2 * math.log(dd) + cos_power * math.log(c)

    def fold_along(p, b):           # IRModel::foldDistance: where the slope changes sign
        slope = lambda r: model(p, b, r * 1.01) - model(p, b, r)
        return min_dist if slope(min_dist) <= 0 else _bisect(min_dist, max_cm, slope)

    peak_dir = (math.sin(peak_angle), math.cos(peak_angle))
    min_dist = R + 2.0
    fold = fold_along(peak_dir, bearing)
    ln_peak = math.log(amp[peak])
    dist = _bisect(fold, max_cm, lambda r: model(peak_dir, bearing, r) - ln_peak)
    out["near_distance_cm"] = _bisect(min_dist, fold, lambda r: model(peak_dir, bearing, r) - ln_peak)

    pts = []
    for k in (left, peak, right):
        if clip[k] or amp[k] < min_neighbour_amp:
            continue
        a = step * k
        pts.append((math.sin(a), math.cos(a), math.log(amp[k]), amp[k] ** 2))
    out["contributors"] = len(pts)

    if len(pts) >= 2:
        for _ in range(4):
            jtj = np.zeros((2, 2))
            jtr = np.zeros(2)
            hb, hr = 1e-3, 1e-2 * dist
            for p in pts:
                m = model(p, bearing, dist)
                jac = np.array([(model(p, bearing + hb, dist) - m) / hb,
                                (model(p, bearing, dist + hr) - m) / hr])
                jtj += p[3] * np.outer(jac, jac)
                jtr += p[3] * jac * (p[2] - m)
            det = jtj[0, 0] * jtj[1, 1] - jtj[0, 1] ** 2
            if abs(det) < 1e-12:
                break
            bearing += (jtj[1, 1] * jtr[0] - jtj[0, 1] * jtr[1]) / det
            dist += (jtj[0, 0] * jtr[1] - jtj[0, 1] * jtr[0]) / det
            bearing = min(max(bearing, peak_angle - step), peak_angle + step)
            dist = min(max(dist, fold), max_cm)

    if out["clipped"]:
        ln_clip = math.log(clip_code - baseline)
        axis = (0.0, 1.0)
        dist = min(dist, _bisect(fold_along(axis, 0.0), max_cm, lambda r: model(axis, 0.0, r) - ln_clip))
    out["angle"] = math.remainder(bearing, math.tau)
    out["distance_cm"] = min(dist, max_cm)
    out["near_fold"] = out["distance_cm"] < 1.25 * fold
    return out


def encoder_body_twist(tick_rates,
                       enc: sensors.EncoderModel = sensors.EncoderModel()) -> tuple[float, float, float]:
    """Encoder speeds (ticks/s, as encoder_get_speed reports) → rim speeds → body twist
    (Motors::wheelSpeeds geometry): (right cm/s, forward cm/s, ω rad/s clockwise), what
    EKF::updateSpeed takes."""
    tps = np.asarray(tick_rates, dtype=float)
    rim = tps / enc.ticks_per_rev * 2 * math.pi * enc.wheel_radius_m / np.asarray(enc.polarity)
    vx, vy, w = kin.body_twist_from_wheels(rim)
    return vx * 100.0, vy * 100.0, w


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
