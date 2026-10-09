"""Physical truth → raw sensor readings, for every sensor on the robot.

Each model is a frozen dataclass of the sensor's physics, mounting and noise; each helper turns
the truth (positions in the sim frame, headings, distances) into exactly what the firmware reads
from the chip: LSB counts, echo pulse widths, ADC counts. Pure numpy; pass a seeded
``np.random.Generator`` for noise (``rng=None`` → noise-free).

Mounting and calibration constants nobody has measured yet are marked UNVERIFIED. Each can be
checked on the robot with the matching firmware test program (testCompass, testSonar, testIR,
testLine, testEncoder) and the value here adjusted.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from bucky.firmware import frames
from bucky.game.field import ROBOT_RADIUS
from bucky.game.geometry import WHITE, ray_cast, surface_at

LineColor = ("red", "green", "blue", "dark")   # firmware LineColor order


def _noise(rng: np.random.Generator | None, sigma: float, shape=()) -> np.ndarray | float:
    if rng is None or sigma <= 0:
        return np.zeros(shape) if shape else 0.0
    return rng.normal(0.0, sigma, shape)


# ── Compass: LIS2MDL magnetometer ─────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class CompassModel:
    """Earth field seen by the LIS2MDL (1.5 mGauss/LSB).

    The field's horizontal component points ``north_deg`` counter-clockwise from the opponent
    goal.
    UNVERIFIED mounting: chip X = robot front, Y = robot left, so the firmware's
    ``atan2(Y, X)`` heading grows when the robot turns clockwise (its angle convention).
    ``axis_sign=-1`` mirrors Y if the chip turns out to be mounted upside down.
    Hard iron (``hard_iron_lsb``) adds, soft iron (``soft_iron``) scales each axis — what
    ``Compass::setCalibration`` undoes."""

    field_h_mgauss: float = 190.0          # horizontal component (the Netherlands ≈ 0.19 G)
    field_v_mgauss: float = 440.0          # vertical (down) component
    north_deg: float = 0.0
    yaw_offset_deg: float = 0.0            # chip rotated on the PCB
    axis_sign: float = 1.0
    hard_iron_lsb: tuple[float, float, float] = (0.0, 0.0, 0.0)
    soft_iron: tuple[float, float, float] = (1.0, 1.0, 1.0)
    noise_lsb: float = 2.0
    lsb_mgauss: float = 1.5


def compass_raw(heading_sim: float, m: CompassModel = CompassModel(),
                rng: np.random.Generator | None = None) -> np.ndarray:
    """Raw LIS2MDL X/Y/Z (int16 LSB) for a robot at sim heading ``heading_sim`` (rad, CCW)."""
    theta_cw = frames.sim_to_fw_heading(heading_sim)
    phi = math.radians(m.north_deg + m.yaw_offset_deg) + theta_cw
    h = m.field_h_mgauss / m.lsb_mgauss
    v = np.array([h * math.cos(phi), m.axis_sign * h * math.sin(phi),
                  -m.field_v_mgauss / m.lsb_mgauss])
    v = v * np.asarray(m.soft_iron) + np.asarray(m.hard_iron_lsb) + _noise(rng, m.noise_lsb, 3)
    return np.clip(np.round(v), -32768, 32767).astype(np.int16)


def compass_heading_ref(raw, offset=(0.0, 0.0, 0.0), scale=(1.0, 1.0, 1.0)) -> float:
    """What ``Compass::processRead`` computes from ``raw`` (degrees, [0, 360))."""
    cx = (float(raw[0]) - offset[0]) * scale[0]
    cy = (float(raw[1]) - offset[1]) * scale[1]
    h = math.degrees(math.atan2(cy, cx)) if (cx or cy) else 0.0
    return h + 360.0 if h < 0 else h


# ── Accelerometer: LSM303AGR ──────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class AccelModel:
    """LSM303AGR in high-resolution ±2 g mode (0.98 mg/LSB, 12 bits left-justified).
    UNVERIFIED mounting: chip X = robot right, Y = robot front, Z = up (the firmware body frame,
    as EKF::predict assumes)."""

    noise_mg: float = 4.0
    bias_mg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    mg_per_lsb: float = 0.98


def accel_raw(a_body_fw_g, m: AccelModel = AccelModel(),
              rng: np.random.Generator | None = None) -> np.ndarray:
    """Raw OUT_X/Y/Z words for a body-frame acceleration (g, firmware body frame: right, front),
    with gravity (+1 g) on Z."""
    a_mg = np.array([float(a_body_fw_g[0]), float(a_body_fw_g[1]), 1.0]) * 1000.0
    a_mg = a_mg + np.asarray(m.bias_mg) + _noise(rng, m.noise_mg, 3)
    counts = np.clip(np.round(a_mg / m.mg_per_lsb), -2048, 2047).astype(np.int32)
    return (counts * 16).astype(np.int16)   # left-justified: firmware does raw >> 4


# ── Sonar: 4 × ultrasonic ─────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class SonarModel:
    """UNVERIFIED: sensor i looks ``angles_cw_deg[i]`` clockwise from the robot front, mounted
    on the robot's rim. Echo time is distance / 0.017 cm/µs (Sonar.cpp's conversion)."""

    angles_cw_deg: tuple[float, ...] = (0.0, 90.0, 180.0, 270.0)
    mount_radius_cm: float = ROBOT_RADIUS * 100.0
    min_range_cm: float = 2.0
    max_range_cm: float = 400.0
    noise_cm: float = 0.5
    dropout: float = 0.0
    latency_us: float = 200.0
    cm_per_us: float = 0.017


def sonar_distances_cm(robot_m, heading_sim: float, m: SonarModel = SonarModel(), *,
                       robots=(), rng: np.random.Generator | None = None) -> np.ndarray:
    """Distance (cm) each sensor would measure to the arena walls / other robots (centres in
    ``robots``, sim frame). NaN = nothing within range (or a dropout)."""
    pos = np.asarray(robot_m, dtype=float)
    circles = [(np.asarray(r, float), ROBOT_RADIUS) for r in robots]
    out = np.empty(len(m.angles_cw_deg))
    for i, a in enumerate(m.angles_cw_deg):
        world = heading_sim - math.radians(a)
        direction = np.array([math.cos(world), math.sin(world)])
        d = ray_cast(pos, direction, circles=circles) * 100.0 - m.mount_radius_cm
        d += _noise(rng, m.noise_cm)
        dropped = rng is not None and m.dropout > 0 and rng.random() < m.dropout
        out[i] = math.nan if (dropped or d > m.max_range_cm) else max(d, m.min_range_cm)
    return out


def sonar_echo_us(distances_cm, m: SonarModel = SonarModel()) -> list[float]:
    """Echo pulse widths (µs) the firmware times; NaN stays NaN (no echo)."""
    return [d / m.cm_per_us if math.isfinite(d) else math.nan for d in distances_cm]


# ── IR ball ring (G port, 16-channel mux) ─────────────────────────────────────────────────────

@dataclass(frozen=True)
class IRModel:
    """IR ring as testIRPositioning reads it: sensor k looks k·``angle_step_deg`` clockwise from
    the front; idle level ``baseline``; a ball at distance d adds (``range_k`` / d)² to the
    sensor facing it — so the firmware's ``range = 700 / sqrt(peak)`` estimate is exact on the
    peak sensor — falling off as cos^``cos_power`` of the angle off-axis, plus ``crosstalk`` of
    the peak on every lit sensor. UNVERIFIED: falloff, crosstalk and noise."""

    n_sensors: int = 12
    angle_step_deg: float = 30.0
    baseline: float = 100.0
    range_k: float = 700.0
    cos_power: float = 2.0
    crosstalk: float = 0.0
    noise: float = 3.0
    min_dist_cm: float = 5.0
    unused_value: int = 0
    adc_max: int = 4095


def ir_adc(bearing_cw_rad: float, dist_cm: float, m: IRModel = IRModel(), *,
           visible: bool = True, rng: np.random.Generator | None = None) -> np.ndarray:
    """16 ADC values indexed by physical sensor (channels past ``n_sensors`` read
    ``unused_value``) for a ball at ``bearing_cw_rad`` (robot frame, 0 = front, CW positive)."""
    out = np.full(16, float(m.unused_value))
    vals = np.full(m.n_sensors, m.baseline)
    if visible:
        peak = (m.range_k / max(dist_cm, m.min_dist_cm)) ** 2
        for k in range(m.n_sensors):
            off = bearing_cw_rad - math.radians(k * m.angle_step_deg)
            c = math.cos(off)
            gain = c ** m.cos_power if c > 0 else 0.0
            vals[k] += peak * gain + (peak * m.crosstalk if gain > 0 else 0.0)
    vals = vals + _noise(rng, m.noise, m.n_sensors)
    out[: m.n_sensors] = vals
    return np.clip(np.round(out), 0, m.adc_max).astype(np.uint16)


# ── Line sensor ring (G port, 4 colour sweeps × 16) ────────────────────────────────────────────

@dataclass(frozen=True)
class LineModel:
    """16 reflectance sensors on a ring under the robot, sensor k at k·22.5° clockwise from the
    front. Each colour sweep reads ambient (``dark``) plus the surface's reflection of that LED.
    UNVERIFIED: ring radius, sensor order and all levels."""

    count: int = 16
    ring_radius_cm: float = 8.0
    angle_offset_deg: float = 0.0
    dark: float = 150.0
    green: tuple[float, float, float] = (220.0, 600.0, 260.0)    # reflected R, G, B
    white: tuple[float, float, float] = (1700.0, 1800.0, 1650.0)
    line_width_cm: float = 2.0
    noise: float = 6.0
    adc_max: int = 4095
    dead_sensors: tuple[int, ...] = field(default_factory=tuple)


def line_points(robot_m, heading_sim: float, m: LineModel = LineModel()) -> np.ndarray:
    """Sim-frame positions (m) of the line sensors."""
    pts = np.empty((m.count, 2))
    for k in range(m.count):
        world = heading_sim - math.radians(m.angle_offset_deg + k * 360.0 / m.count)
        pts[k] = np.asarray(robot_m, float) + m.ring_radius_cm / 100.0 * np.array(
            [math.cos(world), math.sin(world)])
    return pts


def line_hits(robot_m, heading_sim: float, m: LineModel = LineModel()) -> list[bool]:
    """Which sensors are over the white line."""
    surf = surface_at(line_points(robot_m, heading_sim, m), m.line_width_cm / 100.0)
    return [bool(s == WHITE) for s in surf]


def line_adc(robot_m, heading_sim: float, m: LineModel = LineModel(),
             rng: np.random.Generator | None = None) -> np.ndarray:
    """uint16[4, 16]: ``[LineColor][sensor]`` as ``GPort::readLine`` delivers it."""
    hits = line_hits(robot_m, heading_sim, m)
    out = np.zeros((4, 16))
    for k in range(m.count):
        refl = m.white if hits[k] else m.green
        if k in m.dead_sensors:
            refl = (0.0, 0.0, 0.0)
        for c in range(3):
            out[c, k] = m.dark + refl[c]
        out[3, k] = m.dark
    out[:, : m.count] += _noise(rng, m.noise, (4, m.count))
    return np.clip(np.round(out), 0, m.adc_max).astype(np.uint16)


# ── Wheel encoders ────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class EncoderModel:
    """Quadrature encoders in timer encoder mode, counting the wheel's *physical* (kinematic)
    direction — what test_calibrate.cpp's encoder sign check expects, whatever the motor wiring.
    UNVERIFIED: ``ticks_per_rev`` = 72 makes the firmware's default maxTicksPerSec (1200) the
    no-load top speed."""

    ticks_per_rev: float = 72.0
    wheel_radius_m: float = 0.05
    polarity: tuple[float, float, float] = (1.0, 1.0, 1.0)


def encoder_rates(rim_speeds_mps, m: EncoderModel = EncoderModel()) -> np.ndarray:
    """Tick rate (ticks/s, signed) per wheel for the given rim speeds."""
    rps = np.asarray(rim_speeds_mps, dtype=float) / (2.0 * math.pi * m.wheel_radius_m)
    return rps * m.ticks_per_rev * np.asarray(m.polarity)
