"""The IR ball and the IR ring's optics and analog front end, from the PCB and the ball's design.

Signal path (BUCKY_PCB IR_SENSOR + IR_CONTROLLER, RCJ IR golf ball):

* **Ball**: 42 × XL-2835IR-T202 at 100 mA, 30 mW/sr each, 120° (Lambertian), 940 nm. Bursts of 8
  carrier pulses (12 µs on / 12 µs off ≈ 41.7 kHz) every 833.33 µs. Peak power P = 42·π·I0 ≈
  3.96 W; as an isotropic source I = P / 4π ≈ 315 mW/sr.
* **Photodiode** VEMD2020X01: S = 12 µA at 1 mW/cm² (1.2e-6 A per W/m²) on-axis, ±15° half
  sensitivity → cosⁿθ with n ≈ 20.
* **TIA** 220k ‖ 10p, two **MFB band-pass** stages (f0 40.19 kHz, Q 1, gain −1), a **peak
  detector** on 10 nF / 1 MΩ, RESET dumping all holds each frame, 12-bit ADC at 3.3 V.

Python owns the slow part — geometry, which LEDs face which sensor, occlusion — and produces the
LED-on photocurrent of every sensor. The board's C++ :class:`IrFrontEnd` turns that into ADC codes
at the exact sample instants (burst timing, hold, droop, RESET, TIA clip, offset, noise).

Frames: sim frame (m, CCW, +x attacked goal) as in :mod:`bucky.firmware.frames`. Sensor k faces
k·``angle_step_deg`` *clockwise* from the robot front, like the firmware's IRRing.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Literal

import numpy as np

from bucky.game.field import BALL_RADIUS, ROBOT_RADIUS
from bucky.game.geometry import segments_hit_circles

CHANNELS = 16


# ── Electronics ───────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class IRFrontEnd:
    """One IR_SENSOR board's analog chain plus the controller's ADC path."""

    responsivity: float = 1.2e-6      # A per W/m², on-axis, lens included (VEMD2020X01)
    r_tia: float = 220e3
    c_tia: float = 10e-12
    bp_f0_hz: float = 40.19e3
    bp_q: float = 1.0
    bp_gain: float = 1.0
    bp_stages: int = 2
    kappa: float = 1.0                # harmonic correction to (2/π) (simulation says ≈ 1.01)
    v_sat: float = 3.5                # TL072 swing on ±5 V
    tau_hold_s: float = 10e-3         # R12 · C10
    reset_width_s: float = 0.7e-6     # R1 · C3 · ln 2
    envelope_order: int = 1
    offset_mean_v: float = 3e-3       # TL072 Vos (typ); UNVERIFIED sign
    offset_sigma_v: float = 1e-3
    noise_v: float = 10e-6            # rms at the hold output
    ambient_a: float = 0.0            # DC photocurrent from room light (eats TIA headroom)
    vref: float = 3.3
    adc_max: int = 4095

    def h_tia(self, f_hz: float) -> complex:
        return self.r_tia / (1 + 2j * math.pi * f_hz * self.r_tia * self.c_tia)

    def h_bp(self, f_hz: float) -> complex:
        x = 1j * f_hz / (self.bp_f0_hz * self.bp_q)
        return -self.bp_gain * x / (1 - (f_hz / self.bp_f0_hz) ** 2 + x)

    def chain_gain(self, f_hz: float) -> float:
        """Held peak volts per amp of LED-on photocurrent (50 % square wave at ``f_hz``)."""
        return (abs(self.h_tia(f_hz)) * abs(self.h_bp(f_hz)) ** self.bp_stages
                * (2.0 / math.pi) * self.kappa)

    def tau_bp(self) -> float:
        return 2.0 * self.bp_q / (2.0 * math.pi * self.bp_f0_hz)

    def i_sat(self) -> float:
        return self.v_sat / self.r_tia

    def volts_per_code(self) -> float:
        return self.vref / self.adc_max

    def code_of(self, i_on, f_hz: float) -> np.ndarray:
        """Steady-state held code (no droop, no noise) for LED-on photocurrents ``i_on``."""
        i = np.minimum(np.asarray(i_on, float), max(0.0, self.i_sat() - self.ambient_a))
        v = self.chain_gain(f_hz) * i + self.offset_mean_v
        return np.clip(np.round(v / self.volts_per_code()), 0, self.adc_max)

    def clip_code(self, f_hz: float) -> float:
        return (self.chain_gain(f_hz) * max(0.0, self.i_sat() - self.ambient_a)
                + self.offset_mean_v) / self.volts_per_code()


# ── Geometry and ball ─────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class IRRingGeometry:
    n_sensors: int = 12
    angle_step_deg: float = 30.0
    mount_radius_m: float = ROBOT_RADIUS     # TODO measure: photodiode distance from robot centre
    sensor_height_m: float = BALL_RADIUS     # TODO measure: photodiode height above the floor
    cos_power: float = 20.0                  # cosⁿθ fit to ±15° half sensitivity; UNVERIFIED shape
    own_body_radius_m: float = ROBOT_RADIUS - 1e-3

    def angles_cw(self) -> np.ndarray:
        return np.radians(self.angle_step_deg * np.arange(self.n_sensors))

    def sensors_world(self, robot_m, heading: float) -> tuple[np.ndarray, np.ndarray]:
        """3D positions (n, 3) and unit axes (n, 3) of the photodiodes, sim frame."""
        world = heading - self.angles_cw()          # CW in the body = CCW negative
        axes = np.stack([np.cos(world), np.sin(world), np.zeros_like(world)], axis=1)
        pos = np.empty_like(axes)
        pos[:, 0] = robot_m[0] + self.mount_radius_m * axes[:, 0]
        pos[:, 1] = robot_m[1] + self.mount_radius_m * axes[:, 1]
        pos[:, 2] = self.sensor_height_m
        return pos, axes


def fibonacci_sphere(n: int) -> np.ndarray:
    """``n`` unit vectors spread evenly over the sphere."""
    i = np.arange(n) + 0.5
    z = 1.0 - 2.0 * i / n
    r = np.sqrt(1.0 - z * z)
    phi = math.pi * (3.0 - math.sqrt(5.0)) * i
    return np.stack([r * np.cos(phi), r * np.sin(phi), z], axis=1)


@dataclass(frozen=True)
class IRBall:
    """RCJ IR golf ball (github.com/robocup-junior/ir-golf-ball)."""

    mode: Literal["isotropic", "leds"] = "isotropic"
    n_leds: int = 42
    led_i0_w_sr: float = 0.030       # XL-2835IR-T202 at 100 mA
    led_cos_power: float = 1.0       # Lambertian (120° full angle)
    radius_m: float = BALL_RADIUS
    center_height_m: float = BALL_RADIUS
    pulses: int = 8
    on_s: float = 12e-6
    off_s: float = 12e-6
    burst_period_s: float = 833.333e-6
    jitter_s: float = 5e-6
    period_ppm: float = 0.0

    @property
    def carrier_hz(self) -> float:
        return 1.0 / (self.on_s + self.off_s)

    @property
    def led_power_w(self) -> float:
        """Total radiant power of one LED: ∫ I0 cosᵐθ dΩ = 2π I0 / (m + 1)."""
        return 2.0 * math.pi * self.led_i0_w_sr / (self.led_cos_power + 1.0)

    @property
    def total_w(self) -> float:
        return self.n_leds * self.led_power_w

    @property
    def iso_w_sr(self) -> float:
        return self.total_w / (4.0 * math.pi)

    def led_dirs(self) -> np.ndarray:
        return fibonacci_sphere(self.n_leds)


class BallOrientation:
    """The ball's rotation (3×3, body → world), rolled without slipping on the floor."""

    def __init__(self, rot: np.ndarray | None = None) -> None:
        self.rot = np.eye(3) if rot is None else np.asarray(rot, float)
        self._steps = 0

    @staticmethod
    def random(rng: np.random.Generator) -> "BallOrientation":
        q, r = np.linalg.qr(rng.normal(size=(3, 3)))
        q = q @ np.diag(np.sign(np.diag(r)))
        if np.linalg.det(q) < 0:
            q[:, 0] = -q[:, 0]
        return BallOrientation(q)

    def advance(self, vel_mps, dt: float, radius: float = BALL_RADIUS) -> None:
        vx, vy = float(vel_mps[0]), float(vel_mps[1])
        speed = math.hypot(vx, vy)
        if speed <= 0.0 or dt <= 0.0:
            return
        axis = np.array([-vy / speed, vx / speed, 0.0])    # ẑ × v̂
        angle = speed * dt / radius
        k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        rot = np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)
        self.rot = rot @ self.rot
        self._steps += 1
        if self._steps % 64 == 0:
            u, _, vt = np.linalg.svd(self.rot)
            self.rot = u @ vt


# ── The whole ring ────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class IRRingModel:
    """Ball + ring optics + front end. ``noise`` is the ADC noise in LSB rms (only applied when
    the hardware has an rng); ``occlusion`` blocks rays through robots, including our own."""

    geometry: IRRingGeometry = field(default_factory=IRRingGeometry)
    front_end: IRFrontEnd = field(default_factory=IRFrontEnd)
    ball: IRBall = field(default_factory=IRBall)
    occlusion: bool = True
    noise: float = 1.0
    #: How often the sim re-evaluates the optics while the firmware runs (rolling ball, motion).
    update_s: float = 1e-3

    # Quantities the firmware and the lab tools need.
    @property
    def n_sensors(self) -> int:
        return self.geometry.n_sensors

    @property
    def baseline(self) -> float:
        """No-ball code (peak-detector offset)."""
        return self.front_end.offset_mean_v / self.front_end.volts_per_code()

    @property
    def gain(self) -> float:
        return self.front_end.chain_gain(self.ball.carrier_hz)

    @property
    def range_k(self) -> float:
        """K (cm·√code): N − baseline = (K / d_sensor)² on-axis — the firmware's kCm."""
        fe = self.front_end
        k_m = math.sqrt(fe.responsivity * self.ball.iso_w_sr * self.gain / fe.volts_per_code())
        return 100.0 * k_m

    @property
    def clip_code(self) -> float:
        return self.front_end.clip_code(self.ball.carrier_hz)

    def max_range_cm(self, min_amp: float = 8.0) -> float:
        """Centre distance where the facing sensor drops to ``min_amp`` codes (IRConfig.minAmp)."""
        return self.geometry.mount_radius_m * 100.0 + self.range_k / math.sqrt(min_amp)

    def firmware_config(self, fw, **overrides):
        """An ``IRConfig`` whose constants match this model."""
        cfg = fw.IRConfig()
        cfg.sensors = self.n_sensors
        cfg.baselineCode = float(self.baseline)
        cfg.kCm = float(self.range_k)
        cfg.mountRadiusCm = self.geometry.mount_radius_m * 100.0
        cfg.cosPower = self.geometry.cos_power
        for k, v in overrides.items():
            setattr(cfg, k, v)
        return cfg

    # Optics.
    def photocurrents(self, robot_m, heading: float, ball_m, rot: np.ndarray | None = None, *,
                      robots=(), visible: bool = True) -> np.ndarray:
        """LED-on photocurrent (A) of every channel (16; past ``n_sensors`` = 0)."""
        out = np.zeros(CHANNELS)
        if not visible or ball_m is None:
            return out
        g, b, fe = self.geometry, self.ball, self.front_end
        pos, axes = g.sensors_world(np.asarray(robot_m, float), heading)
        centre = np.array([float(ball_m[0]), float(ball_m[1]), b.center_height_m])
        blockers = [np.asarray(r, float)[:2] for r in robots]

        if b.mode == "isotropic":
            w = centre[None, :] - pos                                  # sensor → ball
            d2 = np.einsum("ij,ij->i", w, w)
            cos_s = np.einsum("ij,ij->i", axes, w) / np.sqrt(d2)
            e = b.iso_w_sr / d2 * np.maximum(cos_s, 0.0) ** g.cos_power
            if self.occlusion:
                e[self._blocked(pos[:, :2], np.broadcast_to(centre[:2], (len(pos), 2)),
                                robot_m, blockers)] = 0.0
        else:
            normals = b.led_dirs() @ (np.eye(3) if rot is None else rot).T          # (L, 3)
            leds = centre[None, :] + b.radius_m * normals
            u = pos[:, None, :] - leds[None, :, :]                                  # LED → sensor
            d2 = np.sum(u * u, axis=-1)
            uhat = u / np.sqrt(d2)[..., None]
            cos_led = np.einsum("lk,slk->sl", normals, uhat)
            cos_s = -np.einsum("sk,slk->sl", axes, uhat)
            e = (b.led_i0_w_sr * np.maximum(cos_led, 0.0) ** b.led_cos_power / d2
                 * np.maximum(cos_s, 0.0) ** g.cos_power)
            if self.occlusion and (blockers or g.own_body_radius_m > 0):
                n_s, n_l = e.shape
                p0 = np.repeat(pos[:, :2], n_l, axis=0)
                p1 = np.tile(leds[:, :2], (n_s, 1))
                e[self._blocked(p0, p1, robot_m, blockers).reshape(n_s, n_l)] = 0.0
            e = e.sum(axis=1)
        out[: g.n_sensors] = fe.responsivity * e
        return out

    def _blocked(self, p0, p1, robot_m, blockers) -> np.ndarray:
        hit = segments_hit_circles(p0, p1, [np.asarray(robot_m, float)[:2]],
                                   self.geometry.own_body_radius_m)
        if blockers:
            hit |= segments_hit_circles(p0, p1, blockers, ROBOT_RADIUS)
        return hit

    # Pushing to the simulated board.
    def push(self, board_ir, rng: np.random.Generator | None = None, seed: int = 0) -> None:
        """Configure a board's C++ front end (``bucky_fw.devices.gportN.ir``). Noise and offset
        spread only with an ``rng``; the burst-phase seed applies from the next reboot/reset."""
        fe, b = self.front_end, self.ball
        p = board_ir.params
        p.gain_v_per_a = self.gain
        p.i_sat_a = fe.i_sat()
        p.tau_bp_s = fe.tau_bp()
        p.envelope_order = fe.envelope_order
        p.tau_hold_s = fe.tau_hold_s
        p.reset_width_s = fe.reset_width_s
        p.vref = fe.vref
        p.noise_v = fe.noise_v if rng is not None else 0.0
        p.adc_noise_lsb = self.noise if rng is not None else 0.0
        board_ir.params = p
        bp = board_ir.burst
        bp.period_s = b.burst_period_s
        bp.period_ppm = b.period_ppm
        bp.jitter_s = b.jitter_s
        bp.pulses = b.pulses
        bp.on_s = b.on_s
        bp.off_s = b.off_s
        board_ir.burst = bp
        n = self.n_sensors
        offs = np.zeros(CHANNELS)
        offs[:n] = fe.offset_mean_v + (rng.normal(0.0, fe.offset_sigma_v, n) if rng is not None else 0.0)
        board_ir.offset_v = [float(v) for v in offs]
        amb = np.zeros(CHANNELS)
        amb[:n] = fe.ambient_a
        board_ir.i_ambient = [float(v) for v in amb]
        board_ir.seed(int(seed))


class IRScene:
    """Per-robot IR state that persists between updates: the ball's rolling orientation."""

    def __init__(self, model: IRRingModel, rng: np.random.Generator | None = None) -> None:
        self.model = model
        self.orientation = BallOrientation.random(rng) if rng is not None else BallOrientation()

    def update(self, robot_m, heading: float, ball_m, ball_vel=(0.0, 0.0), *, robots=(),
               visible: bool = True, dt: float = 0.0) -> np.ndarray:
        self.orientation.advance(ball_vel, dt, self.model.ball.radius_m)
        return self.model.photocurrents(robot_m, heading, ball_m, self.orientation.rot,
                                        robots=robots, visible=visible)


def with_ball(model: IRRingModel, **changes) -> IRRingModel:
    return replace(model, ball=replace(model.ball, **changes))
