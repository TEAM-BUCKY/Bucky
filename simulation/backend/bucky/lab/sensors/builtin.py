"""Built-in sensor models."""
from __future__ import annotations

import math

import numpy as np

from bucky.firmware.sensors import SonarModel, sonar_distances_cm
from bucky.game.geometry import WHITE, surface_at
from bucky.lab.frames import CM_PER_M, Vec2, world_to_user, wrap_angle
from bucky.lab.params import Param
from bucky.lab.sensors.base import Sensor, TruthState, register_sensor


@register_sensor
class BallSensor(Sensor):
    """Ball position relative to the robot, user frame (cm, field-aligned, +y = opponent goal).
    Returns ``None`` when the reading drops out."""

    name = "ball"
    params = {
        "noise_cm": Param(0.0, 0.0, 20.0, 0.5, "Gaussian position noise (σ, cm)"),
        "dropout": Param(0.0, 0.0, 1.0, 0.01, "Probability a reading is missing"),
    }

    def read(self, truth: TruthState, rng: np.random.Generator) -> Vec2 | None:
        if self.p.dropout > 0 and rng.random() < self.p.dropout:
            return None
        rel = world_to_user(truth.state.ball_pos - truth.state.robot_pos)
        if self.p.noise_cm > 0:
            rel = rel + Vec2(*rng.normal(0.0, self.p.noise_cm, 2))
        return rel


@register_sensor
class CompassSensor(Sensor):
    """Robot heading in radians: 0 = facing the opponent goal, CCW positive, wrapped to [-π, π)."""

    name = "compass"
    params = {
        "noise_deg": Param(0.0, 0.0, 30.0, 0.5, "Gaussian noise per reading (σ, degrees)"),
        "bias_deg": Param(0.0, 0.0, 45.0, 0.5, "Max constant bias, drawn uniformly per episode"),
    }

    def reset(self, rng: np.random.Generator) -> None:
        b = math.radians(self.p.bias_deg)
        self._bias = float(rng.uniform(-b, b)) if b > 0 else 0.0

    def read(self, truth: TruthState, rng: np.random.Generator) -> float:
        h = truth.state.robot_heading + getattr(self, "_bias", 0.0)
        if self.p.noise_deg > 0:
            h += float(rng.normal(0.0, math.radians(self.p.noise_deg)))
        return wrap_angle(h)


@register_sensor
class SonarSensor(Sensor):
    """Distance (cm) along N body-frame rays, from the robot's rim to the arena walls or the other
    robot. Angles are counter-clockwise from the robot front. Readings beyond ``max_range_cm``
    (or dropped) return ``max_range_cm``."""

    name = "sonar"
    params = {
        "angles_deg": Param([0.0, 90.0, 180.0, 270.0], help="Ray directions, body frame (CCW)"),
        "max_range_cm": Param(300.0, 10.0, 600.0, 10.0, "Readings beyond this return max"),
        "noise_cm": Param(1.0, 0.0, 20.0, 0.5, "Gaussian range noise (σ, cm)"),
        "dropout": Param(0.0, 0.0, 1.0, 0.01, "Probability a reading is lost (returns max)"),
    }

    def read(self, truth: TruthState, rng: np.random.Generator) -> list[float]:
        model = SonarModel(angles_cw_deg=tuple(-float(a) for a in self.p.angles_deg),
                           max_range_cm=self.p.max_range_cm, noise_cm=self.p.noise_cm,
                           dropout=self.p.dropout)
        robots = () if truth.other_robot_pos is None else (truth.other_robot_pos,)
        d = sonar_distances_cm(truth.state.robot_pos, truth.state.robot_heading, model,
                               robots=robots, rng=rng)
        return [float(v) if math.isfinite(v) else self.p.max_range_cm for v in d]


@register_sensor
class LineSensor(Sensor):
    """Which of N sensor points on a ring under the robot are over the white boundary line.
    Point k sits k·360/N degrees counter-clockwise from the robot front."""

    name = "line"
    params = {
        "ring_radius_cm": Param(8.0, 1.0, 11.0, 0.5, "Radius of the sensor ring"),
        "count": Param(16, 4, 64, 1, "Number of sensor points on the ring"),
        "line_width_cm": Param(2.0, 0.5, 5.0, 0.5, "White line width"),
    }

    def read(self, truth: TruthState, rng: np.random.Generator) -> list[bool]:
        pos = np.asarray(truth.state.robot_pos, dtype=float)
        h = float(truth.state.robot_heading)
        r = self.p.ring_radius_cm / CM_PER_M
        ang = h + 2 * math.pi * np.arange(self.p.count) / self.p.count
        pts = pos + r * np.column_stack([np.cos(ang), np.sin(ang)])
        return [bool(v == WHITE) for v in surface_at(pts, self.p.line_width_cm / CM_PER_M)]
