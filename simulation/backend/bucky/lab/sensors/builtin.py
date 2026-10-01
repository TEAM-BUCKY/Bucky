"""Built-in sensor models."""
from __future__ import annotations

import math

import numpy as np

from bucky.lab.frames import Vec2, world_to_user, wrap_angle
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
    """Distance (cm) along N body-frame rays to the arena walls / other robot. Not implemented yet:
    fill in :meth:`read` with a ray-cast against ``bucky.game.field`` ARENA_HALF_X/Y."""

    name = "sonar"
    params = {
        "angles_deg": Param([0.0, 90.0, 180.0, 270.0], help="Ray directions, body frame"),
        "max_range_cm": Param(300.0, 10.0, 600.0, 10.0, "Readings beyond this return max"),
        "noise_cm": Param(1.0, 0.0, 20.0, 0.5, "Gaussian range noise (σ, cm)"),
    }

    def read(self, truth: TruthState, rng: np.random.Generator) -> list[float]:
        raise NotImplementedError("SonarSensor.read is a stub — implement the wall ray-cast first")


@register_sensor
class LineSensor(Sensor):
    """Which of N sensor points under the robot see a white line / are out of bounds. Not
    implemented yet: fill in :meth:`read` using ``bucky.game.field`` HALF_W/HALF_H."""

    name = "line"
    params = {
        "ring_radius_cm": Param(8.0, 1.0, 11.0, 0.5, "Radius of the sensor ring"),
        "count": Param(16, 4, 64, 1, "Number of sensor points on the ring"),
        "line_width_cm": Param(2.0, 0.5, 5.0, 0.5, "White line width"),
    }

    def read(self, truth: TruthState, rng: np.random.Generator) -> list[bool]:
        raise NotImplementedError("LineSensor.read is a stub — implement the line lookup first")
