"""Coordinate frames for lab modules.

The physics works in the **sim frame**: metres, origin at the centre spot, +x towards the goal
robot A attacks, +y to the left of that, heading 0 = facing +x, CCW positive.

Lab modules are written in the **user frame**, which matches the GeoGebra sketches and the
firmware: centimetres, robot-centred (the robot is at O = (0, 0)), field-aligned (it does not
rotate with the robot), +y towards the opponent goal, +x to the right when looking at it.

    user = (-y_sim, x_sim) * 100        sim = (y_user, -x_user) / 100

Headings are identical in both frames (0 = facing the opponent goal, CCW positive), so a
compass reading needs no conversion.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

CM_PER_M = 100.0


@dataclass(frozen=True, slots=True)
class Vec2:
    """Tiny immutable 2-D vector so formulas read like the GeoGebra script."""

    x: float
    y: float

    def __add__(self, o: Vec2) -> Vec2:
        return Vec2(self.x + o.x, self.y + o.y)

    def __sub__(self, o: Vec2) -> Vec2:
        return Vec2(self.x - o.x, self.y - o.y)

    def __mul__(self, k: float) -> Vec2:
        return Vec2(self.x * k, self.y * k)

    __rmul__ = __mul__

    def __truediv__(self, k: float) -> Vec2:
        return Vec2(self.x / k, self.y / k)

    def __neg__(self) -> Vec2:
        return Vec2(-self.x, -self.y)

    def __iter__(self):
        yield self.x
        yield self.y

    def norm(self) -> float:
        return math.hypot(self.x, self.y)

    def unit(self) -> Vec2:
        """Unit vector; the zero vector stays zero (callers handle the degenerate case)."""
        n = self.norm()
        return Vec2(0.0, 0.0) if n < 1e-12 else Vec2(self.x / n, self.y / n)

    def dot(self, o: Vec2) -> float:
        return self.x * o.x + self.y * o.y

    def cross(self, o: Vec2) -> float:
        return self.x * o.y - self.y * o.x

    def perp(self) -> Vec2:
        """Rotated +90° (CCW)."""
        return Vec2(-self.y, self.x)

    def angle(self) -> float:
        """Angle from +x, radians, CCW positive."""
        return math.atan2(self.y, self.x)

    def dist(self, o: Vec2) -> float:
        return math.hypot(self.x - o.x, self.y - o.y)


O = Vec2(0.0, 0.0)  # noqa: E741 — the robot, named as in the GeoGebra sketches


def world_to_user(v) -> Vec2:
    """Sim-frame vector (m) → user-frame vector (cm)."""
    return Vec2(-float(v[1]) * CM_PER_M, float(v[0]) * CM_PER_M)


def user_to_world(u: Vec2) -> np.ndarray:
    """User-frame vector (cm) → sim-frame vector (m)."""
    return np.array([u.y / CM_PER_M, -u.x / CM_PER_M])


def world_to_body(v, heading: float) -> np.ndarray:
    """Rotate a sim-frame vector into the robot body frame (+x = robot front)."""
    c, s = math.cos(heading), math.sin(heading)
    x, y = float(v[0]), float(v[1])
    return np.array([c * x + s * y, -s * x + c * y])


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi
