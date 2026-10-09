"""Field geometry queries shared by the sensor models: ray casts against the arena walls and
other robots, and what surface (green, white line) lies under a point."""
from __future__ import annotations

import math

import numpy as np

from bucky.game.field import ARENA_HALF_X, ARENA_HALF_Y, HALF_H, HALF_W

#: RCJ white line width (rules: 20 mm), drawn inside the playfield edge.
LINE_WIDTH = 0.02

GREEN, WHITE = 0, 1


def ray_to_arena(pos, direction, max_range: float = math.inf) -> float:
    """Distance from ``pos`` along unit ``direction`` to the arena wall box (m)."""
    ts = []
    for axis, half in ((0, ARENA_HALF_X), (1, ARENA_HALF_Y)):
        d = direction[axis]
        if abs(d) > 1e-9:
            bound = math.copysign(half, d)
            t = (bound - pos[axis]) / d
            if t > 0:
                ts.append(t)
    return min(ts) if ts else max_range


def ray_to_circle(pos, direction, center, radius: float) -> float:
    """Distance along unit ``direction`` from ``pos`` to the first hit on a circle (inf: miss)."""
    oc = np.asarray(pos, float) - np.asarray(center, float)
    b = float(np.dot(oc, direction))
    c = float(np.dot(oc, oc)) - radius * radius
    disc = b * b - c
    if disc < 0:
        return math.inf
    root = math.sqrt(disc)
    for t in (-b - root, -b + root):
        if t > 1e-9:
            return t
    return math.inf


def ray_cast(pos, direction, *, circles=(), max_range: float = math.inf) -> float:
    """Nearest hit of the arena walls and any ``(center, radius)`` obstacles."""
    t = ray_to_arena(pos, direction, max_range)
    for center, radius in circles:
        t = min(t, ray_to_circle(pos, direction, center, radius))
    return min(t, max_range)


def surface_at(points, line_width: float = LINE_WIDTH) -> np.ndarray:
    """Surface id under each point (N, 2) in the sim frame: :data:`WHITE` on the playfield's
    boundary line (``line_width`` wide, inside the playfield edge), else :data:`GREEN`."""
    pts = np.atleast_2d(np.asarray(points, dtype=float))
    ax, ay = np.abs(pts[:, 0]), np.abs(pts[:, 1])
    inside = (ax <= HALF_W) & (ay <= HALF_H)
    on_x = inside & (ax >= HALF_W - line_width)
    on_y = inside & (ay >= HALF_H - line_width)
    return np.where(on_x | on_y, WHITE, GREEN)
