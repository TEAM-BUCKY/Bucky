"""Predicted kick-outcome features for the observation.

A memoryless policy can see the ball's position and velocity but not what a *kick* would do.
This module ray-casts the ball forward from its current position along the robot heading — the
direction a kick fires ("if the kicker was shot now") — bouncing off the arena walls, and reports
the first two things it would hit. Each contact is classified as one of {goal, robot, wall} and
located, giving the policy a cheap forward model with which to aim shots.

Geometry mirrors :func:`bucky.physics.python_backend.TwoRobotPhysics._resolve_ball_walls`:
  * reflecting walls are the **arena** walls at ±(ARENA_HALF_* − BALL_RADIUS);
  * a **goal** is the ray crossing a goal line ``x = ±HALF_W`` within the mouth (|y| < GOAL_HALF_WIDTH);
  * a **robot** is the opponent, modelled as a circle of radius ``COLLISION_DIST`` at ``opponent_pos``.

The cast is purely geometric: ball restitution scales speed, not direction, so it never changes
*which* object is hit next. The thin goal-side-wall band is omitted (negligible as an obs cue).

The output is a flat 12-float block: for the 1st then the 2nd contact,
``[is_goal, is_robot, is_wall, px, py, dist]`` where ``px, py`` are the impact point in the robot
frame and ``dist`` is the path length to the contact, both normalized by the field diagonal (same
convention as ``respawn_rel`` in :mod:`bucky.obs`). A terminal first contact (goal/robot) leaves
the second slot zero.
"""
from __future__ import annotations

import numpy as np

from bucky.game.field import (
    ARENA_HALF_X,
    ARENA_HALF_Y,
    BALL_RADIUS,
    COLLISION_DIST,
    GOAL_HALF_WIDTH,
    HALF_W,
)

N_KICK_PRED_FEATURES = 12   # 2 contacts × [is_goal, is_robot, is_wall, px, py, dist]

# Reflecting arena walls, inset by the ball radius (matches the physics wall resolution).
_WALL_HX = ARENA_HALF_X - BALL_RADIUS
_WALL_HY = ARENA_HALF_Y - BALL_RADIUS
_EPS = 1e-9


def _ray_circle(pos: np.ndarray, d: np.ndarray, center: np.ndarray, radius: float) -> float | None:
    """Nearest positive distance along unit ray ``(pos, d)`` to the circle, or None."""
    oc = pos - center
    b = float(np.dot(oc, d))
    c = float(np.dot(oc, oc)) - radius * radius
    disc = b * b - c
    if disc < 0.0:
        return None
    sq = float(np.sqrt(disc))
    t1, t2 = -b - sq, -b + sq
    if t1 > _EPS:
        return t1
    if t2 > _EPS:
        return t2
    return None


def _first_contact(pos: np.ndarray, d: np.ndarray, opponent_pos: np.ndarray | None):
    """Nearest contact along unit ray ``(pos, d)``.

    Returns ``(t, kind, point, axis)`` where ``kind`` is 'goal' | 'robot' | 'wall', ``point`` is
    the world impact point and ``axis`` is the reflecting axis for a wall (0=x, 1=y), else -1.
    Returns ``(None, None, None, -1)`` if nothing is hit (should not happen inside the arena).
    """
    best_t = np.inf
    best_kind: str | None = None
    best_axis = -1

    # Goal lines: x = ±HALF_W, counted only when crossed inside the mouth.
    if abs(d[0]) > _EPS:
        for sign in (1.0, -1.0):
            line = sign * HALF_W
            t = (line - pos[0]) / d[0]
            if _EPS < t < best_t:
                y = pos[1] + t * d[1]
                if abs(y) < GOAL_HALF_WIDTH:
                    best_t, best_kind, best_axis = t, "goal", -1

    # Arena walls: nearest plane whose other coordinate is still within the box.
    for axis, half in ((0, _WALL_HX), (1, _WALL_HY)):
        if abs(d[axis]) <= _EPS:
            continue
        other = 1 - axis
        other_half = _WALL_HY if other == 1 else _WALL_HX
        for bound in (half, -half):
            t = (bound - pos[axis]) / d[axis]
            if _EPS < t < best_t:
                if abs(pos[other] + t * d[other]) <= other_half + 1e-6:
                    best_t, best_kind, best_axis = t, "wall", axis

    # Opponent robot (circle).
    if opponent_pos is not None:
        t = _ray_circle(pos, d, np.asarray(opponent_pos, dtype=float), COLLISION_DIST)
        if t is not None and _EPS < t < best_t:
            best_t, best_kind, best_axis = t, "robot", -1

    if best_kind is None:
        return None, None, None, -1
    return best_t, best_kind, pos + best_t * d, best_axis


_KIND_SLOT = {"goal": 0, "robot": 1, "wall": 2}


def predict_kick_outcome(
    ball_pos: np.ndarray,
    robot_heading: float,
    R: np.ndarray,
    robot_pos: np.ndarray,
    opponent_pos: np.ndarray | None = None,
    *,
    field_diag: float,
) -> np.ndarray:
    """Predict the first two contacts of a kick fired from ``ball_pos`` along ``robot_heading``.

    ``R`` is the world→robot rotation matrix; ``field_diag`` normalizes positions/distances.
    Returns a 12-float block (see module docstring).
    """
    out = np.zeros(N_KICK_PRED_FEATURES, dtype=np.float32)
    pos = np.asarray(ball_pos, dtype=float).copy()
    d = np.array([np.cos(robot_heading), np.sin(robot_heading)], dtype=float)
    robot_pos = np.asarray(robot_pos, dtype=float)
    total_dist = 0.0

    for i in range(2):
        t, kind, point, axis = _first_contact(pos, d, opponent_pos)
        if kind is None:
            break
        total_dist += t
        base = i * 6
        out[base + _KIND_SLOT[kind]] = 1.0
        rel = R @ (point - robot_pos)
        out[base + 3] = rel[0] / field_diag
        out[base + 4] = rel[1] / field_diag
        out[base + 5] = total_dist / field_diag
        if kind != "wall":
            break                       # goal/robot is terminal — leave the 2nd slot zero
        d = d.copy()
        d[axis] = -d[axis]              # reflect off the wall
        pos = point + d * 1e-6         # nudge off the surface so we don't re-hit it
    return out
