"""A parametric reactive striker: ball-orbit + artificial-potential-field + kick-timing.

This is the report's #1 (classical) approach, rendered as a pure function of the observation
vector so it satisfies the shared ``predict`` contract. It reads the egocentric obs (layout in
:mod:`bucky.obs`), computes a body-frame drive vector that approaches the ball from *behind*
(so a push/kick sends it toward the enemy goal), curves around the ball when on the wrong side
(the canonical RoboCup-Junior "orbit"), steers away from walls, and fires the kicker when a shot
would score (using the observation's built-in kick predictor).

Every knob lives in :class:`ReactiveParams`, which vectorizes to/from a flat array so CMA-ES / a
GA can tune the controller directly (Stage 7). Defaults are hand-tuned, not optimal — that is the
point: the optimizer improves them.

Observation indices used (see :mod:`bucky.obs`):
  [0:2] ball bearing (sin, cos), [2] ball distance / FIELD_DIAG, [10:12] nearest-edge bearing
  (sin, cos), [12] edge proximity, [17] kick_ready, [23] predicted-shot-is-goal,
  [35:37] ball→enemy-goal vector (robot frame, FIELD_DIAG-normalized).
"""
from __future__ import annotations

import math
from dataclasses import asdict, astuple, dataclass, fields

import numpy as np

from bucky.game.field import FIELD_H, FIELD_W

_FIELD_DIAG = math.hypot(FIELD_W, FIELD_H)


@dataclass
class ReactiveParams:
    """Tunable parameters for :class:`ReactiveController` (all distances in metres)."""

    approach_offset: float = 0.18     # how far behind the ball to aim
    approach_gain: float = 3.0        # P-gain of the drive toward the aim point
    min_speed: float = 0.15           # floor so the robot keeps moving near the aim point
    orbit_radius: float = 0.40        # start orbiting when this close to the ball on the wrong side
    orbit_gain: float = 0.9           # strength of the tangential swing around the ball
    behind_cone_deg: float = 55.0     # "behind the ball" if ball & goal are within this cone ahead
    aim_goal_dist: float = 0.55       # closer than this: face goal (to shoot); else face ball
    turn_gain: float = 2.0            # P-gain of the heading controller
    wall_proximity: float = 0.85      # steer away from a wall above this proximity (1 = touching)
    wall_avoid_gain: float = 0.7      # strength of wall avoidance
    kick_distance: float = 0.22       # only consider kicking within this ball distance
    kick_align_deg: float = 22.0      # fallback: kick if ball→goal is within this cone of heading

    def to_vector(self) -> np.ndarray:
        return np.array(astuple(self), dtype=np.float64)

    @classmethod
    def from_vector(cls, vec) -> "ReactiveParams":
        names = [f.name for f in fields(cls)]
        return cls(**{n: float(v) for n, v in zip(names, np.asarray(vec, dtype=float))})

    def to_dict(self) -> dict:
        return asdict(self)


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else np.zeros_like(v)


def _clip_norm(v: np.ndarray, m: float = 1.0) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v * (m / n) if n > m else v


class ReactiveController:
    """Deterministic striker policy; a drop-in for a learned model (``predict`` + ``observation_space``)."""  # noqa: E501

    def __init__(self, params: ReactiveParams | None = None, obs_dim: int = 43) -> None:
        self.params = params or ReactiveParams()
        # Advertised so CompatPolicy-style callers and the tournament treat it like a native-width
        # policy; it reads only low-index fields shared by every obs width, so any width works.
        from gymnasium import spaces
        self.observation_space = spaces.Box(-3.0, 3.0, (obs_dim,), np.float32)

    def predict(self, obs, deterministic: bool = True, **_):
        o = np.asarray(obs, dtype=np.float64).reshape(-1)
        p = self.params

        bearing = math.atan2(o[0], o[1])
        ball_dist = float(o[2]) * _FIELD_DIAG
        ball_rel = ball_dist * np.array([math.cos(bearing), math.sin(bearing)])
        ball_to_goal = o[35:37] * _FIELD_DIAG
        g = _unit(ball_to_goal)
        to_ball = _unit(ball_rel)

        # Drive toward the point just behind the ball (opposite the goal), scaled by distance.
        aim_point = ball_rel - p.approach_offset * g
        dist_aim = float(np.linalg.norm(aim_point))
        move = _unit(aim_point) * min(1.0, p.approach_gain * dist_aim + p.min_speed)

        # Orbit: close to the ball but not yet behind it → add a tangential swing so we curve
        # around instead of shoving the ball away from the goal.
        behind_ball = float(np.dot(to_ball, g)) > math.cos(math.radians(p.behind_cone_deg))
        if ball_dist < p.orbit_radius and not behind_ball:
            perp = np.array([-g[1], g[0]])
            side = 1.0 if float(np.dot(perp, ball_rel)) >= 0 else -1.0
            move = _clip_norm(move + side * perp * p.orbit_gain, 1.0)

        # Heading: face the goal to shoot when close, else keep the ball ahead while chasing.
        aim = g if ball_dist < p.aim_goal_dist else to_ball
        omega = float(np.clip(p.turn_gain * math.atan2(aim[1], aim[0]) / math.pi, -1.0, 1.0))

        # Wall avoidance: push away from a nearby wall.
        if float(o[12]) > p.wall_proximity:
            edge_bearing = math.atan2(o[10], o[11])
            away = -np.array([math.cos(edge_bearing), math.sin(edge_bearing)])
            move = _clip_norm(move + p.wall_avoid_gain * away, 1.0)

        # Kick when the shot is predicted to score, or (fallback) we're close and aimed at goal.
        aligned = float(np.dot(to_ball, g)) > math.cos(math.radians(p.kick_align_deg))
        kick_ready = float(o[17]) > 0.5
        predicted_goal = float(o[23]) > 0.5
        kick = 1.0 if (kick_ready and ball_dist < p.kick_distance
                       and (predicted_goal or aligned)) else 0.0

        move = _clip_norm(move, 1.0)
        return np.array([move[0], move[1], omega, kick], dtype=np.float32), None
