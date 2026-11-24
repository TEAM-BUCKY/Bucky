"""Turn an observation vector back into a world picture for the scripted controllers.

Learned policies read the raw observation. The hand-written algorithms (FSM, APF, behaviour
tree) are easier to write and to explain in world coordinates: "where is the ball, where am I,
where is the goal". :func:`decode_obs` inverts :func:`bucky.obs.build_observation` exactly:

* ``obs[35:37]`` = R·(opp_goal − ball)/D and ``obs[37:39]`` = R·(own_goal − ball)/D,
  where R is the world→robot rotation and D the field diagonal.
* ``ball_rel`` (robot frame) comes from ``obs[0:3]`` (sin, cos bearing + distance).
* r1 = ball_rel + R·(opp_goal − ball) = R·(opp_goal − robot), and likewise r2 for the own goal.
  So r1 − r2 = R·(W, 0) = W·(cos h, −sin h), which gives the heading h.
* robot = opp_goal − Rᵀ·r1 and ball = opp_goal − Rᵀ·(obs[35:37]·D).

The opponent is not in the observation. In self-play (43-dim obs) we estimate it from the 4
sonar beams: a beam that reads clearly shorter than the wall distance expected from our own
pose is looking at the opponent (PWS §7.2 uses the same reasoning).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from bucky.game.field import FIELD_H, FIELD_W, HALF_W, OPP_GOAL, ROBOT_RADIUS
from bucky.game.geometry import ray_to_arena

FIELD_DIAG = float(math.hypot(FIELD_W, FIELD_H))
MAX_VEL = 5.236                      # obs velocity normaliser (physics MAX_LINEAR)
OWN_GOAL = -OPP_GOAL

SONAR_BEAMS = (0.0, math.pi / 2, -math.pi / 2, math.pi)    # forward, left, right, back
MAX_SONAR_RANGE = 1.5
SONAR_OPPONENT_MARGIN = 0.12         # m shorter than the expected wall → it's the opponent
SONAR_OBS_SLICE = slice(39, 43)


def rotation(heading: float) -> np.ndarray:
    """World → robot rotation matrix (same as bucky.obs)."""
    c, s = math.cos(heading), math.sin(heading)
    return np.array([[c, s], [-s, c]])


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class WorldView:
    """World-frame picture of one observation (robot attacks +x)."""

    robot_pos: np.ndarray
    heading: float
    ball_pos: np.ndarray
    ball_vel: np.ndarray
    robot_vel: np.ndarray
    kick_ready: bool
    ball_out: bool
    kick_predicts_goal: bool
    wall_proximity: float            # 0 far … 1 touching the nearest field edge
    opponent_pos: np.ndarray | None  # sonar estimate, None if unseen / single-agent obs

    # ── convenience geometry ────────────────────────────────────────────────
    @property
    def ball_rel(self) -> np.ndarray:
        return self.ball_pos - self.robot_pos

    @property
    def ball_dist(self) -> float:
        return float(np.linalg.norm(self.ball_rel))

    @property
    def heading_vec(self) -> np.ndarray:
        return np.array([math.cos(self.heading), math.sin(self.heading)])

    def to_body(self, world_vec: np.ndarray) -> np.ndarray:
        return rotation(self.heading) @ np.asarray(world_vec, dtype=float)

    def bearing_to(self, world_point: np.ndarray) -> float:
        """Angle (rad, CCW +) from our heading to ``world_point``."""
        d = np.asarray(world_point, dtype=float) - self.robot_pos
        return wrap(math.atan2(d[1], d[0]) - self.heading)


def decode_obs(obs) -> WorldView:
    o = np.asarray(obs, dtype=np.float64).reshape(-1)
    bearing = math.atan2(o[0], o[1])
    dist = float(o[2]) * FIELD_DIAG
    ball_rel_body = dist * np.array([math.cos(bearing), math.sin(bearing)])

    ball_to_opp_body = o[35:37] * FIELD_DIAG
    ball_to_own_body = o[37:39] * FIELD_DIAG
    r1 = ball_rel_body + ball_to_opp_body           # robot → opp goal (body frame)
    r2 = ball_rel_body + ball_to_own_body           # robot → own goal (body frame)
    d = r1 - r2                                     # = W·(cos h, −sin h)
    heading = math.atan2(-d[1], d[0])
    Rt = rotation(heading).T                        # robot → world

    robot_pos = OPP_GOAL - Rt @ r1
    ball_pos = OPP_GOAL - Rt @ ball_to_opp_body
    view = WorldView(
        robot_pos=robot_pos,
        heading=heading,
        ball_pos=ball_pos,
        ball_vel=Rt @ (o[3:5] * MAX_VEL),
        robot_vel=Rt @ (o[5:7] * MAX_VEL),
        kick_ready=bool(o[17] > 0.5),
        ball_out=bool(o[22] > 0.5),
        kick_predicts_goal=bool(o[23] > 0.5),
        wall_proximity=float(o[12]),
        opponent_pos=None,
    )
    if o.shape[0] >= SONAR_OBS_SLICE.stop:
        view.opponent_pos = opponent_from_sonar(view, o[SONAR_OBS_SLICE])
    return view


def opponent_from_sonar(view: WorldView, sonar: np.ndarray) -> np.ndarray | None:
    """Estimate the opponent position from the beam that is "shorter than the wall".

    If several beams qualify, the one with the largest shortfall wins (PWS §7.2).
    """
    best, best_gap = None, SONAR_OPPONENT_MARGIN
    for beam, reading in zip(SONAR_BEAMS, sonar):
        angle = view.heading + beam
        direction = np.array([math.cos(angle), math.sin(angle)])
        expected = ray_to_arena(view.robot_pos, direction, MAX_SONAR_RANGE)
        measured = float(reading) * MAX_SONAR_RANGE
        gap = expected - measured
        if gap > best_gap:
            best_gap = gap
            best = view.robot_pos + direction * (measured + ROBOT_RADIUS)
    return best


def in_own_half(point: np.ndarray) -> bool:
    return float(point[0]) < 0.0


def goal_mouth_point(y_offset: float = 0.0) -> np.ndarray:
    return np.array([HALF_W, y_offset])
