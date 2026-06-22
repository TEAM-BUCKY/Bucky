"""Observation builder — single source of truth for the observation vector.

Layout (total 23 dims):
  [0:2]  ball_bearing (sin, cos)            — avoids angle wrap
  [2]    ball_distance (normalized)
  [3:5]  ball_vel (vx, vy) robot frame
  [5:7]  robot_vel (vx, vy) robot frame
  [7]    robot_omega (normalized)
  [8:10] heading_to_opp_goal (sin, cos)     — from BNO085 zero-at-kickoff
  [10:13] nearest_edge (sin, cos, proximity)
  [13]   over_goal_area_flag
  [14:17] teammate (rel_x, rel_y, has_ball) — zeroed in stage 1
  [17]   kick_ready (1.0 = kicker recharged, 0.0 = on cooldown)
  [18]   ball_line_dist  — ball's signed distance to its nearest white line (>0 inside)
  [19]   ball_vel_toward_line — ball speed component toward that line (>0 heading out)
  [20:22] respawn_rel (rel_x, rel_y) — robot-frame vector to where the ball would relocate
  [22]   ball_out_flag   — 1.0 while the ball is past the white line (in the grace window)
"""
from __future__ import annotations

import numpy as np

from bucky.game import field as _field
from bucky.game.field import FIELD_H as _FIELD_H
from bucky.game.field import FIELD_W as _FIELD_W
from bucky.game.field import PENALTY_DEPTH, PENALTY_HALF_WIDTH
from bucky.physics.backend import PhysicsState

_FIELD_DIAG = (_FIELD_W**2 + _FIELD_H**2) ** 0.5
# Normalizer for the observed angular velocity — kept equal to the drivetrain's real max turn
# rate (physics.MAX_OMEGA ≈ 58 rad/s) so obs[7] spans ~[-1, 1] instead of saturating.
_MAX_OMEGA = 58.18
# Velocity normalizer — equal to the drivetrain's real top speed (physics.MAX_LINEAR ≈ 5.24 m/s)
# so robot/ball velocity observations span ~[-1, 1] instead of clipping.
_MAX_VEL = 5.236

OBS_DIM: int = 23

# The last block of the observation — indices [18:23]: ball_line_dist, ball_vel_toward_line,
# respawn_rel (x, y), ball_out_flag — was appended later (ball-vs-boundary awareness). Policies
# trained before it saw an 18-dim base. Kept as a named constant so the self-play layer can
# project the current obs back to that older layout for backward-compatible match play.
N_BALL_BOUNDARY_FEATURES: int = 5
LEGACY_OBS_DIM: int = OBS_DIM - N_BALL_BOUNDARY_FEATURES   # 18 — base width before that block

_NOISE_BEARING_STD = np.deg2rad(3.0)
_NOISE_DIST_FRAC_STD = 0.15


def _rotation_matrix(heading: float) -> np.ndarray:
    c, s = np.cos(heading), np.sin(heading)
    return np.array([[c, s], [-s, c]])   # world→robot


def build_observation(
    state: PhysicsState,
    *,
    add_noise: bool = False,
    rng: np.random.Generator | None = None,
    heading_drift: float = 0.0,
    kick_ready: float = 1.0,
) -> np.ndarray:
    """Build an 18-dim float32 observation from a PhysicsState.

    ``kick_ready`` (1.0 recharged / 0.0 on cooldown) lets the policy time its kicks.
    """
    if rng is None:
        rng = np.random.default_rng()

    R = _rotation_matrix(state.robot_heading)

    # Ball in robot frame
    ball_rel = R @ (state.ball_pos - state.robot_pos)
    bearing = np.arctan2(ball_rel[1], ball_rel[0])
    dist = np.linalg.norm(ball_rel)

    if add_noise:
        bearing += rng.normal(0, _NOISE_BEARING_STD)
        dist *= (1.0 + rng.normal(0, _NOISE_DIST_FRAC_STD))
        dist = max(dist, 0.0)

    ball_vel_robot = R @ state.ball_vel
    own_vel_robot = R @ state.robot_vel

    # Heading to opponent goal
    opp_goal = np.array([_FIELD_W / 2, 0.0])
    heading_to_goal = np.arctan2(opp_goal[1] - state.robot_pos[1],
                                 opp_goal[0] - state.robot_pos[0])
    heading_to_goal -= (state.robot_heading + heading_drift)
    heading_to_goal = (heading_to_goal + np.pi) % (2 * np.pi) - np.pi

    # Nearest field edge
    dists = [
        _FIELD_W / 2 - state.robot_pos[0],
        _FIELD_H / 2 - state.robot_pos[1],
        state.robot_pos[0] + _FIELD_W / 2,
        state.robot_pos[1] + _FIELD_H / 2,
    ]
    edge_angles_world = [0.0, np.pi / 2, np.pi, -np.pi / 2]
    idx = int(np.argmin(dists))
    nearest_dist = dists[idx]
    edge_bearing = edge_angles_world[idx] - state.robot_heading
    edge_bearing = (edge_bearing + np.pi) % (2 * np.pi) - np.pi
    proximity = 1.0 - np.clip(nearest_dist / (_FIELD_W / 2), 0.0, 1.0)

    # Over (either) penalty area flag — used to learn the §4.7.2 "too long in the
    # goal area" defective-robot risk.
    in_goal_x = abs(state.robot_pos[0]) > _FIELD_W / 2 - PENALTY_DEPTH
    in_goal_y = abs(state.robot_pos[1]) < PENALTY_HALF_WIDTH
    over_goal = float(in_goal_x and in_goal_y)

    # Ball-vs-boundary awareness, so the memoryless policy can *anticipate* the ball going out
    # (and where it will respawn) instead of only reacting after the fact. Signed distance from
    # the ball to its nearest white line (>0 inside), the ball's velocity component toward that
    # line (>0 = heading out), a robot-frame vector to the spot the ball would be relocated to,
    # and a flag for the ball already being out. Derived from ground truth (no sensor noise) —
    # these are coarse spatial cues, not a sensor reading.
    bx, by = float(state.ball_pos[0]), float(state.ball_pos[1])
    line_dists = (_FIELD_W / 2 - bx, bx + _FIELD_W / 2,
                  _FIELD_H / 2 - by, by + _FIELD_H / 2)
    line_normals = ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0))
    near_idx = int(np.argmin(line_dists))
    ball_line_dist = line_dists[near_idx] / (_FIELD_H / 2)
    n = line_normals[near_idx]
    ball_vel_toward_line = float(state.ball_vel[0] * n[0] + state.ball_vel[1] * n[1]) / _MAX_VEL

    respawn = _field.nearest_neutral_spot(state.ball_pos)
    respawn_rel = (R @ (respawn - state.robot_pos)) / _FIELD_DIAG
    ball_out_flag = float(_field.ball_out_of_play(state.ball_pos))

    obs = np.array([
        np.sin(bearing),
        np.cos(bearing),
        dist / _FIELD_DIAG,
        ball_vel_robot[0] / _MAX_VEL,
        ball_vel_robot[1] / _MAX_VEL,
        own_vel_robot[0] / _MAX_VEL,
        own_vel_robot[1] / _MAX_VEL,
        state.robot_omega / _MAX_OMEGA,
        np.sin(heading_to_goal),
        np.cos(heading_to_goal),
        np.sin(edge_bearing),
        np.cos(edge_bearing),
        proximity,
        over_goal,
        0.0, 0.0, 0.0,   # teammate (stage-1 zeros)
        float(kick_ready),
        ball_line_dist,
        ball_vel_toward_line,
        respawn_rel[0],
        respawn_rel[1],
        ball_out_flag,
    ], dtype=np.float32)

    return obs
