"""Shared building blocks ("skills") for the scripted algorithms (FSM, APF, behaviour tree).

The skills are written once and reused by every algorithm (DRY, PWS §2.4). Each algorithm only
decides *which* skill to use and when.

Frames: every target is a world-frame point (robot attacks +x). An action is the robot-frame
``[vx, vy, ω, kick]`` in [-1, 1]. The omni-wheel mixing of PWS §4
(wheel_i = sin(d − θ_i)) happens below this layer, in physics and firmware.
"""
from __future__ import annotations

import math

import numpy as np

from bucky.controllers.perception import OWN_GOAL, WorldView, goal_mouth_point
from bucky.game.field import GOAL_HALF_WIDTH, HALF_H, HALF_W, OPP_GOAL

# PWS §5.2: C, the point behind the ball, at 10 cm. The first design cycle used 20 cm, which
# made the robot drive past the ball.
BEHIND_DIST = 0.10
KICK_RANGE = 0.16                    # physics: ball within this of the robot centre…
KICK_FRONT_COS = 0.906               # …and inside the ±25° front cone
KICK_AIM_TOL = math.radians(10.0)    # heading must point this close to the aim point to shoot
TURN_GAIN = 0.45                     # ω command per rad of heading error (≈ half the error per step)
DEFEND_RADIUS = 0.55                 # m from our goal centre (just outside the penalty area)
HAS_BALL_DIST = 0.15                 # m robot centre ↔ ball counts as "in possession"


def unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return np.asarray(v, dtype=float) / n if n > 1e-9 else np.zeros(2)


def clip_norm(v: np.ndarray, m: float = 1.0) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v * (m / n) if n > m else v


# ── where to shoot / where to stand ─────────────────────────────────────────
def aim_point(view: WorldView) -> np.ndarray:
    """Where in the goal to shoot. The centre, or the post farthest from the opponent when the
    sonar sees them standing near the goal line."""
    if view.opponent_pos is None or view.opponent_pos[0] < HALF_W - 0.45:
        return OPP_GOAL.copy()
    side = -1.0 if view.opponent_pos[1] > 0 else 1.0
    return goal_mouth_point(side * 0.6 * GOAL_HALF_WIDTH)


def approach_point(view: WorldView, behind: float = BEHIND_DIST, aim: np.ndarray | None = None) -> np.ndarray:
    """PWS §5: the bisector approach point A.

    B = ball, O = robot, C = the point ``behind`` m behind the ball on the line aim→ball.
    A lies on the bisector of ∠CBO at distance (|BC| + |BO|)/2 from B. If the robot re-aims at A
    every step, it swings around the ball and settles at C, behind the ball and facing the goal.
    This is a proportional-navigation-like pursuit of a point that sits in front of the target.
    """
    B, O = view.ball_pos, view.robot_pos
    g = unit((OPP_GOAL if aim is None else aim) - B)
    C = B - behind * g
    BO = O - B
    if np.linalg.norm(BO) < 1e-6:
        return C
    s = unit(C - B) + unit(BO)
    if np.linalg.norm(s) < 1e-6:                  # robot exactly in front: go around (left)
        s = np.array([-g[1], g[0]])
    r = (behind + float(np.linalg.norm(BO))) / 2.0
    return B + unit(s) * r


def defend_point(view: WorldView) -> np.ndarray:
    """On the line between the ball and our goal, ``DEFEND_RADIUS`` out from the goal centre."""
    d = view.ball_pos - OWN_GOAL
    dist = float(np.linalg.norm(d))
    radius = min(DEFEND_RADIUS, 0.7 * dist) if dist > 1e-6 else DEFEND_RADIUS
    p = OWN_GOAL + unit(d) * radius
    p[0] = max(p[0], -HALF_W + 0.15)
    return p


def retreat_point(view: WorldView) -> np.ndarray:
    """A safe spot while the ball is out of play: between our goal and the centre."""
    return np.array([-0.35, float(np.clip(view.ball_pos[1], -0.3, 0.3))])


# ── motion primitives ───────────────────────────────────────────────────────
def drive_to(view: WorldView, target: np.ndarray, gain: float = 3.0, min_speed: float = 0.15,
             max_speed: float = 1.0) -> np.ndarray:
    """Robot-frame (vx, vy) toward a world point: proportional and saturating."""
    d = np.asarray(target, dtype=float) - view.robot_pos
    dist = float(np.linalg.norm(d))
    if dist < 1e-4:
        return np.zeros(2)
    speed = min(max_speed, gain * dist + min_speed)
    return view.to_body(unit(d) * speed)


def body_vec(view: WorldView, world_vec: np.ndarray) -> np.ndarray:
    return clip_norm(view.to_body(world_vec), 1.0)


def turn_toward(view: WorldView, target: np.ndarray, gain: float = TURN_GAIN) -> float:
    return float(np.clip(gain * view.bearing_to(target), -1.0, 1.0))


def wall_push(view: WorldView, margin: float = 0.12) -> np.ndarray:
    """World-frame push back inside the white lines (0 when well inside)."""
    x, y = view.robot_pos
    push = np.zeros(2)
    for coord, lim, axis in ((x, HALF_W, 0), (y, HALF_H, 1)):
        over = abs(coord) - (lim - margin)
        if over > 0:
            push[axis] = -math.copysign(over / margin, coord)
    return clip_norm(push, 1.0)


# ── perception predicates (shared by FSM transitions and BT conditions) ─────
def has_ball(view: WorldView) -> bool:
    if view.ball_dist > HAS_BALL_DIST:
        return False
    return float(np.dot(view.heading_vec, unit(view.ball_rel))) > KICK_FRONT_COS


def behind_ball(view: WorldView, cone_deg: float = 35.0) -> bool:
    """Robot on the goal-opposite side of the ball, inside a cone around the aim line."""
    g = unit(aim_point(view) - view.ball_pos)
    return float(np.dot(unit(view.ball_rel), g)) > math.cos(math.radians(cone_deg))


def opponent_closer_to_ball(view: WorldView) -> bool:
    if view.opponent_pos is None:
        return False
    return float(np.linalg.norm(view.opponent_pos - view.ball_pos)) < view.ball_dist


def should_kick(view: WorldView) -> bool:
    """Fire only with a charged kicker and the ball in the kick cone, and either the kick
    predictor says it scores or the heading points at the aim point."""
    if not view.kick_ready or view.ball_dist > KICK_RANGE:
        return False
    if float(np.dot(view.heading_vec, unit(view.ball_rel))) < KICK_FRONT_COS:
        return False
    return view.kick_predicts_goal or abs(view.bearing_to(aim_point(view))) < KICK_AIM_TOL


def action(move_body: np.ndarray, omega: float, kick: bool) -> np.ndarray:
    m = clip_norm(np.asarray(move_body, dtype=float), 1.0)
    return np.array([m[0], m[1], float(np.clip(omega, -1, 1)), 1.0 if kick else 0.0], dtype=np.float32)


# ── composite behaviours (one "output" per FSM state / BT action) ───────────
def act_approach(view: WorldView) -> np.ndarray:
    """Drive to the bisector point A behind the ball, facing the ball."""
    target = approach_point(view, aim=aim_point(view))
    move = drive_to(view, target) + view.to_body(wall_push(view))
    return action(move, turn_toward(view, view.ball_pos), False)


def act_attack(view: WorldView) -> np.ndarray:
    """Dribble the ball toward the aim point, turn to aim, and kick when the shot is on."""
    aim = aim_point(view)
    push_dir = unit(view.ball_pos + 0.3 * unit(aim - view.ball_pos) - view.robot_pos)
    move = view.to_body(push_dir * 0.8) + view.to_body(wall_push(view))
    return action(move, turn_toward(view, aim), should_kick(view))


def act_defend(view: WorldView) -> np.ndarray:
    """Stand on the ball→own-goal line, facing the ball."""
    move = drive_to(view, defend_point(view), gain=4.0) + view.to_body(wall_push(view))
    return action(move, turn_toward(view, view.ball_pos), should_kick(view))


def act_recover(view: WorldView) -> np.ndarray:
    """Ball out of play or robot at the line: get back to a safe spot inside the field."""
    move = drive_to(view, retreat_point(view)) + view.to_body(2.0 * wall_push(view))
    return action(move, turn_toward(view, OPP_GOAL), False)
