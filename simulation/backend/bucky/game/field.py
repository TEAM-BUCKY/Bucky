"""Official RoboCup Junior Soccer 1:1 (NK / RoboCup Netherlands) field geometry.

Single source of truth for every field/goal/robot/ball dimension. All other modules
(physics, observation, rewards, referee, match) import from here so the simulation and
training share one — correct — pitch.

Coordinate convention (meters), matching the existing backend:
  * **X = goal-to-goal (long) axis** — goals sit on the ±X walls.
  * **Y = lateral axis.**
  * Origin at the centre spot.

Official dimensions (rules §1):
  * Total walled field: 2430 × 1820 mm.
  * Playfield (white lines):  1830 × 1220 mm   → a uniform 300 mm band to the wall.
  * Goal: 450 mm wide.
  * Penalty area (strafschopgebied): 450 mm deep from the goal line, goal-width wide.
  * Robot fits a 220 mm cylinder → 110 mm radius.
  * Ball (RCJ-05): modelled at 42 mm diameter (21 mm radius).
"""
from __future__ import annotations

import numpy as np

# ── Playfield (white-line) extents ──────────────────────────────────────────
FIELD_W = 1.83          # goal-to-goal (X). half = 0.915 → goal lines at x = ±0.915
FIELD_H = 1.22          # lateral (Y).      half = 0.61
HALF_W = FIELD_W / 2    # 0.915
HALF_H = FIELD_H / 2    # 0.61

GOAL_WIDTH = 0.45       # goal opening on Y
GOAL_HALF_WIDTH = GOAL_WIDTH / 2  # 0.225

# ── Walled arena (playfield + uniform 300 mm neutral band) ──────────────────
WALL_BAND = 0.30
ARENA_HALF_X = HALF_W + WALL_BAND   # 1.215
ARENA_HALF_Y = HALF_H + WALL_BAND   # 0.91

# ── Robot & ball ────────────────────────────────────────────────────────────
ROBOT_RADIUS = 0.11     # 220 mm diameter
BALL_RADIUS = 0.021     # 42 mm diameter (decision: keep)
COLLISION_DIST = ROBOT_RADIUS + BALL_RADIUS

# ── Penalty area (one per goal) ─────────────────────────────────────────────
PENALTY_DEPTH = 0.45
PENALTY_HALF_WIDTH = GOAL_HALF_WIDTH  # 0.225 — same width as the goal opening

# Opponent goal centre (robot A attacks +x).
OPP_GOAL = np.array([HALF_W, 0.0])

# ── Neutral spots (rules §1.4) ──────────────────────────────────────────────
# Five neutral zones: the inner (field-side) corner of each of the two penalty
# areas, plus the centre spot. Used to re-place the ball (ball-out / out-of-reach /
# lack-of-progress) and to re-enter suspended/defective robots.
_PEN_INNER_X = HALF_W - PENALTY_DEPTH   # 0.465
NEUTRAL_SPOTS = np.array([
    [0.0, 0.0],
    [_PEN_INNER_X, PENALTY_HALF_WIDTH],
    [_PEN_INNER_X, -PENALTY_HALF_WIDTH],
    [-_PEN_INNER_X, PENALTY_HALF_WIDTH],
    [-_PEN_INNER_X, -PENALTY_HALF_WIDTH],
])
CENTER_SPOT = NEUTRAL_SPOTS[0]


def in_goal_mouth(ball_pos) -> bool:
    """True if the ball's lateral position lies within the goal opening."""
    return abs(float(ball_pos[1])) < GOAL_HALF_WIDTH


def is_goal(ball_pos) -> tuple[bool, bool]:
    """(goal_for_plus_x, goal_for_minus_x): ball wholly over a goal line, in the mouth."""
    x = float(ball_pos[0])
    mouth = in_goal_mouth(ball_pos)
    return (x > HALF_W and mouth, x < -HALF_W and mouth)


def ball_out_of_play(ball_pos) -> bool:
    """True if the ball has crossed the white line (and is not in a goal mouth)."""
    x, y = float(ball_pos[0]), float(ball_pos[1])
    past = abs(x) > HALF_W or abs(y) > HALF_H
    return past and not in_goal_mouth(ball_pos)


def robot_fully_out(robot_pos) -> bool:
    """True once the whole robot body has crossed the white line."""
    return (abs(float(robot_pos[0])) > HALF_W + ROBOT_RADIUS or
            abs(float(robot_pos[1])) > HALF_H + ROBOT_RADIUS)


def robot_in_goal(pos) -> bool:
    """True if the robot centre is inside *either* goal box (behind a goal line, within the mouth).

    Entering the mouth is physically possible (it is an opening) but is a violation — heavily
    penalised in the reward. The goal's side/back walls are solid for the robot (see physics
    ``_resolve_robot_goal``), so it can never drive *through* the goal into the rear band.
    Covers both goals: attacking into the opponent goal *and* backing into one's own goal are bad.
    """
    x, y = float(pos[0]), float(pos[1])
    return abs(x) > HALF_W and abs(y) < GOAL_HALF_WIDTH


def in_penalty_area(pos, goal_sign: int) -> bool:
    """True if ``pos`` is inside the penalty area in front of the ``goal_sign`` goal.

    ``goal_sign`` is +1 for the +x goal, -1 for the -x goal.
    """
    x, y = float(pos[0]), float(pos[1])
    if goal_sign > 0:
        in_x = x > HALF_W - PENALTY_DEPTH
    else:
        in_x = x < -(HALF_W - PENALTY_DEPTH)
    return in_x and abs(y) <= PENALTY_HALF_WIDTH


def _occupied(spot, occupied, clearance: float) -> bool:
    for o in occupied:
        if o is None:
            continue
        if float(np.linalg.norm(spot - np.asarray(o, dtype=float))) < clearance:
            return True
    return False


def nearest_neutral_spot(pos, occupied=(), clearance: float = 0.0):
    """Nearest neutral spot to ``pos`` that is not within ``clearance`` of any
    ``occupied`` position. Falls back to the plain nearest spot if all are occupied.
    """
    pos = np.asarray(pos, dtype=float)
    order = sorted(range(len(NEUTRAL_SPOTS)),
                   key=lambda i: float(np.linalg.norm(NEUTRAL_SPOTS[i] - pos)))
    for i in order:
        if not _occupied(NEUTRAL_SPOTS[i], occupied, clearance):
            return NEUTRAL_SPOTS[i].copy()
    return NEUTRAL_SPOTS[order[0]].copy()


def nearest_neutral_to_goal(defended_goal_sign: int, ball_pos=None,
                            occupied=(), clearance: float = 0.0):
    """Re-entry spot for a robot: the neutral spot nearest the goal it defends, but
    not the one directly opposite the ball (so the robot isn't unfairly advantaged —
    rules §4.7.5 / §4.9.3). Excludes the centre spot.
    """
    goal = np.array([defended_goal_sign * HALF_W, 0.0])
    candidates = [s for s in NEUTRAL_SPOTS
                  if np.sign(s[0]) == np.sign(defended_goal_sign) and s[0] != 0.0]
    candidates.sort(key=lambda s: float(np.linalg.norm(s - goal)))

    def opposite_ball(spot) -> bool:
        if ball_pos is None:
            return False
        # "Directly opposite the ball" ≈ on the same lateral side as the ball.
        return np.sign(spot[1]) == np.sign(float(ball_pos[1])) and abs(float(ball_pos[1])) > 0.05

    for s in candidates:
        if not opposite_ball(s) and not _occupied(s, occupied, clearance):
            return s.copy()
    for s in candidates:
        if not _occupied(s, occupied, clearance):
            return s.copy()
    return candidates[0].copy() if candidates else CENTER_SPOT.copy()