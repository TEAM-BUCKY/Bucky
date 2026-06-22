"""Opponent-aware play-event detection for 1v1 self-play.

:func:`bucky.rewards.compute_rewards` is stateless and only sees one robot (a
:class:`PhysicsState` carries a single robot + the ball), so the skilled-play events that
depend on the opponent *and* on history — stealing the ball, blocking a shot, threading a
kick past a defender, losing a kicked ball to the enemy, scoring off a kick or a wall — are
detected here instead. :class:`PlayEventTracker` is owned by the self-play env, ticked once
per step, and emits the boolean/scalar flags that ``compute_rewards`` multiplies by weights.

All detection is from robot A's frame (A attacks the +x goal, defends −x). Thresholds are
deliberately simple and tunable; they shape behaviour, they are not physics.
"""
from __future__ import annotations

import numpy as np

from bucky.game.field import COLLISION_DIST, GOAL_HALF_WIDTH, HALF_W, ball_out_of_play
from bucky.physics.backend import PhysicsState
from bucky.rewards import CAPTURE_RADIUS

# ── tunable thresholds ───────────────────────────────────────────────────────
SHOT_SPEED = 0.6          # m/s toward our goal to count as an incoming shot
KICK_LOST_WINDOW = 40     # steps (~0.8 s) after A kicks within which an enemy capture is "kick lost"
KICK_GOAL_WINDOW = 60     # steps (~1.2 s) after A kicks within which a goal counts as "kicked"
RISKY_RADIUS = 0.30       # m: opponent within this of the shot line (but clearing it) → "risky" thread
BLOCK_RADIUS = COLLISION_DIST   # m: kick line passing within this of the opponent will *hit* them
KICK_AT_OPP_RANGE = 0.8   # m: only count a strike on the opponent within this forward distance
CONTACT_DIST = COLLISION_DIST + 0.02   # ball "in contact" with a robot (blocks / dribble check)


def _owns(state: PhysicsState) -> tuple[bool, float]:
    """(robot controls the ball, distance robot→ball). Control = ball within CAPTURE_RADIUS
    and in front of the robot (same rule the possession reward uses)."""
    delta = state.ball_pos - state.robot_pos
    dist = float(np.linalg.norm(delta))
    if dist >= CAPTURE_RADIUS or dist < 1e-9:
        return False, dist
    heading_vec = np.array([np.cos(state.robot_heading), np.sin(state.robot_heading)])
    return bool(np.dot(heading_vec, delta) > 0.0), dist


def _incoming_shot_on_own_goal(ball_pos, ball_vel) -> bool:
    """True if the ball is travelling toward A's own goal (−x) fast and on a scoring line."""
    if ball_vel[0] >= -SHOT_SPEED:
        return False
    s = (-HALF_W - ball_pos[0]) / ball_vel[0]   # time to reach the goal line
    if s <= 0:
        return False
    y_at = ball_pos[1] + ball_vel[1] * s
    return abs(y_at) < GOAL_HALF_WIDTH


class PlayEventTracker:
    """Stateful per-episode detector. Call :meth:`reset` on env reset and :meth:`update`
    once per step; merge the returned dict into the reward ``info``."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._last_owner: str | None = None
        self._kick_lost_timer = 0      # >0 while a recent A kick can still be "lost"
        self._kick_goal_timer = 0      # >0 while a recent A kick can still produce a "kicked goal"
        self._bounce_since_kick = False
        self._shot_live = False
        self._shot_credited = False
        self._shot_in_flight = False     # a kicked ball still travelling free
        self._shot_departed = False      # …and it has since left A's contact

    def update(self, state_a: PhysicsState, state_b: PhysicsState, phys_info: dict) -> dict:
        ball_pos = state_a.ball_pos
        ball_vel = state_a.ball_vel
        a_owns, a_dist = _owns(state_a)
        b_owns, b_dist = _owns(state_b)
        goal_a = bool(phys_info.get("goal_a", False))
        goal_b = bool(phys_info.get("goal_b", False))

        # ── possession owner (sticky when nobody clearly controls the ball) ──
        if a_owns and b_owns:
            owner = "a" if a_dist <= b_dist else "b"
        elif a_owns:
            owner = "a"
        elif b_owns:
            owner = "b"
        else:
            owner = self._last_owner

        out: dict = {}

        # ── steal: ownership flips B → A, weighted by how far up the pitch ──
        if owner == "a" and self._last_owner == "b":
            out["stole_ball"] = True
            out["steal_gradient"] = float(np.clip(0.5 + 0.5 * ball_pos[0] / HALF_W, 0.0, 1.0))
        self._last_owner = owner

        # ── kick bookkeeping ──
        if phys_info.get("kicked_a", False):
            self._kick_lost_timer = KICK_LOST_WINDOW
            self._kick_goal_timer = KICK_GOAL_WINDOW
            self._bounce_since_kick = False
            self._shot_in_flight = True
            self._shot_departed = False
            # Classify the kick relative to B: a hit (ball fired into the opponent → give-away)
            # or a thread (clears the opponent toward the goal mouth → skilful, risky).
            out.update(self._kick_shot_eval(state_a, state_b))

        if self._kick_goal_timer > 0 and phys_info.get("ball_wall_bounce", False):
            self._bounce_since_kick = True

        # ── kick lost to the enemy ──
        if self._kick_lost_timer > 0:
            self._kick_lost_timer -= 1
            if goal_a:
                self._kick_lost_timer = 0           # the kick scored — not lost
            elif owner == "b":
                out["kick_lost"] = True
                self._kick_lost_timer = 0

        # ── skilled goal: kicked in (ball free) and/or banked off a wall ──
        if goal_a and self._kick_goal_timer > 0:
            if a_dist > CONTACT_DIST:               # ball was free, not dribbled over the line
                out["kicked_goal"] = True
            if self._bounce_since_kick:
                out["bank_shot"] = True
        if self._kick_goal_timer > 0:
            self._kick_goal_timer -= 1

        # ── shot out of bounds: a kicked ball that left A and then crossed the white line,
        #    without scoring. Fires the moment the ball goes out (``ball_out_of_play``) rather
        #    than waiting for the referee's relocation grace, so the penalty lands a few steps
        #    after the kick — tight enough for the memoryless policy to credit it. The goal case
        #    is excluded structurally (a bank/rebound that scores fires ``goal_a`` first), and
        #    resolving on recovery / loss-to-B keeps it from misfiring later. ──
        if self._shot_in_flight:
            if not self._shot_departed and a_dist > CONTACT_DIST:
                self._shot_departed = True
            if goal_a:
                self._shot_in_flight = False                       # scored — rewarded, not punished
            elif self._shot_departed and ball_out_of_play(ball_pos):
                out["shot_out_of_bounds"] = True
                self._shot_in_flight = False
            elif self._shot_departed and (a_owns or owner == "b"):
                self._shot_in_flight = False   # A recovered it, or lost to B (see kick_lost)

        # ── blocked shot on our own goal ──
        live_now = _incoming_shot_on_own_goal(ball_pos, ball_vel)
        if (self._shot_live and not self._shot_credited and not live_now
                and not goal_b and a_dist < CONTACT_DIST):
            out["blocked_shot"] = True
            self._shot_credited = True
        if not live_now:
            self._shot_credited = False
        self._shot_live = live_now

        return out

    def _kick_shot_eval(self, state_a: PhysicsState, state_b: PhysicsState) -> dict:
        """Classify A's kick relative to opponent B (called on the kick step).

        - ``kick_at_opponent``: the kick line passes within ``BLOCK_RADIUS`` of B and B is in
          front within ``KICK_AT_OPP_RANGE`` → the ball will strike B and be given away.
        - ``risky_shot``: the kick is aimed at the goal mouth and passes B *clearing* it
          (``BLOCK_RADIUS ≤ perp < RISKY_RADIUS``) → a skilful threaded shot; ``risky_factor``
          rises as the pass gets closer to (but still clears) the defender.
        """
        ball = state_a.ball_pos
        shot_dir = np.array([np.cos(state_a.robot_heading), np.sin(state_a.robot_heading)])
        to_b = state_b.robot_pos - ball
        proj = float(np.dot(to_b, shot_dir))               # opponent's distance along the kick ray
        if proj <= 0:                                      # opponent is behind the kick — irrelevant
            return {}
        perp = float(np.linalg.norm(to_b - shot_dir * proj))

        # Fired straight into the opponent → give-away.
        if proj < KICK_AT_OPP_RANGE and perp < BLOCK_RADIUS:
            return {"kick_at_opponent": True}

        # Otherwise, a thread past the defender toward the goal mouth.
        if shot_dir[0] <= 1e-6:
            return {}
        s_goal = (HALF_W - ball[0]) / shot_dir[0]          # distance along ray to the goal line
        if s_goal <= 0 or proj > s_goal:                   # goal behind us, or B beyond the goal line
            return {}
        y_at = ball[1] + shot_dir[1] * s_goal
        if abs(y_at) >= GOAL_HALF_WIDTH:                   # not aimed at the goal mouth
            return {}
        if BLOCK_RADIUS <= perp < RISKY_RADIUS:            # passes close but clears the defender
            factor = float(np.clip((RISKY_RADIUS - perp) / (RISKY_RADIUS - BLOCK_RADIUS), 0.0, 1.0))
            return {"risky_shot": True, "risky_factor": factor}
        return {}
