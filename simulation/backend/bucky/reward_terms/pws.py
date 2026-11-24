"""The five PWS reward weights (PWS §9.3, Bijlage A "gewichtentabel versie 1.0").

Following Bassani et al. (2020), *A Framework for Studying Reinforcement Learning and Sim-to-Real
in Robot Soccer*, the reward rewards the actual goal (scoring) and also adds shaping terms
that guide the model during the task:

* goal           — scoring (the actual objective)
* ball_distance  — getting closer to the ball ("dichterbij de bal geeft meer punten")
* ball_position  — the ball moving away from our half toward the opponent's half
* kick_goal      — a goal scored with the kicker (a shot, not a dribble)
* smoothness     — smooth driving ("gladheid"): penalise jerky action changes

The shaping terms ``ball_distance`` and ``ball_position`` are *potential-based*,
``Φ(s') − Φ(s)`` (Ng, Harada & Russell, 1999). Two consequences:

1. Over an episode they telescope to ``Φ(end) − Φ(start)``, so they are bounded and cannot be
   farmed by e.g. hovering near the ball.
2. They do not change which policy is optimal. They only make it faster to find.
"""
from __future__ import annotations

import numpy as np

from bucky.game.field import ARENA_HALF_X, ARENA_HALF_Y, FIELD_W, OPP_GOAL
from bucky.reward_terms.registry import RewardContext, reward_term

OWN_GOAL = -OPP_GOAL
# Largest possible robot↔ball distance (corner to corner of the arena, incl. the wall band).
ARENA_DIAG = float(np.hypot(2 * ARENA_HALF_X, 2 * ARENA_HALF_Y))


def ball_distance_potential(robot_pos, ball_pos) -> float:
    """Φ_d ∈ [0, 1]: 1 when the robot is on the ball, 0 at the largest possible distance."""
    d = float(np.linalg.norm(np.asarray(ball_pos) - np.asarray(robot_pos)))
    return 1.0 - min(1.0, d / ARENA_DIAG)


def ball_position_potential(ball_pos) -> float:
    """Φ_b ∈ [-0.5, 0.5]: +0.5 at the opponent goal, -0.5 at our own goal, 0 on the centre line.

    Bassani's ball-potential: (distance to own goal − distance to opponent goal), normalised.
    """
    d_own = float(np.linalg.norm(np.asarray(ball_pos) - OWN_GOAL))
    d_opp = float(np.linalg.norm(np.asarray(ball_pos) - OPP_GOAL))
    return float(np.clip((d_own - d_opp) / (2.0 * FIELD_W), -0.5, 0.5))


@reward_term("goal", weight=10, group="pws",
             description="+1 per goal scored (the real objective).")
def goal(ctx: RewardContext) -> float:
    return 1.0 if ctx.info.get("goal_scored", False) else 0.0


@reward_term("ball_distance", weight=2, group="pws",
             description="ΔΦ, Φ = 1 − d(robot, ball)/arena diagonal. Getting closer to the ball pays; "
                         "the episode sum is in [-1, 1].")
def ball_distance(ctx: RewardContext) -> float:
    return (ball_distance_potential(ctx.s1.robot_pos, ctx.s1.ball_pos)
            - ball_distance_potential(ctx.s0.robot_pos, ctx.s0.ball_pos))


@reward_term("ball_position", weight=4, group="pws",
             description="ΔΦ, Φ = (d(ball, own goal) − d(ball, opp goal)) / 2·field length. "
                         "Moving the ball toward the opponent goal pays.")
def ball_position(ctx: RewardContext) -> float:
    return ball_position_potential(ctx.s1.ball_pos) - ball_position_potential(ctx.s0.ball_pos)


@reward_term("kick_goal", weight=5, group="pws",
             description="+1 per goal scored with the kicker (on top of `goal`): a shot is worth "
                         "more than a dribbled goal.")
def kick_goal(ctx: RewardContext) -> float:
    info = ctx.info
    return 1.0 if info.get("goal_scored", False) and info.get("kicked_goal", False) else 0.0


@reward_term("smoothness", weight=2, group="pws",
             description="−Σ|a_t − a_{t−1}| over vx, vy, ω, divided by 6·episode length (the "
                         "largest possible change), so the episode sum is in [-1, 0].")
def smoothness(ctx: RewardContext) -> float:
    if ctx.action is None or ctx.prev_action is None:
        return 0.0
    da = np.abs(np.asarray(ctx.action[:3], dtype=float) - np.asarray(ctx.prev_action[:3], dtype=float))
    return -float(np.sum(da)) / (6.0 * max(1, int(ctx.max_steps)))
