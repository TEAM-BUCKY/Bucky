"""RoboCup Junior rule penalties (PWS §1.3–1.4), part of the default PWS profile.

These are not extra shaping. They mirror events that cost the team in a real match:
- a goal against;
- the robot leaving the field (out of bounds, 30 s penalty);
- being declared defective (30 s penalty);
- a lack-of-progress stoppage.
Each is −1 per event; the integer weight sets how bad it is.
"""
from __future__ import annotations

from bucky.reward_terms.registry import RewardContext, reward_term


def _event(ctx: RewardContext, key: str) -> float:
    return -1.0 if ctx.info.get(key, False) else 0.0


@reward_term("goal_against", weight=10, group="rule",
             description="−1 per goal conceded (symmetric with `goal`).")
def goal_against(ctx: RewardContext) -> float:
    return _event(ctx, "goal_against")


@reward_term("out_of_bounds", weight=5, group="rule",
             description="−1 when the robot fully leaves the field (§1.3, 30 s time penalty).")
def out_of_bounds(ctx: RewardContext) -> float:
    return _event(ctx, "out_of_bounds")


@reward_term("defective", weight=3, group="rule",
             description="−1 when the robot is declared defective (§1.3, e.g. stuck in a goal area).")
def defective(ctx: RewardContext) -> float:
    return _event(ctx, "defective")


@reward_term("lack_of_progress", weight=1, group="rule",
             description="−1 per lack-of-progress stoppage (§1.4, ball stuck between robots).")
def lack_of_progress(ctx: RewardContext) -> float:
    return _event(ctx, "lack_of_progress")
