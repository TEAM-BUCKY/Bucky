"""Reward computation for Bucky RL.

All reward terms are individually weighted and individually logged to TensorBoard.
Potential-based shaping: R_shaping = Φ(s) - Φ(s') so moving toward goal gives +reward.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from rl.physics.backend import PhysicsState
from rl.physics.python_backend import FIELD_W

OPP_GOAL = np.array([FIELD_W / 2, 0.0])
CAPTURE_RADIUS = 0.14
MAX_OMEGA_PENALTY = 3.0


@dataclass
class RewardConfig:
    w_approach: float = 0.3
    w_ball_to_goal: float = 1.0
    w_possession: float = 0.1
    w_goal: float = 10.0
    w_out_of_bounds: float = -5.0
    w_spin: float = -0.02
    w_time: float = -0.001
    w_action_mag: float = -0.005


@dataclass
class RewardTerms:
    approach: float = 0.0
    ball_to_goal: float = 0.0
    possession: float = 0.0
    goal: float = 0.0
    out_of_bounds: float = 0.0
    spin: float = 0.0
    time_penalty: float = 0.0
    action_magnitude: float = 0.0

    @property
    def total(self) -> float:
        return (self.approach + self.ball_to_goal + self.possession +
                self.goal + self.out_of_bounds + self.spin +
                self.time_penalty + self.action_magnitude)

    def as_dict(self) -> dict[str, float]:
        return {
            "approach": self.approach,
            "ball_to_goal": self.ball_to_goal,
            "possession": self.possession,
            "goal": self.goal,
            "out_of_bounds": self.out_of_bounds,
            "spin": self.spin,
            "time_penalty": self.time_penalty,
            "action_magnitude": self.action_magnitude,
        }


def compute_rewards(
    s0: PhysicsState,
    s1: PhysicsState,
    config: RewardConfig,
    info: dict,
    action: np.ndarray | None = None,
) -> RewardTerms:
    terms = RewardTerms()

    d_robot_ball_0 = float(np.linalg.norm(s0.ball_pos - s0.robot_pos))
    d_robot_ball_1 = float(np.linalg.norm(s1.ball_pos - s1.robot_pos))
    terms.approach = config.w_approach * (d_robot_ball_0 - d_robot_ball_1)

    d_ball_goal_0 = float(np.linalg.norm(s0.ball_pos - OPP_GOAL))
    d_ball_goal_1 = float(np.linalg.norm(s1.ball_pos - OPP_GOAL))
    terms.ball_to_goal = config.w_ball_to_goal * (d_ball_goal_0 - d_ball_goal_1)

    if d_robot_ball_1 < CAPTURE_RADIUS:
        terms.possession = config.w_possession

    if info.get("goal_scored", False):
        terms.goal = config.w_goal

    if info.get("out_of_bounds", False):
        terms.out_of_bounds = config.w_out_of_bounds

    excess_spin = max(0.0, abs(s1.robot_omega) - MAX_OMEGA_PENALTY)
    terms.spin = config.w_spin * excess_spin

    terms.time_penalty = config.w_time

    if action is not None:
        terms.action_magnitude = config.w_action_mag * float(np.sum(action**2))

    return terms
