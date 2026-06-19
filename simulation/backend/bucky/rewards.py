"""Reward computation for Bucky.

All reward terms are individually weighted and individually logged to TensorBoard.
Potential-based shaping: R_shaping = Φ(s) - Φ(s') so moving toward goal gives +reward.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from bucky.field import OPP_GOAL
from bucky.physics.backend import PhysicsState

CAPTURE_RADIUS = 0.14
MAX_OMEGA_PENALTY = 3.0
ALIGN_RADIUS = 0.4  # proximity fade-in distance for front_alignment (m)


@dataclass
class RewardConfig:
    w_approach: float = 0.5
    w_ball_to_goal: float = 2.5

    w_possession: float = 1
    w_front_align: float = 0.3

    w_goal: float = 20.0
    w_goal_against: float = -20.0

    w_out_of_bounds: float = -10.0        # robot fully out → 30 s suspension (rules §4.9)
    w_lack_of_progress: float = -2.0      # ball stuck between robots (rules §4.6)
    w_defective: float = -10.0            # removed as defective (rules §4.7)
    w_spin: float = -0.2

    w_time: float = -0.001
    w_action_mag: float = -0.005


@dataclass
class RewardTerms:
    approach: float = 0.0
    ball_to_goal: float = 0.0

    possession: float = 0.0
    front_alignment: float = 0.0

    goal: float = 0.0
    goal_against: float = 0.0

    out_of_bounds: float = 0.0
    lack_of_progress: float = 0.0
    defective: float = 0.0
    spin: float = 0.0

    time_penalty: float = 0.0
    action_magnitude: float = 0.0

    @property
    def total(self) -> float:
        return (self.approach + self.ball_to_goal + self.possession +
                self.front_alignment + self.goal + self.goal_against +
                self.out_of_bounds + self.lack_of_progress + self.defective +
                self.spin + self.time_penalty + self.action_magnitude)

    def as_dict(self) -> dict[str, float]:
        return {
            "approach": self.approach,
            "ball_to_goal": self.ball_to_goal,
            "possession": self.possession,
            "front_alignment": self.front_alignment,
            "goal": self.goal,
            "goal_against": self.goal_against,
            "out_of_bounds": self.out_of_bounds,
            "lack_of_progress": self.lack_of_progress,
            "defective": self.defective,
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
        heading_vec = np.array([np.cos(s1.robot_heading), np.sin(s1.robot_heading)])
        if np.dot(heading_vec, s1.ball_pos - s1.robot_pos) > 0:
            terms.possession = config.w_possession
        else:
            terms.possession = -config.w_possession

    # Reward setting up to catch the ball and drive it at the enemy goal: the
    # robot's front (where the catch zone is) must face the ball so it can catch
    # it, AND the robot must be behind the ball (on the side away from the goal)
    # so that catching and driving forward sends the ball toward the goal.
    # Faded in by proximity.
    to_goal = OPP_GOAL - s1.ball_pos
    to_goal_dist = float(np.linalg.norm(to_goal))
    if to_goal_dist > 1e-6 and d_robot_ball_1 > 1e-6:
        goal_dir = to_goal / to_goal_dist
        approach_dir = (s1.ball_pos - s1.robot_pos) / d_robot_ball_1
        heading_vec = np.array([np.cos(s1.robot_heading), np.sin(s1.robot_heading)])
        face_ball = float(np.dot(heading_vec, approach_dir))  # front/catch zone toward ball
        drive_pos = float(np.dot(approach_dir, goal_dir))     # behind ball -> can drive to goal
        align = 0.5 * (face_ball + drive_pos)
        prox = max(0.0, 1.0 - d_robot_ball_1 / ALIGN_RADIUS)
        terms.front_alignment = config.w_front_align * align * prox

    if info.get("goal_scored", False):
        terms.goal = config.w_goal

    if info.get("goal_against", False):
        terms.goal_against = config.w_goal_against

    if info.get("out_of_bounds", False):
        terms.out_of_bounds = config.w_out_of_bounds

    if info.get("lack_of_progress", False):
        terms.lack_of_progress = config.w_lack_of_progress

    if info.get("defective", False):
        terms.defective = config.w_defective

    excess_spin = max(0.0, abs(s1.robot_omega) - MAX_OMEGA_PENALTY)
    terms.spin = config.w_spin * excess_spin

    terms.time_penalty = config.w_time

    if action is not None:
        terms.action_magnitude = config.w_action_mag * float(np.sum(action**2))

    return terms
