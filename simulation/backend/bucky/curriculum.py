"""Training curriculum stages.

Stage enum + per-stage env configs. Stages:
  APPROACH_STATIC_BALL  → robot learns to reach ball (implemented)
  PUSH_TO_EMPTY_GOAL    → robot learns to score (implemented)
  SCRIPTED_OPPONENT     → STUB: rule-based defender (not yet implemented)
  SELF_PLAY_2V2         → STUB: requires bucky_team.py multi-agent env
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class Stage(str, Enum):
    SELF_PLAY_1V1 = "SELF_PLAY_1V1"
    SELF_PLAY_2V2 = "SELF_PLAY_2V2"


@dataclass
class StageConfig:
    stage: Stage
    max_episode_steps: int
    ball_spawn_radius: float
    goal_present: bool
    active_reward_terms: list[str] = field(default_factory=list)
    opponent_present: bool = False
    # Default PPO entropy coefficient for this stage when the model config doesn't set one.
    # Kick/self-play stages want more exploration (firing the kicker, breaking symmetry).
    recommended_ent_coef: float = 0.01
    description: str = ""


STAGE_CONFIGS: dict[Stage, StageConfig] = {
    Stage.SELF_PLAY_1V1: StageConfig(
        stage=Stage.SELF_PLAY_1V1,
        max_episode_steps=1500,
        ball_spawn_radius=0.5,
        goal_present=True,
        active_reward_terms=[
            "approach", "speed", "ball_to_goal", "possession", "front_alignment", "goal", "goal_against",
            "predicted_goal", "in_goal",
            "steal", "blocked_shot", "kick_attempt", "kick_power_to_goal", "shot_on_goal",
            "kick_goal", "bank_shot", "risky_shot", "kick_lost", "kick_at_opponent",
            "shot_out_of_bounds",
            "out_of_bounds", "lack_of_progress", "defective", "spin", "time_penalty",
            "action_smoothness", "play_oob_ball", "stuck",
        ],
        opponent_present=True,
        recommended_ent_coef=0.025,  # more exploration so the two robots don't lock into a symmetric stalemate
        description="1v1 self-play: learner (robot A) vs a frozen-snapshot opponent (robot B).",
    ),
}


def get_stage_config(stage: Stage | str) -> StageConfig:
    return STAGE_CONFIGS[Stage(stage)]
