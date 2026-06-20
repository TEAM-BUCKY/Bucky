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
    APPROACH_STATIC_BALL = "APPROACH_STATIC_BALL"
    PUSH_TO_EMPTY_GOAL = "PUSH_TO_EMPTY_GOAL"
    SELF_PLAY_1V1 = "SELF_PLAY_1V1"
    SCRIPTED_OPPONENT = "SCRIPTED_OPPONENT"
    SELF_PLAY_2V2 = "SELF_PLAY_2V2"


@dataclass
class StageConfig:
    stage: Stage
    max_episode_steps: int
    ball_spawn_radius: float
    goal_present: bool
    active_reward_terms: list[str] = field(default_factory=list)
    opponent_present: bool = False
    description: str = ""


STAGE_CONFIGS: dict[Stage, StageConfig] = {
    Stage.APPROACH_STATIC_BALL: StageConfig(
        stage=Stage.APPROACH_STATIC_BALL,
        max_episode_steps=600,
        ball_spawn_radius=0.5,
        goal_present=False,
        active_reward_terms=[
            "approach", "possession", "front_alignment", "out_of_bounds",
            "spin", "time_penalty", "action_magnitude",
        ],
        opponent_present=False,
        description="Stage 1: robot learns to reach and face a (near-)static ball.",
    ),
    Stage.PUSH_TO_EMPTY_GOAL: StageConfig(
        stage=Stage.PUSH_TO_EMPTY_GOAL,
        max_episode_steps=800,
        ball_spawn_radius=0.6,
        goal_present=True,
        active_reward_terms=[
            "approach", "ball_to_goal", "possession", "front_alignment", "goal",
            "kick_goal", "bank_shot",
            "out_of_bounds", "spin", "time_penalty", "action_magnitude",
        ],
        opponent_present=False,
        description="Stage 2: robot learns to drive/kick the ball into an empty goal.",
    ),
    Stage.SELF_PLAY_1V1: StageConfig(
        stage=Stage.SELF_PLAY_1V1,
        max_episode_steps=1500,
        ball_spawn_radius=0.5,
        goal_present=True,
        active_reward_terms=[
            "approach", "ball_to_goal", "possession", "front_alignment", "goal", "goal_against",
            "steal", "blocked_shot", "kick_goal", "bank_shot", "risky_shot", "kick_lost",
            "out_of_bounds", "lack_of_progress", "defective", "spin", "time_penalty",
            "action_magnitude",
        ],
        opponent_present=True,
        description="1v1 self-play: learner (robot A) vs a frozen-snapshot opponent (robot B).",
    ),
}


def get_stage_config(stage: Stage | str) -> StageConfig:
    return STAGE_CONFIGS[Stage(stage)]
