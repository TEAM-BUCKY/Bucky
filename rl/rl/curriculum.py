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
    SCRIPTED_OPPONENT = "SCRIPTED_OPPONENT"
    SELF_PLAY_2V2 = "SELF_PLAY_2V2"


@dataclass
class StageConfig:
    stage: Stage
    max_episode_steps: int
    ball_spawn_radius: float
    goal_present: bool
    active_reward_terms: list[str] = field(default_factory=list)
    description: str = ""


STAGE_CONFIGS: dict[Stage, StageConfig] = {
    Stage.APPROACH_STATIC_BALL: StageConfig(
        stage=Stage.APPROACH_STATIC_BALL,
        max_episode_steps=300,
        ball_spawn_radius=1.0,
        goal_present=False,
        active_reward_terms=["approach", "possession", "spin", "time_penalty"],
        description="Robot must reach ball. No goal — approach shaping only.",
    ),
    Stage.PUSH_TO_EMPTY_GOAL: StageConfig(
        stage=Stage.PUSH_TO_EMPTY_GOAL,
        max_episode_steps=500,
        ball_spawn_radius=0.5,
        goal_present=True,
        active_reward_terms=[
            "approach", "ball_to_goal", "possession", "goal",
            "out_of_bounds", "spin", "time_penalty", "action_magnitude",
        ],
        description="Robot must score in an undefended goal.",
    ),
    Stage.SCRIPTED_OPPONENT: StageConfig(
        stage=Stage.SCRIPTED_OPPONENT,
        max_episode_steps=500,
        ball_spawn_radius=0.5,
        goal_present=True,
        active_reward_terms=[],
        description="STUB: scripted rule-based opponent. Not yet implemented.",
    ),
    Stage.SELF_PLAY_2V2: StageConfig(
        stage=Stage.SELF_PLAY_2V2,
        max_episode_steps=1000,
        ball_spawn_radius=0.5,
        goal_present=True,
        active_reward_terms=[],
        description="STUB: requires PettingZoo 2v2 env. Not yet implemented.",
    ),
}


def get_stage_config(stage: Stage | str) -> StageConfig:
    return STAGE_CONFIGS[Stage(stage)]
