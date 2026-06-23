"""Training curriculum stages.

Stage enum + per-stage env configs. Stages:
  APPROACH_STATIC_BALL  → robot learns to reach ball (internal full-training phase)
  PUSH_TO_EMPTY_GOAL    → robot learns to score (internal full-training phase)
  SELF_PLAY_1V1         → 1v1 vs frozen-snapshot opponent
  FULL_TRAINING         → meta-stage: runs APPROACH → PUSH → SELF_PLAY in one job (see FULL_TRAINING_PHASES)
  SELF_PLAY_2V2         → STUB: requires bucky_team.py multi-agent env
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class Stage(str, Enum):
    APPROACH_STATIC_BALL = "APPROACH_STATIC_BALL"
    PUSH_TO_EMPTY_GOAL = "PUSH_TO_EMPTY_GOAL"
    SELF_PLAY_1V1 = "SELF_PLAY_1V1"
    FULL_TRAINING = "FULL_TRAINING"
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
    # ── Foundational single-agent drills (no opponent) ──────────────────────────
    # Run in BuckySingleEnv (35-dim obs). Train these first, then transfer into
    # SELF_PLAY_1V1 (the trainer expands the 35→39-dim input layer on resume).
    Stage.APPROACH_STATIC_BALL: StageConfig(
        stage=Stage.APPROACH_STATIC_BALL,
        max_episode_steps=300,
        ball_spawn_radius=0.6,           # ball placed up to 0.6 m from the robot, stationary
        goal_present=True,
        active_reward_terms=[
            "approach", "speed", "possession", "front_alignment",
            "stuck", "out_of_bounds", "in_goal", "spin",
            "time_penalty", "action_smoothness",
        ],
        opponent_present=False,
        recommended_ent_coef=0.01,
        description="Robot learns to drive to a stationary ball (no goal/kick terms).",
    ),
    Stage.PUSH_TO_EMPTY_GOAL: StageConfig(
        stage=Stage.PUSH_TO_EMPTY_GOAL,
        max_episode_steps=600,
        ball_spawn_radius=0.9,
        goal_present=True,
        active_reward_terms=[
            "approach", "speed", "ball_to_goal", "possession", "front_alignment",
            "goal", "predicted_goal", "in_goal",
            "kick_attempt", "kick_power_to_goal", "shot_on_goal", "kick_goal", "bank_shot",
            "shot_out_of_bounds", "out_of_bounds", "lack_of_progress", "defective", "spin",
            "time_penalty", "action_smoothness", "play_oob_ball", "stuck",
        ],
        opponent_present=False,
        recommended_ent_coef=0.02,       # encourage firing the kicker
        description="Robot learns to drive/kick the ball into an empty goal (no opponent).",
    ),
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

# Safety net: FULL_TRAINING is a meta-stage the trainer expands into the phases below — it never
# builds an env directly. Mirror SELF_PLAY_1V1 so any incidental get_stage_config() call is sane.
STAGE_CONFIGS[Stage.FULL_TRAINING] = StageConfig(
    **{**STAGE_CONFIGS[Stage.SELF_PLAY_1V1].__dict__, "stage": Stage.FULL_TRAINING,
       "description": "Meta-stage: APPROACH → PUSH → SELF_PLAY in one run (see FULL_TRAINING_PHASES)."}
)

# The phases FULL_TRAINING runs, in order, with each one's default fraction of the total budget
# (steps or wall-clock). Overridable per run via the config's ``full_training_split``.
FULL_TRAINING_PHASES: list[tuple[Stage, float]] = [
    (Stage.APPROACH_STATIC_BALL, 0.15),
    (Stage.PUSH_TO_EMPTY_GOAL, 0.25),
    (Stage.SELF_PLAY_1V1, 0.60),
]


def get_stage_config(stage: Stage | str) -> StageConfig:
    return STAGE_CONFIGS[Stage(stage)]
