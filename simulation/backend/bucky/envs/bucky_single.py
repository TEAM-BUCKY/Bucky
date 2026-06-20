"""Gymnasium single-agent env for Bucky (stage 1 & 2).

Observation: robot-egocentric only — no ground-truth global pose.
Action: body-frame velocity [vx, vy, ω] in [-1, 1], scaled to physical limits.
"""
from __future__ import annotations

from collections import deque

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from bucky import field
from bucky.curriculum import Stage, StageConfig, get_stage_config
from bucky.obs import OBS_DIM, build_observation
from bucky.physics.python_backend import PyPhysics
from bucky.play_events import CONTACT_DIST, KICK_GOAL_WINDOW
from bucky.randomization import DomainRandomConfig, EpisodeRandomization, sample_episode_randomization
from bucky.rewards import RewardConfig, RewardTerms, compute_rewards

MAX_LINEAR = 1.0
MAX_OMEGA = 6.0
OOB_GRACE_STEPS = 50
ROBOT_OOB_PENALTY_STEPS = 50

OBS_LOW = np.full(OBS_DIM, -3.0, dtype=np.float32)
OBS_HIGH = np.full(OBS_DIM, 3.0, dtype=np.float32)


class BuckySingleEnv(gym.Env):
    """RoboCup Junior Bucky single-agent gymnasium environment.

    Args:
        stage:        Curriculum stage (determines ball spawn, goal presence, rewards).
        domain_rand:  Enable domain randomization.
        reward_config: Reward term weights (optional override).
        viz_callback: Optional callable(state, reward_terms) for external visualization.
    """

    metadata = {"render_modes": ["human", "none"], "render_fps": 50}

    def __init__(
        self,
        stage: Stage | str = Stage.APPROACH_STATIC_BALL,
        domain_rand: bool = True,
        reward_config: RewardConfig | None = None,
        viz_callback=None,
    ) -> None:
        super().__init__()
        self._stage_cfg: StageConfig = get_stage_config(stage)
        self._rand_cfg = DomainRandomConfig(enabled=domain_rand)
        self._reward_cfg = reward_config or RewardConfig()
        self._viz_callback = viz_callback

        self.observation_space = spaces.Box(OBS_LOW, OBS_HIGH, dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(4,), dtype=np.float32)

        self._physics = PyPhysics()
        self._rng = np.random.default_rng()
        self._ep_rand: EpisodeRandomization = EpisodeRandomization()
        self._action_buffer: deque[np.ndarray] = deque()
        self._step_count = 0
        self._heading_drift = 0.0
        self._ball_out_steps = 0
        self._robot_penalty_steps = 0
        self._kick_goal_timer = 0
        self._bounce_since_kick = False

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        self._physics.reset(seed=seed)
        self._ep_rand = sample_episode_randomization(self._rand_cfg, self._rng)
        self._spawn_ball()
        self._action_buffer.clear()
        self._step_count = 0
        self._heading_drift = 0.0
        self._ball_out_steps = 0
        self._robot_penalty_steps = 0
        self._kick_goal_timer = 0
        self._bounce_since_kick = False
        obs = self._get_obs()
        return obs, {}

    def step(self, action: np.ndarray):
        action = np.clip(action, -1.0, 1.0).astype(np.float32)

        # Robot is frozen during OOB penalty — ignore the agent's action.
        if self._robot_penalty_steps > 0:
            self._robot_penalty_steps -= 1
            vx, vy, omega, kick = 0.0, 0.0, 0.0, 0.0
        else:
            self._action_buffer.append(action.copy())
            latency = self._ep_rand.action_latency_steps
            buffered = len(self._action_buffer) > latency
            delayed_action = self._action_buffer[0] if buffered else action
            if len(self._action_buffer) > latency + 1:
                self._action_buffer.popleft()

            sat = self._ep_rand.motor_saturation
            vx = delayed_action[0] * MAX_LINEAR * sat
            vy = delayed_action[1] * MAX_LINEAR * sat
            omega = delayed_action[2] * MAX_OMEGA
            kick = delayed_action[3]

        state0 = self._physics._make_state()
        state1, info = self._physics.step(vx, vy, omega, kick)

        # Skilled-goal tracking (no opponent here): a goal counts as "kicked" if it follows
        # a recent kick with the ball free, and as a "bank shot" if a wall bounce intervened.
        if info["kicked"]:
            self._kick_goal_timer = KICK_GOAL_WINDOW
            self._bounce_since_kick = False
        if self._kick_goal_timer > 0 and info["ball_wall_bounce"]:
            self._bounce_since_kick = True
        if info["goal_scored"] and self._kick_goal_timer > 0:
            d_robot_ball = float(np.linalg.norm(state1.ball_pos - state1.robot_pos))
            info["kicked_goal"] = d_robot_ball > CONTACT_DIST
            info["bank_shot"] = self._bounce_since_kick
        if self._kick_goal_timer > 0:
            self._kick_goal_timer -= 1

        self._heading_drift += self._ep_rand.heading_drift_rate

        if info["ball_out"]:
            self._ball_out_steps += 1
        else:
            self._ball_out_steps = 0
        ball_out_of_bounds = self._ball_out_steps >= OOB_GRACE_STEPS
        if ball_out_of_bounds:
            # Rule §4.8/§4.9.5: replace the ball at the nearest neutral spot, play on.
            spot = field.nearest_neutral_spot(self._physics._ball_pos,
                                              occupied=[self._physics._robot_pos],
                                              clearance=field.COLLISION_DIST)
            self._physics._ball_pos = spot.copy()
            self._physics._ball_vel = np.zeros(2)
            self._ball_out_steps = 0

        info["ball_out"] = ball_out_of_bounds

        robot_just_out = info["robot_fully_out"] and self._robot_penalty_steps == 0
        if robot_just_out:
            self._robot_penalty_steps = ROBOT_OOB_PENALTY_STEPS

        info["out_of_bounds"] = robot_just_out

        reward_terms = compute_rewards(state0, state1, self._reward_cfg, info, action=action)
        reward = self._filter_reward(reward_terms)

        terminated = bool(info["goal_scored"] or info["robot_fully_out"])
        self._step_count += 1
        truncated = self._step_count >= self._stage_cfg.max_episode_steps

        if self._viz_callback is not None:
            self._viz_callback(state1, reward_terms)

        obs = self._get_obs(state1)
        info["reward_terms"] = reward_terms.as_dict()
        return obs, float(reward), terminated, truncated, info

    def render(self):
        pass

    def close(self):
        pass

    def _get_obs(self, state=None) -> np.ndarray:
        if state is None:
            state = self._physics._make_state()
        add_noise = self._rand_cfg.enabled
        kick_ready = 1.0 if self._physics._kick_cooldown == 0 else 0.0
        obs = build_observation(
            state,
            add_noise=add_noise,
            rng=self._rng,
            heading_drift=self._heading_drift,
            kick_ready=kick_ready,
        )
        return np.clip(obs, OBS_LOW, OBS_HIGH)

    def _filter_reward(self, terms: RewardTerms) -> float:
        active = set(self._stage_cfg.active_reward_terms)
        if not active:
            return terms.total
        d = terms.as_dict()
        return sum(d.get(k, 0.0) for k in active)

    def _spawn_ball(self) -> None:
        angle = self._rng.uniform(-np.pi, np.pi)
        r = self._rng.uniform(0.15, self._stage_cfg.ball_spawn_radius)
        robot_pos = self._physics._robot_pos
        self._physics._ball_pos = robot_pos + r * np.array([np.cos(angle), np.sin(angle)])
        margin = 0.1
        self._physics._ball_pos = np.clip(
            self._physics._ball_pos,
            [-(field.HALF_W - margin), -(field.HALF_H - margin)],
            [field.HALF_W - margin, field.HALF_H - margin],
        )
        self._physics._ball_vel = np.zeros(2)
