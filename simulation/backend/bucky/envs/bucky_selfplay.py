"""Gymnasium 1v1 self-play env (stage SELF_PLAY_1V1).

The learning agent controls robot A (attacks +x). Robot B is the opponent, driven by a
*frozen* policy snapshot via the x-axis mirror trick (:mod:`bucky.selfplay`). The agent's
observation is opponent-aware: the 17-dim single-agent vector plus a 4-beam sonar block
(21 dims total). Rewards reuse :func:`bucky.rewards.compute_rewards` from robot A's frame.

The frozen opponent is hot-swappable via :meth:`set_opponent` so the trainer can refresh
it with newer snapshots during ``learn()`` (callable per-worker through ``env_method``).
"""
from __future__ import annotations

from collections import deque

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from bucky.curriculum import Stage, StageConfig, get_stage_config
from bucky.physics.python_backend import TwoRobotPhysics
from bucky.randomization import DomainRandomConfig, EpisodeRandomization, sample_episode_randomization
from bucky.referee import Referee
from bucky.rewards import RewardConfig, RewardTerms, compute_rewards
from bucky.selfplay import SELF_PLAY_OBS_DIM, build_robot_obs, predict_opponent_action

MAX_LINEAR = 1.0
MAX_OMEGA = 6.0

OBS_LOW = np.full(SELF_PLAY_OBS_DIM, -3.0, dtype=np.float32)
OBS_HIGH = np.full(SELF_PLAY_OBS_DIM, 3.0, dtype=np.float32)


class BuckySelfPlayEnv(gym.Env):
    """1v1 self-play environment: learner (A) vs a frozen-snapshot opponent (B)."""

    metadata = {"render_modes": ["none"], "render_fps": 50}

    def __init__(
        self,
        stage: Stage | str = Stage.SELF_PLAY_1V1,
        domain_rand: bool = True,
        reward_config: RewardConfig | None = None,
        opponent_path: str | None = None,
    ) -> None:
        super().__init__()
        self._stage_cfg: StageConfig = get_stage_config(stage)
        self._rand_cfg = DomainRandomConfig(enabled=domain_rand)
        self._reward_cfg = reward_config or RewardConfig()

        self.observation_space = spaces.Box(OBS_LOW, OBS_HIGH, dtype=np.float32)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)

        self._phys = TwoRobotPhysics()
        self._ref = Referee(match_mode=False)
        self._rng = np.random.default_rng()
        self._ep_rand: EpisodeRandomization = EpisodeRandomization()
        self._action_buffer: deque[np.ndarray] = deque()
        self._step_count = 0
        self._heading_drift = 0.0
        self._opponent = None
        self._last_terms = RewardTerms()
        if opponent_path:
            self.set_opponent(opponent_path)

    # ── opponent management ───────────────────────────────────────────────────
    def set_opponent(self, path: str | None) -> None:
        """Load (or clear) the frozen opponent policy. ``None`` → opponent stands still."""
        if not path:
            self._opponent = None
            return
        try:
            from stable_baselines3 import PPO
            self._opponent = PPO.load(path, device="cpu")
        except Exception:  # noqa: BLE001 — fall back to a passive opponent
            self._opponent = None

    # ── gym API ───────────────────────────────────────────────────────────────
    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        self._phys.reset(seed=seed)
        # Random opening kickoff each episode; the referee performs it.
        self._ref = Referee(match_mode=False,
                            first_kickoff="a" if self._rng.random() < 0.5 else "b")
        self._ref.reset(self._phys)
        self._ep_rand = sample_episode_randomization(self._rand_cfg, self._rng)
        self._action_buffer.clear()
        self._step_count = 0
        self._heading_drift = 0.0
        return self._get_obs(), {}

    def step(self, action: np.ndarray):
        action = np.clip(action, -1.0, 1.0).astype(np.float32)

        self._action_buffer.append(action.copy())
        latency = self._ep_rand.action_latency_steps
        delayed = self._action_buffer[0] if len(self._action_buffer) > latency else action
        if len(self._action_buffer) > latency + 1:
            self._action_buffer.popleft()

        sat = self._ep_rand.motor_saturation
        act_a = (delayed[0] * sat, delayed[1] * sat, delayed[2])
        act_b = self._opponent_action()

        state0 = self._phys.state_a()
        info = self._phys.step(act_a, act_b)
        state1 = self._phys.state_a()                 # post-physics, pre-referee (shaping)

        # The referee enforces all rules and may relocate the ball / remove a robot.
        decision = self._ref.update(self._phys, info)

        self._heading_drift += self._ep_rand.heading_drift_rate

        a_removed = decision.status["a"]["removed"]
        reward_info = {
            "goal_scored": decision.goal_a,
            "goal_against": decision.goal_b,
            "out_of_bounds": any(e.startswith("out_of_bounds_a") for e in decision.events),
            "lack_of_progress": "lack_of_progress" in decision.events,
            "defective": any(e.startswith("defective_a") for e in decision.events),
            "ball_out": decision.ball_relocated,
        }
        reward_terms = compute_rewards(state0, state1, self._reward_cfg, reward_info, action=action)
        self._last_terms = reward_terms
        reward = self._filter_reward(reward_terms)

        # Play continues through goals (the referee kicks off); the episode ends only
        # when the learner (A) is sent off, or on the step budget.
        terminated = bool(a_removed)
        self._step_count += 1
        truncated = self._step_count >= self._stage_cfg.max_episode_steps

        obs = self._get_obs()
        info["goal_a"] = decision.goal_a
        info["goal_b"] = decision.goal_b
        info["referee_events"] = decision.events
        info["reward_terms"] = reward_terms.as_dict()
        return obs, float(reward), terminated, truncated, info

    def render(self):
        pass

    def close(self):
        pass

    # ── viz access (used by LiveVizCallback's self-play branch) ────────────────
    @property
    def physics(self) -> TwoRobotPhysics:
        return self._phys

    @property
    def last_terms(self) -> RewardTerms:
        return self._last_terms

    # ── helpers ───────────────────────────────────────────────────────────────
    def _opponent_action(self):
        if self._opponent is None:
            return (0.0, 0.0, 0.0)
        return predict_opponent_action(
            self._opponent, self._phys.state_b(), self._phys.state_a().robot_pos,
            add_noise=self._rand_cfg.enabled, rng=self._rng,
        )

    def _get_obs(self) -> np.ndarray:
        state_a = self._phys.state_a()
        opp_pos = self._phys.state_b().robot_pos
        obs = build_robot_obs(
            state_a, opponent_pos=opp_pos,
            add_noise=self._rand_cfg.enabled, rng=self._rng,
            heading_drift=self._heading_drift,
        )
        return np.clip(obs, OBS_LOW, OBS_HIGH)

    def _filter_reward(self, terms: RewardTerms) -> float:
        active = set(self._stage_cfg.active_reward_terms)
        if not active:
            return terms.total
        d = terms.as_dict()
        return sum(d.get(k, 0.0) for k in active)
