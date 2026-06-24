"""Gymnasium 1v1 self-play env (stage SELF_PLAY_1V1).

The learning agent controls robot A (attacks +x). Robot B is the opponent, driven by a
*frozen* policy snapshot via the x-axis mirror trick (:mod:`bucky.selfplay`). The agent's
observation is opponent-aware: the 18-dim single-agent vector plus a 4-beam sonar block
(22 dims total). Rewards reuse :func:`bucky.rewards.compute_rewards` from robot A's frame.

The frozen opponent is hot-swappable via :meth:`set_opponent` so the trainer can refresh
it with newer snapshots during ``learn()`` (callable per-worker through ``env_method``).
"""
from __future__ import annotations

from collections import deque

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from bucky.curriculum import Stage, StageConfig, get_stage_config
from bucky.physics.python_backend import TwoRobotPhysics, predict_goal_by_rollout
from bucky.play_events import PlayEventTracker
from bucky.randomization import DomainRandomConfig, EpisodeRandomization, sample_episode_randomization
from bucky.game.field import ball_out_of_play
from bucky.game.referee import Referee
from bucky.rewards import ALIGN_RADIUS, RewardConfig, RewardTerms, compute_rewards
from bucky.selfplay import (
    SELF_PLAY_OBS_DIM, build_robot_obs, load_numpy_opponent, predict_opponent_action,
)

# Action scaling (MAX_LINEAR / MAX_OMEGA) lives in python_backend and is applied inside
# TwoRobotPhysics._drive — this env passes the raw normalized action straight through.

# Stochastic-opponent exploration scale (training only). The frozen opponent samples its
# action around the policy mean at this fraction of the policy's own std, so the learner
# faces a non-deterministic copy of itself instead of one memorizable pattern.
OPP_TEMPERATURE = 0.8
# Wider lateral spawn jitter than the physics default (0.05 m) so self-play episodes start
# spread across the field width — encouraging play out to the wings, not just down the middle.
SELF_PLAY_SPAWN_JITTER = 0.25
# Whether a confidently-predicted goal ends the (long, multi-goal) self-play episode early. Default
# off: a ball-only rollout ignores the opponent's *future* moves, so terminating risks ending an
# episode the defender would have saved. The predicted-goal bonus still fires (closing the loop);
# only the early *termination* is gated here. The interception safeguard (frozen opponent on the
# ball's path → not confident) further protects the static case.
EARLY_TERMINATE_ON_PREDICTED_GOAL = False

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
        self.action_space = spaces.Box(-1.0, 1.0, shape=(4,), dtype=np.float32)

        self._phys = TwoRobotPhysics()
        self._ref = Referee(match_mode=False)
        self._rng = np.random.default_rng()
        self._ep_rand: EpisodeRandomization = EpisodeRandomization()
        self._action_buffer: deque[np.ndarray] = deque()
        self._prev_action = np.zeros(4, dtype=np.float32)
        self._events = PlayEventTracker()
        self._step_count = 0
        self._heading_drift = 0.0
        self._opponent = None              # opponent for the current episode (picked in reset)
        self._opponent_pool: list = []     # rolling pool of frozen snapshots to sample from
        self._last_terms = RewardTerms()
        self._last_reward_info: dict = {}
        self._dwell_steps = 0
        self._predicted_goal_guard = 0     # steps remaining to suppress a double-paid real goal
        if opponent_path:
            self.set_opponent(opponent_path)

    # ── opponent management ───────────────────────────────────────────────────
    def set_opponent(self, path: str | None) -> None:
        """Load (or clear) a single frozen opponent. ``None`` → opponent stands still.

        Back-compat shim around :meth:`set_opponent_pool` — sets a pool of one. ``path`` is a
        ``.npz`` of exported policy weights (see ``bucky.selfplay``), evaluated in pure numpy
        so this stays torch-free inside SubprocVecEnv workers.
        """
        self.set_opponent_pool([path] if path else [])

    def set_opponent_pool(self, paths) -> None:
        """Load a rolling pool of frozen opponents; each episode samples one (fictitious
        self-play). Facing a *mix* of past snapshots rather than only the latest stops the
        learner overfitting to one opponent and breaks the cyclic "counter the counter"
        collapse. Bad/missing paths are skipped; an empty pool → a passive opponent.
        """
        pool = []
        for p in paths or []:
            if not p:
                continue
            try:
                pool.append(load_numpy_opponent(p))
            except Exception:  # noqa: BLE001 — skip a bad snapshot, keep the rest
                pass
        self._opponent_pool = pool
        # Make a sane choice available immediately (before the next reset re-samples).
        self._opponent = pool[0] if pool else None

    # ── gym API ───────────────────────────────────────────────────────────────
    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        # Sample this episode's opponent from the pool (fictitious self-play) and widen the
        # opening spread so episodes start across the field, not always down the middle.
        if self._opponent_pool:
            self._opponent = self._opponent_pool[self._rng.integers(len(self._opponent_pool))]
        self._phys.reset(seed=seed, spawn_jitter=SELF_PLAY_SPAWN_JITTER)
        # Random opening kickoff each episode; the referee performs it.
        self._ref = Referee(match_mode=False,
                            first_kickoff="a" if self._rng.random() < 0.5 else "b")
        self._ref.reset(self._phys)
        self._ep_rand = sample_episode_randomization(self._rand_cfg, self._rng)
        self._action_buffer.clear()
        self._prev_action = np.zeros(4, dtype=np.float32)
        self._events.reset()
        self._step_count = 0
        self._heading_drift = 0.0
        self._dwell_steps = 0
        self._predicted_goal_guard = 0
        return self._get_obs(), {}

    def step(self, action: np.ndarray):
        action = np.clip(action, -1.0, 1.0).astype(np.float32)

        self._action_buffer.append(action.copy())
        latency = self._ep_rand.action_latency_steps
        delayed = self._action_buffer[0] if len(self._action_buffer) > latency else action
        if len(self._action_buffer) > latency + 1:
            self._action_buffer.popleft()

        sat = self._ep_rand.motor_saturation
        act_a = (delayed[0] * sat, delayed[1] * sat, delayed[2], delayed[3])
        act_b = self._opponent_action()

        state0 = self._phys.state_a()
        info = self._phys.step(act_a, act_b)
        state1 = self._phys.state_a()                 # post-physics, pre-referee (shaping)
        state_b1 = self._phys.state_b()

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
            # Raw crossing flag (pre-relocation): the ball is past the white line this step.
            # Suppresses positive shaping and drives the play-out-of-bounds penalty.
            "ball_out_raw": ball_out_of_play(state1.ball_pos),
            # Learner is robot A; surface its legal-kick flag for the dense kick-shaping terms.
            "kicked": bool(info.get("kicked_a", False)),
        }
        # Tell the play-event tracker *why* the ball was relocated, so it can tell an
        # out-of-bounds shot ("out_of_reach") from a lack-of-progress / void-goal reset.
        info["ball_oob_relocated"] = "out_of_reach" in decision.events
        # Opponent-aware skilled-play events (steal, block, risky shot, kick lost, kicked/bank goal,
        # shot out of bounds).
        reward_info.update(self._events.update(state1, state_b1, info))

        # Dwell counter (consecutive steps A lingers near the ball within ALIGN_RADIUS, facing it)
        # → decays the possession + front_alignment rewards so camping stops paying.
        d_ball_a = float(np.linalg.norm(state1.ball_pos - state1.robot_pos))
        heading_a = np.array([np.cos(state1.robot_heading), np.sin(state1.robot_heading)])
        facing_a = float(np.dot(heading_a, state1.ball_pos - state1.robot_pos)) > 0
        if d_ball_a < ALIGN_RADIUS and facing_a:
            self._dwell_steps += 1
        else:
            self._dwell_steps = 0
        reward_info["dwell_steps"] = self._dwell_steps

        # Look-ahead "inevitable goal" for the learner (A): on a kick that didn't already score,
        # roll the ball forward with the opponent as a static obstacle. Confident only if it scores
        # *and* the path doesn't pass through the opponent (interception safeguard).
        predicted_goal = False
        if reward_info.get("kicked") and not decision.goal_a:
            scored, _, intercepted = predict_goal_by_rollout(
                state1.ball_pos, state1.ball_vel, attack_sign=1,
                horizon_steps=int(self._reward_cfg.predicted_goal_horizon_steps),
                opponent_pos=state_b1.robot_pos,
            )
            predicted_goal = scored and not intercepted
        reward_info["predicted_goal"] = predicted_goal

        # Avoid double-paying: a predicted goal pre-pays the credit, so suppress the matching real
        # goal (and its kick bonus) if it lands within the rollout horizon.
        if predicted_goal:
            self._predicted_goal_guard = int(self._reward_cfg.predicted_goal_horizon_steps)
        elif self._predicted_goal_guard > 0:
            self._predicted_goal_guard -= 1
        if self._predicted_goal_guard > 0 and decision.goal_a:
            reward_info["goal_scored"] = False
            reward_info["kicked_goal"] = False
            self._predicted_goal_guard = 0

        reward_terms = compute_rewards(state0, state1, self._reward_cfg, reward_info,
                                       action=action, prev_action=self._prev_action)
        self._last_terms = reward_terms
        self._last_reward_info = reward_info  # the exact signals the reward fn saw (for debugging)
        reward = self._filter_reward(reward_terms)
        self._prev_action = action.copy()

        # Play continues through goals (the referee kicks off); the episode ends only
        # when the learner (A) is sent off, or on the step budget.
        terminated = bool(a_removed)
        if EARLY_TERMINATE_ON_PREDICTED_GOAL and predicted_goal:
            terminated = True
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

    @property
    def last_reward_info(self) -> dict:
        """The signal dict the reward function saw this step (flags + play-events)."""
        return self._last_reward_info

    # ── helpers ───────────────────────────────────────────────────────────────
    def _opponent_action(self):
        if self._opponent is None:
            return (0.0, 0.0, 0.0, 0.0)
        kick_ready = 1.0 if self._phys._b_kick_cooldown == 0 else 0.0
        return predict_opponent_action(
            self._opponent, self._phys.state_b(), self._phys.state_a().robot_pos,
            add_noise=self._rand_cfg.enabled, rng=self._rng, kick_ready=kick_ready,
            deterministic=False, temperature=OPP_TEMPERATURE,
        )

    def _get_obs(self) -> np.ndarray:
        state_a = self._phys.state_a()
        opp_pos = self._phys.state_b().robot_pos
        kick_ready = 1.0 if self._phys._a_kick_cooldown == 0 else 0.0
        obs = build_robot_obs(
            state_a, opponent_pos=opp_pos,
            add_noise=self._rand_cfg.enabled, rng=self._rng,
            heading_drift=self._heading_drift, kick_ready=kick_ready,
        )
        return np.clip(obs, OBS_LOW, OBS_HIGH)

    def _filter_reward(self, terms: RewardTerms) -> float:
        active = set(self._stage_cfg.active_reward_terms)
        if not active:
            return terms.total
        d = terms.as_dict()
        return sum(d.get(k, 0.0) for k in active)
