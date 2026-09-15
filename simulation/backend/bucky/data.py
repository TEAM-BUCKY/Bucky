"""Trajectory dataset collection for imitation / supervised / offline learning.

Roll out an *expert* policy (the classical controller, or a trained champion — anything with the
shared ``predict`` contract) through an env and log transitions. The result feeds:
  * **behavioral cloning / supervised** — ``(obs, action)`` pairs,
  * **offline RL** — full ``(obs, action, reward, next_obs, terminal)`` transitions.

Stored as a flat ``.npz`` (torch-free). A d3rlpy ``MDPDataset`` can be produced on demand from a
:class:`Transitions` via :meth:`Transitions.to_d3rlpy` (imported lazily, so d3rlpy stays optional).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

import numpy as np


@dataclass
class Transitions:
    obs: np.ndarray            # (N, obs_dim) float32
    actions: np.ndarray        # (N, act_dim) float32
    rewards: np.ndarray        # (N,) float32
    next_obs: np.ndarray       # (N, obs_dim) float32
    terminals: np.ndarray      # (N,) bool  — true episode end (goal / out / defective)
    timeouts: np.ndarray       # (N,) bool  — time-limit truncation (NOT a real terminal)

    def __len__(self) -> int:
        return int(self.obs.shape[0])

    def save_npz(self, path: str) -> None:
        np.savez(path, obs=self.obs, actions=self.actions, rewards=self.rewards,
                 next_obs=self.next_obs, terminals=self.terminals, timeouts=self.timeouts)

    @classmethod
    def load_npz(cls, path: str) -> "Transitions":
        d = np.load(path)
        terminals = d["terminals"].astype(bool)
        # Older datasets predate the timeouts array → default to none (no truncation marks).
        timeouts = (d["timeouts"].astype(bool) if "timeouts" in d.files
                    else np.zeros_like(terminals))
        return cls(d["obs"].astype(np.float32), d["actions"].astype(np.float32),
                   d["rewards"].astype(np.float32), d["next_obs"].astype(np.float32),
                   terminals, timeouts)

    def to_d3rlpy(self):
        """Build a d3rlpy ``MDPDataset`` (lazy import — d3rlpy is an optional dependency).

        ``timeouts`` marks time-limit truncations as episode boundaries WITHOUT treating them as
        real terminals — critical because Bucky episodes usually end by truncation, and without
        this d3rlpy would bootstrap each episode's last value from the next episode's reset state.
        """
        from d3rlpy.dataset import MDPDataset

        return MDPDataset(
            observations=self.obs, actions=self.actions, rewards=self.rewards,
            terminals=self.terminals.astype(np.float32),
            timeouts=self.timeouts.astype(np.float32),
        )


def collect(
    policy: Any,
    env: Any,
    n_steps: int,
    *,
    seed: int = 0,
    deterministic: bool = True,
    on_progress: Callable[[int, int], None] | None = None,
) -> Transitions:
    """Roll ``policy`` through ``env`` for ``n_steps`` transitions, resetting on episode end.

    ``policy`` needs only ``predict(obs, deterministic=...) -> (action, _)``; ``env`` is a
    Gymnasium env (single-agent drill, or a self-play env with its opponent already set).
    """
    obs_buf: list = []
    act_buf: list = []
    rew_buf: list = []
    nxt_buf: list = []
    term_buf: list = []
    timeout_buf: list = []

    obs, _ = env.reset(seed=seed)
    steps = 0
    while steps < n_steps:
        action, _ = policy.predict(obs, deterministic=deterministic)
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        nxt, reward, terminated, truncated, _info = env.step(action)
        obs_buf.append(np.asarray(obs, dtype=np.float32))
        act_buf.append(action)
        rew_buf.append(np.float32(reward))
        nxt_buf.append(np.asarray(nxt, dtype=np.float32))
        term_buf.append(bool(terminated))
        # Truncation (step-limit) ends the episode but is NOT a real terminal — kept separate so
        # offline RL treats it as a timeout (no bootstrap across the reset seam). See to_d3rlpy.
        timeout_buf.append(bool(truncated) and not bool(terminated))
        steps += 1
        obs = env.reset()[0] if (terminated or truncated) else nxt
        if on_progress is not None and steps % 1000 == 0:
            on_progress(steps, n_steps)

    return Transitions(
        obs=np.asarray(obs_buf, dtype=np.float32),
        actions=np.asarray(act_buf, dtype=np.float32),
        rewards=np.asarray(rew_buf, dtype=np.float32),
        next_obs=np.asarray(nxt_buf, dtype=np.float32),
        terminals=np.asarray(term_buf, dtype=bool),
        timeouts=np.asarray(timeout_buf, dtype=bool),
    )
