"""Residual-action wrapper: the fuzzy + RL hybrid.

Following Leottau et al. (RoboCup 2014) — a hand-designed base controller handles the bulk of
the skill while a learned policy refines it — the RL agent here outputs a *residual* that is added
to the classical reactive controller's action. The agent only has to learn the correction, which is
easier and safer than learning to play from scratch, and the controller guarantees sane baseline
behaviour throughout training.

The agent's action space stays ``Box(4)`` (the residual). The final applied action is
``clip(controller(obs) + RESIDUAL_SCALE * residual, -1, 1)``. :class:`ResidualActionWrapper` is the
training wrapper; :class:`bucky.policies.residual.ResidualPolicyAdapter` recombines the same way at
match/eval time — both share :data:`RESIDUAL_SCALE`, so keep them in sync.
"""
from __future__ import annotations

import gymnasium as gym
import numpy as np

from bucky.controllers import ReactiveController

# How much authority the learned residual has over the controller's base action.
RESIDUAL_SCALE: float = 0.5


class ResidualActionWrapper(gym.Wrapper):
    """Interpret the agent's action as a residual on the classical controller's action."""

    def __init__(self, env: gym.Env, base: ReactiveController | None = None,
                 scale: float = RESIDUAL_SCALE) -> None:
        super().__init__(env)
        self._base = base or ReactiveController(obs_dim=env.observation_space.shape[0])
        self._scale = scale
        self._obs: np.ndarray | None = None   # last obs → the controller's input this step

    def reset(self, **kwargs):
        obs, info = self.env.reset(**kwargs)
        self._obs = np.asarray(obs, dtype=np.float32)
        return obs, info

    def step(self, residual):
        base_action, _ = self._base.predict(self._obs, deterministic=True)
        residual = np.asarray(residual, dtype=np.float32)
        final = np.clip(base_action + self._scale * residual, -1.0, 1.0)
        obs, reward, terminated, truncated, info = self.env.step(final.astype(np.float32))
        self._obs = np.asarray(obs, dtype=np.float32)
        return obs, reward, terminated, truncated, info
