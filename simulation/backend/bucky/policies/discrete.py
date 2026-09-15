"""Adapter that turns a discrete-action policy into the continuous ``predict`` contract.

A value-based model (DQN, QR-DQN, tabular Q) predicts a discrete action *index*; the match
engine and tournament feed the current observation and expect a continuous ``Box(4)`` action.
This adapter projects the observation to the policy's width, predicts the index, and looks up the
continuous primitive in the shared :data:`bucky.envs.discrete_wrapper.DISCRETE_ACTIONS` table.
"""
from __future__ import annotations

from typing import Any

import numpy as np

from bucky.envs.discrete_wrapper import DISCRETE_ACTIONS


class DiscretePolicyAdapter:
    def __init__(self, model: Any, obs_dim: int | None = None) -> None:
        self._model = model
        # Width the policy was trained on; a wider match obs is sliced to its single-agent prefix.
        if obs_dim is None:
            space = getattr(model, "observation_space", None)
            obs_dim = int(space.shape[0]) if space is not None else None
        self._dim = obs_dim

    @property
    def observation_space(self):
        return getattr(self._model, "observation_space", None)

    def predict(self, obs, deterministic: bool = True, **_):
        x = np.asarray(obs, dtype=np.float32).reshape(-1)
        if self._dim is not None and x.shape[0] > self._dim:
            x = x[:self._dim]          # self-play obs → single-agent prefix
        idx, _ = self._model.predict(x, deterministic=deterministic)
        return DISCRETE_ACTIONS[int(idx)].copy(), None
