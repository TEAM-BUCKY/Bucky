"""Greedy tabular policy: discretize the obs, pick argmax Q, emit the discrete primitive."""
from __future__ import annotations

import numpy as np
from gymnasium import spaces

from bucky.algos.tabular import Discretizer, load_qtable
from bucky.envs.discrete_wrapper import DISCRETE_ACTIONS


class TabularPolicy:
    def __init__(self, q_table: np.ndarray, disc: Discretizer, obs_dim: int = 43) -> None:
        self._q = q_table
        self._disc = disc
        self.observation_space = spaces.Box(-3.0, 3.0, (obs_dim,), np.float32)

    @classmethod
    def load(cls, path: str) -> "TabularPolicy":
        q, disc = load_qtable(path)
        return cls(q, disc)

    def predict(self, obs, deterministic: bool = True, **_):
        s = self._disc.index(obs)
        action_idx = int(np.argmax(self._q[s]))
        return DISCRETE_ACTIONS[action_idx].copy(), None
