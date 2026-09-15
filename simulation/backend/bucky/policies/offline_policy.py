"""Match-time adapter for d3rlpy offline-RL policies (CQL / IQL / BCQ)."""
from __future__ import annotations

from typing import Any

import numpy as np
from gymnasium import spaces


class D3rlpyPolicy:
    def __init__(self, model: Any) -> None:
        self._model = model
        shape = getattr(model, "observation_shape", None)
        self._dim = int(shape[0]) if shape else None
        self.observation_space = spaces.Box(-3.0, 3.0, (self._dim or 43,), np.float32)

    @classmethod
    def load(cls, path: str, device=False) -> "D3rlpyPolicy":
        from bucky.algos.offline import load_offline
        return cls(load_offline(path, device=device))

    def predict(self, obs, deterministic: bool = True, **_):
        x = np.asarray(obs, dtype=np.float32).reshape(-1)
        if self._dim and x.shape[0] > self._dim:
            x = x[:self._dim]
        action = np.asarray(self._model.predict(x.reshape(1, -1))[0], dtype=np.float32).reshape(-1)
        return np.clip(action, -1.0, 1.0), None
