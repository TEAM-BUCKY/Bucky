"""Match-time adapter for scikit-learn supervised / behavioral-cloning policies."""
from __future__ import annotations

from typing import Any

import numpy as np
from gymnasium import spaces


class SklearnPolicy:
    def __init__(self, model: Any) -> None:
        self._model = model
        # The obs width the model was trained on; a wider match obs is sliced to its prefix.
        self._dim = int(getattr(model, "n_features_in_", 0)) or None
        self.observation_space = (
            spaces.Box(-3.0, 3.0, (self._dim,), np.float32) if self._dim else None)

    @classmethod
    def load(cls, path: str) -> "SklearnPolicy":
        from bucky.algos.supervised import load_model
        return cls(load_model(path))

    def predict(self, obs, deterministic: bool = True, **_):
        x = np.asarray(obs, dtype=np.float32).reshape(-1)
        if self._dim and x.shape[0] > self._dim:
            x = x[:self._dim]
        action = np.asarray(self._model.predict(x.reshape(1, -1))[0], dtype=np.float32).reshape(-1)
        return np.clip(action, -1.0, 1.0), None
