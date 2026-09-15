"""Match-time adapter for the fuzzy + RL residual hybrid.

Recombines the classical controller's base action with the learned residual exactly as
:class:`bucky.envs.residual_wrapper.ResidualActionWrapper` did during training, so a policy trained
as ``residual_*`` behaves identically in matches/tournaments.
"""
from __future__ import annotations

from typing import Any

import numpy as np

from bucky.controllers import ReactiveController
from bucky.envs.residual_wrapper import RESIDUAL_SCALE


class ResidualPolicyAdapter:
    def __init__(self, rl_model: Any, base: ReactiveController | None = None,
                 scale: float = RESIDUAL_SCALE) -> None:
        self._rl = rl_model
        self._base = base or ReactiveController()
        self._scale = scale
        self.observation_space = getattr(rl_model, "observation_space", None)

    def predict(self, obs, deterministic: bool = True, **_):
        x = np.asarray(obs, dtype=np.float32).reshape(-1)
        base_action, _ = self._base.predict(x, deterministic=True)
        # The learned residual net may be narrower than the match obs → slice to its prefix.
        rl_dim = int(self._rl.observation_space.shape[0])
        xr = x[:rl_dim] if x.shape[0] > rl_dim else x
        residual, _ = self._rl.predict(xr, deterministic=deterministic)
        final = np.clip(
            base_action + self._scale * np.asarray(residual, dtype=np.float32).reshape(-1),
            -1.0, 1.0)
        return final.astype(np.float32), None
