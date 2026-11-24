"""Match-time adapter for the FSM + RL hybrid (``fsm_ppo`` / ``fsm_sac``).

Runs the same Mealy FSM as :class:`bucky.envs.fsm_hybrid_wrapper.FSMHybridObsWrapper` and
feeds the RL model ``[mode one-hot, obs]``. Match observations (43-dim) are sliced to the
width the model was trained on (39 for a single-agent-trained model).
"""
from __future__ import annotations

from typing import Any

import numpy as np

from bucky.controllers.fsm import FSMController
from bucky.envs.fsm_hybrid_wrapper import N_MODES, mode_onehot


class HybridPolicy:
    def __init__(self, rl_model: Any) -> None:
        self._rl = rl_model
        self.fsm = FSMController()
        self.observation_space = getattr(rl_model, "observation_space", None)

    def predict(self, obs, deterministic: bool = True, **_):
        x = np.asarray(obs, dtype=np.float32).reshape(-1)
        state, _ = self.fsm.step_state(x)          # FSM sees the full obs (incl. sonar)
        base_dim = int(self._rl.observation_space.shape[0]) - N_MODES
        xr = np.concatenate([mode_onehot(state), x[:base_dim]])
        action, _ = self._rl.predict(xr, deterministic=deterministic)
        return np.clip(np.asarray(action, dtype=np.float32).reshape(-1), -1.0, 1.0), None
