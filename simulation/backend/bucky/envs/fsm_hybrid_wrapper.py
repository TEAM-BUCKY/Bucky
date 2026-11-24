"""FSM + RL hybrid (PWS §9.2.1): "the FSM decides *what*, the RL decides *how*".

Every step the Mealy FSM of :mod:`bucky.controllers.fsm` is advanced on the raw observation.
Its current state (APPROACH / ATTACK / DEFEND / RECOVER) is given to the RL policy as a one-hot
**prefix** of the observation. The policy then learns how to execute each mode (which path to
drive, when to kick) from the normal reward. Its overall structure stays readable: you can always
see which mode the robot is in.

The one-hot is *prepended* (obs = [mode(4), base_obs]) rather than appended. The curriculum's
phase transfer (``transfer_weights_expand_obs``) copies the leading input columns when the
observation grows from single-agent to self-play (the sonar block is appended), and prepending
keeps the mode and base columns aligned across that transfer.

:class:`bucky.policies.hybrid.HybridPolicy` rebuilds the same input at match time.
"""
from __future__ import annotations

import gymnasium as gym
import numpy as np

from bucky.controllers.fsm import STATES, FSMController

N_MODES = len(STATES)


def mode_onehot(state) -> np.ndarray:
    v = np.zeros(N_MODES, dtype=np.float32)
    v[STATES.index(state)] = 1.0
    return v


class FSMHybridObsWrapper(gym.ObservationWrapper):
    def __init__(self, env: gym.Env) -> None:
        super().__init__(env)
        base = env.observation_space
        self.fsm = FSMController(obs_dim=base.shape[0])
        low = np.concatenate([np.zeros(N_MODES, np.float32), base.low.astype(np.float32)])
        high = np.concatenate([np.ones(N_MODES, np.float32), base.high.astype(np.float32)])
        self.observation_space = gym.spaces.Box(low, high, dtype=np.float32)

    def reset(self, **kwargs):
        self.fsm.reset()
        return super().reset(**kwargs)

    def observation(self, obs):
        state, _ = self.fsm.step_state(obs)
        return np.concatenate([mode_onehot(state), np.asarray(obs, dtype=np.float32)])
