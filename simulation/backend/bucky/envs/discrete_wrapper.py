"""Discrete-action wrapper so value-based methods (DQN, tabular Q/SARSA) can drive Bucky.

Bucky's native action is continuous ``Box(4) = [vx, vy, omega, kick]``. Value-based algorithms
need a finite action set, so :data:`DISCRETE_ACTIONS` enumerates a compact, expressive set of
body-frame primitives (8 move directions, curved moves, in-place turns, a forward kick, idle) and
:class:`DiscreteActionWrapper` maps a ``Discrete(N)`` index onto the underlying ``Box(4)``.

The same table is used at match/eval time by :class:`bucky.policies.discrete.DiscretePolicyAdapter`
so a trained discrete policy produces the continuous action the match engine expects.
"""
from __future__ import annotations

import math

import gymnasium as gym
import numpy as np
from gymnasium import spaces


def _build_action_table() -> np.ndarray:
    actions: list[tuple[float, float, float, float]] = []
    # 8 move directions at full speed (body frame), no turn, no kick.
    for k in range(8):
        ang = 2.0 * math.pi * k / 8.0
        actions.append((math.cos(ang), math.sin(ang), 0.0, 0.0))
    # Forward while turning (curve-to-ball), in-place turns.
    actions.append((1.0, 0.0, +0.6, 0.0))   # forward + turn left
    actions.append((1.0, 0.0, -0.6, 0.0))   # forward + turn right
    actions.append((0.0, 0.0, +1.0, 0.0))   # spin left in place
    actions.append((0.0, 0.0, -1.0, 0.0))   # spin right in place
    # Kick primitives + idle.
    actions.append((1.0, 0.0, 0.0, 1.0))     # drive forward and kick
    actions.append((0.0, 0.0, 0.0, 1.0))     # kick in place
    actions.append((0.0, 0.0, 0.0, 0.0))     # idle
    return np.array(actions, dtype=np.float32)


# (N, 4) table shared by the training wrapper and the inference adapter — keep them in sync.
DISCRETE_ACTIONS: np.ndarray = _build_action_table()
N_DISCRETE_ACTIONS: int = int(DISCRETE_ACTIONS.shape[0])


class DiscreteActionWrapper(gym.ActionWrapper):
    """Expose a ``Discrete(N)`` action space over a continuous ``Box(4)`` Bucky env."""

    def __init__(self, env: gym.Env) -> None:
        super().__init__(env)
        self.action_space = spaces.Discrete(N_DISCRETE_ACTIONS)

    def action(self, action):
        return DISCRETE_ACTIONS[int(action)]
