"""Tabular reinforcement learning: Q-learning and SARSA(λ).

The report ranks these low for continuous soccer, but they're cheap, self-contained, and a
useful comparison point. Continuous obs are discretized to a coarse state (ball bearing +
distance, goal bearing, kick-ready), and actions come from the shared discrete set
(:data:`bucky.envs.discrete_wrapper.DISCRETE_ACTIONS`). The learned Q-table saves to ``.npz``;
:class:`bucky.policies.tabular.TabularPolicy` drives it greedily at match time.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np

from bucky.envs.discrete_wrapper import DISCRETE_ACTIONS, N_DISCRETE_ACTIONS


@dataclass
class Discretizer:
    """Coarse map from an observation to a flat discrete state index."""

    n_bearing: int = 8      # ball bearing (angle) bins
    n_dist: int = 4         # ball distance bins
    n_goal: int = 8         # goal bearing (angle) bins
    dist_max: float = 0.6   # obs[2] (dist / FIELD_DIAG) saturates here

    @property
    def n_states(self) -> int:
        return self.n_bearing * self.n_dist * self.n_goal * 2   # ×2 for kick_ready

    def _abin(self, sin_v: float, cos_v: float, n: int) -> int:
        ang = math.atan2(sin_v, cos_v)            # [-pi, pi]
        frac = (ang + math.pi) / (2.0 * math.pi)  # [0, 1)
        return min(n - 1, max(0, int(frac * n)))

    def index(self, obs) -> int:
        o = np.asarray(obs, dtype=np.float64).reshape(-1)
        b = self._abin(o[0], o[1], self.n_bearing)
        d = min(self.n_dist - 1, max(0, int(float(o[2]) / self.dist_max * self.n_dist)))
        g = self._abin(o[8], o[9], self.n_goal)
        k = 1 if float(o[17]) > 0.5 else 0
        return ((b * self.n_dist + d) * self.n_goal + g) * 2 + k

    def to_dict(self) -> dict:
        return asdict(self)


def _eps_greedy(q_row: np.ndarray, eps: float, rng: np.random.Generator) -> int:
    if rng.random() < eps:
        return int(rng.integers(N_DISCRETE_ACTIONS))
    return int(np.argmax(q_row))


def train_tabular(
    env,
    *,
    algo: str = "q",              # "q" (Q-learning) | "sarsa" (SARSA(λ))
    episodes: int = 3000,
    max_steps: int = 400,
    alpha: float = 0.2,
    gamma: float = 0.99,
    lam: float = 0.9,
    eps_start: float = 1.0,
    eps_end: float = 0.05,
    seed: int = 0,
    disc: Discretizer | None = None,
    on_progress=None,
) -> tuple[np.ndarray, Discretizer]:
    """Train a Q-table on ``env`` (a raw Box(4) Bucky env; actions are the discrete primitives).

    Returns ``(q_table, discretizer)``. ``algo="sarsa"`` uses accumulating eligibility traces.
    """
    disc = disc or Discretizer()
    rng = np.random.default_rng(seed)
    q = np.zeros((disc.n_states, N_DISCRETE_ACTIONS), dtype=np.float64)

    for ep in range(episodes):
        eps = eps_start + (eps_end - eps_start) * min(1.0, ep / max(1, episodes - 1))
        obs, _ = env.reset(seed=seed + ep)
        s = disc.index(obs)
        a = _eps_greedy(q[s], eps, rng)
        elig = np.zeros_like(q) if algo == "sarsa" else None

        for _ in range(max_steps):
            obs2, reward, terminated, truncated, _info = env.step(DISCRETE_ACTIONS[a])
            s2 = disc.index(obs2)
            a2 = _eps_greedy(q[s2], eps, rng)
            done = bool(terminated)
            if algo == "sarsa":
                delta = reward + gamma * q[s2, a2] * (not done) - q[s, a]
                elig[s, a] += 1.0
                q += alpha * delta * elig
                elig *= gamma * lam
            else:  # Q-learning (off-policy target uses the greedy next value)
                target = reward + gamma * float(np.max(q[s2])) * (not done)
                q[s, a] += alpha * (target - q[s, a])
            s, a = s2, a2
            if terminated or truncated:
                break

        if on_progress is not None and (ep + 1) % 100 == 0:
            on_progress(ep + 1, episodes)

    return q, disc


def save_qtable(path: str, q: np.ndarray, disc: Discretizer) -> None:
    np.savez(path, q_table=q, kind=np.array("tabular"), **{f"disc_{k}": np.array(v)
                                                           for k, v in disc.to_dict().items()})


def load_qtable(path: str) -> tuple[np.ndarray, Discretizer]:
    data = np.load(path)
    params: dict = {}
    for key in data.files:
        if key.startswith("disc_"):
            name = key[len("disc_"):]
            params[name] = float(data[key]) if name == "dist_max" else int(data[key])
    return data["q_table"].astype(np.float64), Discretizer(**params)
