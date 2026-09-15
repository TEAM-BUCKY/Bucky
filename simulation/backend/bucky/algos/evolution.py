"""Evolutionary / black-box policy optimization (dependency-free).

The report ranks evolutionary tuning as the highest-ROI "learning" method here. Two black-box
optimizers — a cross-entropy-method / evolution-strategy (``evolve_cem``) and a simple genetic
algorithm (``evolve_ga``) — optimize a real parameter vector against a fitness function. Two
genomes plug in:

  * :class:`ControllerGenome` — the :class:`bucky.controllers.ReactiveParams` vector (the report's
    "tune the classical controller" method). Saves to a ``*.controller.json``.
  * :class:`MLPGenome` — the weights of a small tanh MLP policy (neuroevolution). Saves to the
    numpy-opponent ``.npz`` format, so it loads via the existing adapter and runs on-device.

Both artifacts load through :func:`bucky.policies.load_policy` with no new code. A proper
CMA-ES (via the ``cma`` library) registers alongside these once that dependency is added.
"""
from __future__ import annotations

from typing import Callable

import numpy as np

Fitness = Callable[[np.ndarray], float]


# ── optimizers ──────────────────────────────────────────────────────────────────────────
def evolve_cem(
    fitness: Fitness, dim: int, *, x0=None, sigma=0.3, generations: int = 30,
    popsize: int = 32, elite_frac: float = 0.25, sigma_decay: float = 0.97, seed: int = 0,
    on_gen: Callable[[int, float, float], None] | None = None,
) -> tuple[np.ndarray, float]:
    """Cross-entropy-method / evolution-strategy search. ``sigma`` may be scalar or per-dim.

    Returns ``(best_vector, best_fitness)``. Robust and derivative-free — the practical stand-in
    for CMA-ES until the ``cma`` library is wired in.
    """
    rng = np.random.default_rng(seed)
    mean = np.zeros(dim, dtype=float) if x0 is None else np.asarray(x0, dtype=float).copy()
    sigma = np.full(dim, float(sigma)) if np.isscalar(sigma) else np.asarray(sigma, dtype=float)
    n_elite = max(1, int(popsize * elite_frac))
    best_vec, best_fit = mean.copy(), -np.inf

    for g in range(generations):
        pop = mean + sigma * rng.standard_normal((popsize, dim))
        fits = np.array([fitness(ind) for ind in pop])
        order = np.argsort(fits)[::-1]
        elite = pop[order[:n_elite]]
        mean = elite.mean(axis=0)
        sigma = sigma * sigma_decay + 1e-6
        if fits[order[0]] > best_fit:
            best_fit = float(fits[order[0]])
            best_vec = pop[order[0]].copy()
        if on_gen is not None:
            on_gen(g, float(fits[order[0]]), float(fits.mean()))
    return best_vec, best_fit


def evolve_ga(
    fitness: Fitness, dim: int, *, x0=None, sigma=0.3, generations: int = 30,
    popsize: int = 32, elite: int = 2, tournament: int = 3, mut_rate: float = 0.1, seed: int = 0,
    on_gen: Callable[[int, float, float], None] | None = None,
) -> tuple[np.ndarray, float]:
    """A simple real-coded GA (tournament select, blend crossover, Gaussian mutation)."""
    rng = np.random.default_rng(seed)
    base = np.zeros(dim) if x0 is None else np.asarray(x0, dtype=float)
    pop = base + sigma * rng.standard_normal((popsize, dim))
    best_vec, best_fit = base.copy(), -np.inf

    def _select(fits):
        idx = rng.integers(0, len(pop), tournament)
        return pop[idx[np.argmax(fits[idx])]]

    for g in range(generations):
        fits = np.array([fitness(ind) for ind in pop])
        order = np.argsort(fits)[::-1]
        if fits[order[0]] > best_fit:
            best_fit = float(fits[order[0]])
            best_vec = pop[order[0]].copy()
        new = [pop[order[i]].copy() for i in range(elite)]        # elitism
        while len(new) < popsize:
            pa, pb = _select(fits), _select(fits)
            w = rng.random(dim)
            child = w * pa + (1.0 - w) * pb                        # blend crossover
            mask = rng.random(dim) < mut_rate
            child = child + mask * sigma * rng.standard_normal(dim)  # Gaussian mutation
            new.append(child)
        pop = np.array(new)
        if on_gen is not None:
            on_gen(g, float(fits[order[0]]), float(fits.mean()))
    return best_vec, best_fit


def evolve_cmaes(
    fitness: Fitness, dim: int, *, x0=None, sigma=0.3, generations: int = 30,
    popsize: int | None = None, seed: int = 0,
    on_gen: Callable[[int, float, float], None] | None = None,
) -> tuple[np.ndarray, float]:
    """CMA-ES via the ``cma`` library — adapts a full covariance, the strongest black-box
    optimizer here. ``cma`` minimizes, so fitness is negated internally."""
    import cma

    x0 = np.zeros(dim, dtype=float) if x0 is None else np.asarray(x0, dtype=float)
    sigma0 = float(np.mean(sigma)) if not np.isscalar(sigma) else float(sigma)
    opts: dict = {"seed": int(seed), "verbose": -9, "maxiter": int(generations)}
    if popsize:
        opts["popsize"] = int(popsize)
    es = cma.CMAEvolutionStrategy(list(x0), sigma0, opts)
    best_vec, best_fit = x0.copy(), -np.inf
    g = 0
    while not es.stop():
        sols = es.ask()
        fits = [float(fitness(np.asarray(s, dtype=float))) for s in sols]
        es.tell(sols, [-f for f in fits])          # minimize the negative of fitness
        gi = int(np.argmax(fits))
        if fits[gi] > best_fit:
            best_fit = fits[gi]
            best_vec = np.asarray(sols[gi], dtype=float).copy()
        if on_gen is not None:
            on_gen(g, max(fits), float(np.mean(fits)))
        g += 1
    return best_vec, best_fit


OPTIMIZERS = {"es": evolve_cem, "cem": evolve_cem, "ga": evolve_ga, "cmaes": evolve_cmaes}


# ── genomes ─────────────────────────────────────────────────────────────────────────────
class ControllerGenome:
    """Genome = the reactive controller's parameter vector."""

    def __init__(self):
        from bucky.controllers import ReactiveParams
        self._defaults = ReactiveParams()

    @property
    def dim(self) -> int:
        return len(self._defaults.to_vector())

    def x0(self) -> np.ndarray:
        return self._defaults.to_vector()

    def sigma0(self) -> np.ndarray:
        # Per-dim scale so each knob explores proportionally to its magnitude.
        return 0.25 * np.abs(self._defaults.to_vector()) + 0.05

    def to_policy(self, vec):
        from bucky.controllers import ReactiveController, ReactiveParams
        clipped = np.clip(np.asarray(vec, dtype=float), 0.0, None)   # all knobs are non-negative
        return ReactiveController(ReactiveParams.from_vector(clipped))

    def save(self, vec, path: str) -> None:
        import json

        from bucky.controllers import ReactiveParams
        clipped = np.clip(np.asarray(vec, dtype=float), 0.0, None)
        with open(path, "w") as f:
            json.dump(ReactiveParams.from_vector(clipped).to_dict(), f, indent=2)


class MLPGenome:
    """Genome = the flat weights of a tanh MLP policy (neuroevolution)."""

    def __init__(self, sizes):
        self.sizes = list(sizes)                     # e.g. [obs_dim, 32, act_dim]
        self.shapes = []
        for i in range(len(self.sizes) - 1):
            self.shapes.append((self.sizes[i + 1], self.sizes[i]))   # weight
            self.shapes.append((self.sizes[i + 1],))                 # bias

    @property
    def dim(self) -> int:
        return int(sum(int(np.prod(s)) for s in self.shapes))

    def x0(self, seed: int = 0) -> np.ndarray:
        # Small random init (a reasonable neuroevolution starting point); seeded so different
        # runs start from different genomes.
        return 0.3 * np.random.default_rng(seed).standard_normal(self.dim)

    def _unpack(self, vec):
        arrs, k = [], 0
        for s in self.shapes:
            n = int(np.prod(s))
            arrs.append(np.asarray(vec[k:k + n], dtype=np.float32).reshape(s))
            k += n
        pairs = list(zip(arrs[0::2], arrs[1::2]))    # (W, b) per layer
        hidden, (out_w, out_b) = pairs[:-1], pairs[-1]
        return hidden, out_w, out_b

    def to_policy(self, vec):
        from bucky.selfplay import NumpyOpponent
        hidden, out_w, out_b = self._unpack(vec)
        return NumpyOpponent(hidden, out_w, out_b, activation="tanh", squash=False)

    def save(self, vec, path: str) -> None:
        hidden, out_w, out_b = self._unpack(vec)
        arrays = {"n_hidden": np.array(len(hidden)), "activation": np.array("tanh"),
                  "squash": np.array(False)}
        for i, (w, b) in enumerate(hidden):
            arrays[f"h{i}_W"] = w
            arrays[f"h{i}_b"] = b
        arrays["out_W"] = out_w
        arrays["out_b"] = out_b
        np.savez(path, **arrays)


# ── fitness ─────────────────────────────────────────────────────────────────────────────
def rollout_fitness(policy, env, episodes: int, seed: int, max_steps: int = 400) -> float:
    """Mean episode return of ``policy`` over ``episodes`` on ``env`` (a single-agent drill)."""
    total = 0.0
    for ep in range(episodes):
        obs, _ = env.reset(seed=seed + ep)
        ep_ret = 0.0
        for _ in range(max_steps):
            action, _ = policy.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, _ = env.step(
                np.asarray(action, dtype=np.float32).reshape(-1))
            ep_ret += float(reward)
            if terminated or truncated:
                break
        total += ep_ret
    return total / episodes
