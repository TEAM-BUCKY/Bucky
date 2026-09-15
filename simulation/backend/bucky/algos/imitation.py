"""Imitation learning: behavioral cloning (BC) and DAgger.

Both distill an *expert* (the classical controller, or any trained policy — anything with the
shared ``predict`` contract) into a compact supervised policy.

  * **BC** — fit a regressor on the expert's ``(obs, action)`` pairs. Simple, but suffers
    distribution shift: the clone drifts into states the expert never demonstrated.
  * **DAgger** — fixes that by iterating: roll out the current clone, relabel the states *it*
    visits with the expert's action, aggregate, and refit. The expert here is automatic (no human
    in the loop), so this is cheap and sample-efficient.
"""
from __future__ import annotations

from typing import Any, Callable

import numpy as np

from bucky.algos.supervised import fit_supervised
from bucky.data import collect


def behavioral_cloning(dataset, kind: str = "mlp", **kw):
    """Clone the expert demonstrated in ``dataset`` (a :class:`bucky.data.Transitions`)."""
    return fit_supervised(dataset.obs, dataset.actions, kind=kind, **kw)


def dagger(
    expert: Any,
    env: Any,
    *,
    rounds: int = 5,
    steps_per_round: int = 4000,
    kind: str = "mlp",
    seed: int = 0,
    on_round: Callable[[int, int], None] | None = None,
    **kw,
):
    """Train a policy by DAgger against ``expert`` in ``env``. Returns the fitted regressor."""
    from bucky.policies.sklearn_policy import SklearnPolicy

    data = collect(expert, env, steps_per_round, seed=seed)
    obs_agg = [data.obs]
    act_agg = [data.actions]
    model = fit_supervised(data.obs, data.actions, kind=kind, seed=seed, **kw)

    for r in range(1, rounds):
        # Roll out the current clone; then relabel the states IT visited with the expert.
        visited = collect(SklearnPolicy(model), env, steps_per_round, seed=seed + r)
        expert_actions = np.array(
            [np.asarray(expert.predict(o, deterministic=True)[0], dtype=np.float32).reshape(-1)
             for o in visited.obs], dtype=np.float32)
        obs_agg.append(visited.obs)
        act_agg.append(expert_actions)
        model = fit_supervised(np.concatenate(obs_agg), np.concatenate(act_agg),
                               kind=kind, seed=seed, **kw)
        if on_round is not None:
            on_round(r, int(sum(len(o) for o in obs_agg)))
    return model
