"""Offline reinforcement learning (CQL / IQL / BCQ) via d3rlpy.

Learns a policy from a fixed logged dataset (see :mod:`bucky.data`) with no live environment
interaction — attractive because it avoids unsafe on-robot exploration and needs no simulator, but
algorithmically finicky (distributional shift; conservative value penalties). d3rlpy is an optional
dependency (the ``learners-extra`` group), imported lazily so the core trainer never needs it.
Artifacts save as ``.d3`` and drive matches via :class:`bucky.policies.offline_policy.D3rlpyPolicy`.
"""
from __future__ import annotations

from bucky.data import Transitions

_CONFIGS = {"cql": "CQLConfig", "iql": "IQLConfig", "bcq": "BCQConfig"}


def train_offline(
    transitions: Transitions,
    algo: str = "cql",
    *,
    n_steps: int = 20_000,
    n_steps_per_epoch: int = 1_000,
    seed: int = 0,
    device=False,
):
    """Fit a d3rlpy offline algo on ``transitions``. ``algo`` ∈ {cql, iql, bcq}."""
    import d3rlpy

    if algo not in _CONFIGS:
        raise ValueError(f"unknown offline algo {algo!r} (cql | iql | bcq)")
    d3rlpy.seed(seed)
    dataset = transitions.to_d3rlpy()
    config_cls = getattr(d3rlpy.algos, _CONFIGS[algo])
    model = config_cls().create(device=device)
    model.fit(dataset, n_steps=int(n_steps),
              n_steps_per_epoch=min(int(n_steps_per_epoch), int(n_steps)),
              logger_adapter=d3rlpy.logging.NoopAdapterFactory(), show_progress=False)
    return model


def save_offline(model, path: str) -> None:
    model.save(path)   # d3rlpy's native .d3 bundle (config + weights)


def load_offline(path: str, device=False):
    import d3rlpy
    return d3rlpy.load_learnable(path, device=device)
