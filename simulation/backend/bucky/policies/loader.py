"""Load any trained artifact into a ``.predict``-compatible policy.

The match engine (:class:`bucky.match.MatchEngine`) drives a side by calling
``model.predict(obs, deterministic=True) -> (action, _)`` on the current 43-dim self-play
observation. This module resolves a :class:`PolicyRef` to such an object regardless of which
learning algorithm produced it:

  * **SB3 ``.zip``** — loaded with the correct algorithm class (read from the sibling
    ``config.json``/``meta.json`` ``algo`` field, defaulting to ``ppo``). A checkpoint whose
    observation is narrower than the current match obs is wrapped in
    :class:`bucky.selfplay.CompatPolicy` so it still plays.
  * **numpy ``.npz``** — a frozen actor (:class:`bucky.selfplay.NumpyOpponent`), torch-free.

Future artifact kinds (sklearn trees, tabular Q-tables, d3rlpy, evolved weight vectors, the
classical controller) register their own extension/kind handlers here as those stages land.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from bucky.selfplay import (
    MATCH_COMPATIBLE_OBS_DIMS,
    SELF_PLAY_OBS_DIM,
    CompatPolicy,
    load_numpy_opponent,
)


@dataclass(frozen=True)
class PolicyRef:
    """A pointer to a trained policy plus a human label for standings/tables.

    ``path`` is the artifact (``.zip``/``.npz``/…). ``algo`` overrides the algorithm class for
    SB3 zips; when ``None`` it is read from the checkpoint's sidecar config (default ``ppo``).
    """

    path: str
    label: str = ""
    algo: str | None = None

    @property
    def display(self) -> str:
        return self.label or Path(self.path).parent.name or Path(self.path).stem


def _algo_for_checkpoint(path: Path, override: str | None) -> str:
    """Resolve the SB3 algorithm for a ``.zip`` from an explicit override, else the run's
    ``config.json``/``meta.json`` sidecar, else the historical default ``ppo``."""
    if override:
        return override
    run_dir = path.parent
    for name in ("config.json", "meta.json"):
        sidecar = run_dir / name
        if not sidecar.is_file():
            continue
        try:
            data = json.loads(sidecar.read_text())
        except Exception:  # noqa: BLE001 — a corrupt sidecar must not break loading
            continue
        # config.json is a ModelConfig; meta.json nests it under "config".
        cfg = data.get("config", data)
        algo = cfg.get("algo")
        if algo:
            return str(algo)
    return "ppo"


def _load_sb3(path: Path, algo: str | None) -> Any:
    from bucky.algos import registry

    spec = registry.get_spec(_algo_for_checkpoint(path, algo))
    model = spec.load(str(path), device="cpu")
    dim = int(model.observation_space.shape[0])
    # Discrete value-based policies output an action index → wrap so predict() yields the
    # continuous Box(4) primitive the match engine applies (the adapter also slices the obs).
    if spec.action_kind == "discrete":
        from bucky.policies.discrete import DiscretePolicyAdapter
        return DiscretePolicyAdapter(model, dim)
    # Residual hybrid: recombine the learned residual with the classical controller's action.
    if spec.env_wrapper == "residual":
        from bucky.policies.residual import ResidualPolicyAdapter
        return ResidualPolicyAdapter(model)
    # A narrower-obs checkpoint plays via the projection wrapper the match engine expects.
    if dim != SELF_PLAY_OBS_DIM and dim in MATCH_COMPATIBLE_OBS_DIMS:
        return CompatPolicy(model, dim)
    return model


def _load_controller(path: Path | None) -> Any:
    from bucky.controllers import ReactiveController, ReactiveParams

    if path is None:
        return ReactiveController()
    data = json.loads(path.read_text())
    return ReactiveController(ReactiveParams(**data))


def load_policy(ref: PolicyRef | str) -> Any:
    """Resolve a :class:`PolicyRef` (or a bare path string) to a ``.predict``-compatible policy.

    Recognized artifacts: SB3 ``.zip`` (any registered algo), numpy ``.npz``, a hand-coded
    controller (the literal ``"classical"``, or a ``*.controller.json`` parameter file).
    """
    if isinstance(ref, str):
        ref = PolicyRef(ref)
    # Non-file pseudo-artifact: the default classical controller.
    if ref.path == "classical":
        return _load_controller(None)
    path = Path(ref.path)
    if not path.exists():
        raise FileNotFoundError(f"policy artifact not found: {path}")
    if path.name.endswith(".controller.json"):
        return _load_controller(path)
    if path.name.endswith(".qtable.npz"):
        from bucky.policies.tabular import TabularPolicy
        return TabularPolicy.load(str(path))
    suffix = path.suffix.lower()
    if suffix == ".joblib":
        from bucky.policies.sklearn_policy import SklearnPolicy
        return SklearnPolicy.load(str(path))
    if suffix == ".d3":
        from bucky.policies.offline_policy import D3rlpyPolicy
        return D3rlpyPolicy.load(str(path))
    if suffix == ".npz":
        # Several trainers write .npz (frozen actor, tabular Q-table, evolved MLP). Dispatch on
        # the file's contents, not just its name, so a mis-named artifact fails loudly-or-correctly
        # rather than crashing the numpy-opponent loader with a KeyError.
        keys = set(np.load(str(path)).files)
        if "q_table" in keys:
            from bucky.policies.tabular import TabularPolicy
            return TabularPolicy.load(str(path))
        if "n_hidden" in keys:            # frozen actor / evolved MLP (numpy-opponent layout)
            return load_numpy_opponent(str(path))
        raise ValueError(f"unrecognized .npz policy artifact {path.name!r} (keys: {sorted(keys)})")
    if suffix == ".zip":
        return _load_sb3(path, ref.algo)
    raise ValueError(f"unsupported policy artifact {path.name!r} (extension {suffix!r})")
