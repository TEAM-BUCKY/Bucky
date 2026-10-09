"""Experiments: a scenario generator + per-episode metrics + aggregation for one module kind.

To test a new kind of module (e.g. a compass filter), add an experiment that

* declares its grid knobs as ``params`` (rendered by the UI);
* yields JSON-able scenario dicts from :meth:`Experiment.scenarios`;
* runs one scenario on an :class:`~bucky.lab.executor.Executor` in :meth:`run_episode`, returning
  a flat dict of numeric/bool metrics (plus ``score``, higher = better) and optionally a trace;
* registers itself with :func:`register_experiment`.

:meth:`Experiment.aggregate` has a generic default (mean of every metric overall + worst cases);
override it to add experiment-specific breakdowns.
"""
from __future__ import annotations

from typing import Any, ClassVar

import numpy as np

from bucky.lab.executor import Executor
from bucky.lab.params import Configurable, describe_params


class Experiment(Configurable):
    name: ClassVar[str] = ""
    module_kind: ClassVar[str] = ""
    #: Other module kinds that can stand in for ``module_kind`` (e.g. a firmware program that
    #: drives the robot can be scored like a drive module).
    also_accepts: ClassVar[tuple[str, ...]] = ()
    #: How the UI shows this experiment's metrics: dicts with ``key``, ``label``,
    #: ``higher_is_better``, ``unit`` ("pct" for 0..1 rates, "deg", "cm", "cm/s", "s", "")
    #: and optional ``domain`` [lo, hi] / ``summary`` (show as a column). Empty → the UI's
    #: built-in drive metrics. A string ``note`` in a record's metrics is shown as "why".
    metric_specs: ClassVar[list[dict]] = []

    @classmethod
    def accepts(cls, kind: str) -> bool:
        return kind == cls.module_kind or kind in cls.also_accepts

    def scenarios(self) -> list[dict]:
        raise NotImplementedError

    def run_episode(self, ex: Executor, scenario: dict, record: bool = False) -> dict:
        raise NotImplementedError

    def aggregate(self, records: list[dict], worst_n: int = 25) -> dict:
        worst = sorted(records, key=lambda r: r["metrics"].get("score", 0.0))[:worst_n]
        return {"overall": summarize_metrics([r["metrics"] for r in records]), "worst": worst}


def summarize_metrics(metrics: list[dict]) -> dict[str, Any]:
    """Mean of every numeric/bool metric (bools become rates), plus ``n``."""
    out: dict[str, Any] = {"n": len(metrics)}
    if not metrics:
        return out
    for key in metrics[0]:
        vals = [m[key] for m in metrics if m.get(key) is not None]
        if vals and all(isinstance(v, (bool, int, float, np.floating)) for v in vals):
            out[key] = float(np.mean(vals))
    return out


_EXPERIMENTS: dict[str, type[Experiment]] = {}


def register_experiment(cls: type[Experiment]) -> type[Experiment]:
    if not cls.name or not cls.module_kind:
        raise ValueError(f"{cls.__name__} needs `name` and `module_kind`")
    _EXPERIMENTS[cls.name] = cls
    return cls


def get_experiment(name: str) -> type[Experiment]:
    try:
        return _EXPERIMENTS[name]
    except KeyError:
        raise KeyError(f"unknown experiment {name!r}; known: {sorted(_EXPERIMENTS)}") from None


def list_experiments(kind: str | None = None) -> list[dict]:
    return [
        {"name": n, "module_kind": c.module_kind,
         "accepts_kinds": [c.module_kind, *c.also_accepts], "metrics": c.metric_specs,
         "doc": (c.__doc__ or "").strip(),
         "params": describe_params(c.params)}
        for n, c in sorted(_EXPERIMENTS.items())
        if kind is None or c.accepts(kind)
    ]
