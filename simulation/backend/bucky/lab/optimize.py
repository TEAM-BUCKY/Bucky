"""Best-fit search over module params marked ``*``.

Give a module param the value ``"*"`` (search the param's own ``min``..``max``) or
``"*lo..hi"`` (search that range) and :func:`run_optimize` looks for the value(s) with the highest
mean ``score`` over every scenario of the experiment. Bool and choice params take a plain ``"*"``
and try every option.

Candidates snap to the param's ``step`` (when it has one), so the search never splits hairs
finer than the param's own resolution. Several params can be starred at once. The search is a coordinate search with shrinking brackets:
each round walks the starred params one at a time, sweeping ``points`` evenly spaced values of
that param (others held at the best values so far, which are always among the candidates), moves
to the best, then narrows that param's bracket to ± one grid step around it. Every candidate is
one variant of a normal :func:`bucky.lab.runner.run_sweep` call, so each line search runs in
parallel and points already scored are never re-run.

The result is an ordinary sweep result (variants ``best`` and ``start``) plus an ``optimize``
block with the best values and every evaluated point.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Callable

import numpy as np

from bucky.lab import registry
from bucky.lab.experiments import get_experiment
from bucky.lab.params import Param
from bucky.lab.runner import ProgressFn, run_sweep

STAR = "*"
OBJECTIVE = "score"
DEFAULT_POINTS = 5
DEFAULT_ROUNDS = 4

StatusFn = Callable[[dict], None]


def is_star(value: Any) -> bool:
    return isinstance(value, str) and value.strip().startswith(STAR)


def starred(params: dict | None) -> list[str]:
    return [k for k, v in (params or {}).items() if is_star(v)]


@dataclass
class Axis:
    name: str
    type: str
    start: Any
    lo: float = 0.0           # full search range (continuous axes)
    hi: float = 0.0
    cur_lo: float = 0.0       # current bracket
    cur_hi: float = 0.0
    choices: list | None = None   # discrete axes (bool / choice)
    step: float | None = None     # the param's own resolution: candidates snap to it

    def value(self, x: float) -> Any:
        if self.step:
            x = min(self.hi, max(self.lo, round(x / self.step) * self.step))
        return int(round(x)) if self.type == "int" else float(f"{x:.6g}")

    def candidates(self, center: Any, points: int) -> list:
        if self.choices is not None:
            return list(self.choices)
        xs = [self.value(x) for x in np.linspace(self.cur_lo, self.cur_hi, points)] + [center]
        return sorted(set(xs))

    def shrink(self, best: Any, points: int) -> None:
        if self.choices is not None:
            return
        step = (self.cur_hi - self.cur_lo) / max(1, points - 1)
        step = max(step, self.step or 0.0, 1.0 if self.type == "int" else 0.0)
        self.cur_lo = max(self.lo, best - step)
        self.cur_hi = min(self.hi, best + step)


def parse_axis(name: str, raw: str, param: Param) -> Axis:
    rest = raw.strip()[len(STAR):].strip()
    t = param.type
    if t in ("bool", "choice"):
        if rest:
            raise ValueError(f"{name}: a {t} param takes a plain '*' (tries every option)")
        choices = [False, True] if t == "bool" else list(param.options)
        return Axis(name, t, param.default, choices=choices)
    if t not in ("int", "float"):
        raise ValueError(f"{name}: can't search a {t} param, only numbers, bools and choices")
    if rest:
        lo_s, sep, hi_s = rest.partition("..")
        try:
            if not sep:
                raise ValueError
            lo, hi = float(lo_s), float(hi_s)
        except ValueError:
            raise ValueError(f"{name}: expected '*' or '*lo..hi', got {raw!r}") from None
    elif param.min is None or param.max is None:
        d = float(param.default) or 1.0
        raise ValueError(f"{name} has no min/max to search; give a range, e.g. "
                         f"*{min(d / 2, d * 2):g}..{max(d / 2, d * 2):g}")
    else:
        lo, hi = float(param.min), float(param.max)
    lo, hi = min(lo, hi), max(lo, hi)
    if hi - lo < 1e-12 or (t == "int" and round(hi) == round(lo)):
        raise ValueError(f"{name}: empty search range {raw!r}")
    ax = Axis(name, t, None, lo=lo, hi=hi, cur_lo=lo, cur_hi=hi,
              step=float(param.step) if param.step else None)
    ax.start = ax.value(min(hi, max(lo, float(param.default))))
    return ax


def _key(point: dict) -> tuple:
    return tuple(sorted(point.items()))


def _label(point: dict) -> str:
    return ", ".join(f"{k}={v}" for k, v in point.items())


def run_optimize(
    module: str,
    *,
    kind: str = "drive",
    experiment: str = "drive_approach",
    params: dict | None = None,
    variants: list[dict] | None = None,
    grid: dict | None = None,
    sensor_params: dict | None = None,
    workers: int | None = None,
    seed: int = 0,
    progress: ProgressFn | None = None,
    status: StatusFn | None = None,
    points: int = DEFAULT_POINTS,
    rounds: int = DEFAULT_ROUNDS,
) -> dict:
    """Find the starred params' values with the best mean ``score`` (see the module doc)."""
    if variants:
        raise ValueError("'*' params can't be combined with variants")
    points, rounds = max(2, int(points)), max(1, int(rounds))
    params = dict(params or {})
    names = starred(params)
    if not names:
        raise ValueError("no params marked '*' to search")
    registry.discover()
    spec = registry.get_module(kind, name=module).params
    unknown = [n for n in names if n not in spec]
    if unknown:
        raise ValueError(f"unknown param(s): {', '.join(unknown)}")
    axes = [parse_axis(n, params[n], spec[n]) for n in names]
    fixed = {k: v for k, v in params.items() if k not in names}

    n_scen = len(get_experiment(experiment)(**(grid or {})).scenarios())
    best = {a.name: a.start for a in axes}
    start_key = _key(best)
    # Upper bound (already-scored points are skipped and taken off as we go).
    total = n_scen * rounds * sum(len(a.candidates(a.start, points)) for a in axes)
    done = 0
    scores: dict[tuple, float] = {}
    history: list[dict] = []
    best_score, best_v, start_v, last = -math.inf, None, None, None
    t0 = time.time()

    def report(round_i: int, axis: str) -> None:
        if status:
            status({"round": round_i, "rounds": rounds, "axis": axis,
                    "evaluations": len(scores), "best": dict(best),
                    "best_score": None if best_v is None else best_score})

    for r in range(rounds):
        for ax in axes:
            todo = []
            for c in ax.candidates(best[ax.name], points):
                p = {**best, ax.name: c}
                if _key(p) in scores:
                    total -= n_scen
                else:
                    todo.append(p)
            report(r + 1, ax.name)
            if todo:
                offset = done
                last = run_sweep(
                    module, kind=kind, experiment=experiment, params=fixed,
                    variants=[{"label": _label(p), "params": p} for p in todo], grid=grid,
                    sensor_params=sensor_params, workers=workers, seed=seed,
                    progress=(lambda d, _t: progress(offset + d, total)) if progress else None,
                )
                done += n_scen * len(todo)
                for p, v in zip(todo, last["variants"]):
                    s = v["overall"].get(OBJECTIVE)
                    if s is None:
                        raise ValueError(f"experiment {experiment!r} reports no {OBJECTIVE!r}")
                    scores[_key(p)] = s
                    history.append({"params": p, "score": s, "overall": v["overall"]})
                    if _key(p) == start_key:
                        start_v = v
                    if s > best_score:   # ties keep the earlier point
                        best_score, best_v, best = s, v, dict(p)
            ax.shrink(best[ax.name], points)
    if progress:
        progress(done, done)
    report(rounds, "")

    def relabel(v: dict, label: str) -> dict:
        # Records carry their variant's label; the UI uses it to find the params to replay with.
        tag = lambda recs: [{**r, "variant": label} for r in recs]
        return {**v, "label": label, "records": tag(v["records"]), "worst": tag(v["worst"])}

    result_variants = [relabel(best_v, "best")]
    if start_v is not None and start_v is not best_v:
        result_variants.append(relabel(start_v, "start"))
    return {
        "module": module, "kind": kind, "experiment": experiment, "grid": last["grid"],
        "n_scenarios": n_scen, "elapsed_s": round(time.time() - t0, 2),
        "variants": result_variants,
        "optimize": {
            "objective": OBJECTIVE, "points": points, "rounds": rounds,
            "best": best, "best_score": best_score,
            "start": {a.name: a.start for a in axes},
            "axes": [{"name": a.name, "type": a.type, "lo": a.lo, "hi": a.hi,
                      "choices": a.choices} for a in axes],
            "history": sorted(history, key=lambda h: -h["score"]),
        },
    }
