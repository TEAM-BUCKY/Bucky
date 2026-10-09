"""Sweep runner: every scenario of an experiment × one or more param variants of a module.

Work is split into chunks and fanned out over a process pool. Workers re-run
:func:`bucky.lab.registry.discover` and look the module up by ``(kind, name)``, so user modules
never need to be picklable. Default workers = half the CPUs: this machine also runs the
production trainer.

Firmware programs (kind ``"firmware"``, see :mod:`bucky.firmware.lab`) always run in fresh
``spawn``ed worker processes, one chunk each, so a firmware hang can never take down the caller
(e.g. the web server) and every chunk starts from clean firmware globals.
"""
from __future__ import annotations

import os
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from typing import Callable

from bucky.lab import registry
from bucky.lab.executor import Executor
from bucky.lab.experiments import get_experiment

ProgressFn = Callable[[int, int], None]


def default_workers() -> int:
    return max(1, (os.cpu_count() or 2) // 2)


#: Module kinds that run the compiled firmware (bucky.firmware): isolated worker processes.
FIRMWARE_KINDS = ("firmware", "firmware_sensor", "ekf")
FIRMWARE_CHUNK = 16


def build_executor(kind: str, name: str, params: dict | None, sensor_params: dict | None,
                   seed: int) -> Executor:
    module = registry.get_module(kind, name)(**(params or {}))
    if kind == "ekf":
        from bucky.firmware.ekf_bench import EkfBench

        return EkfBench(module, seed=seed)
    if kind in FIRMWARE_KINDS:
        from bucky.firmware.lab import FirmwareExecutor

        return FirmwareExecutor(module, seed=seed)
    return Executor([module], sensor_params=sensor_params, seed=seed)


def _run_chunk(job: dict) -> list[dict]:
    registry.discover()
    exp = get_experiment(job["experiment"])(**job["grid"])
    out = []
    for sc in job["scenarios"]:
        # Seed per scenario so results don't depend on how work was chunked.
        ex = build_executor(job["kind"], job["module"], job["params"], job["sensor_params"],
                            seed=job["seed"] + sc["id"])
        res = exp.run_episode(ex, sc)
        out.append({"variant": job["variant"], "scenario": sc, "metrics": res["metrics"]})
    return out


def run_sweep(
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
    chunk_size: int = 64,
) -> dict:
    """Run the sweep. ``variants`` is a list of ``{"label": str, "params": {...}}`` overrides
    applied on top of ``params``; with no variants a single "base" variant is run."""
    registry.discover()
    module_cls = registry.get_module(kind, name=module)
    exp_cls = get_experiment(experiment)
    if not exp_cls.accepts(kind):
        raise ValueError(f"experiment {experiment!r} tests {exp_cls.module_kind!r} modules")
    firmware = kind in FIRMWARE_KINDS
    if firmware:
        from bucky.firmware.build import ensure_built

        ensure_built()   # once, here, rather than racing in every worker
        chunk_size = min(chunk_size, FIRMWARE_CHUNK)
    exp = exp_cls(**(grid or {}))
    scenarios = exp.scenarios()
    base = dict(params or {})
    variants = variants or [{"label": "base", "params": {}}]
    resolved = []
    for i, v in enumerate(variants):
        p = {**base, **(v.get("params") or {})}
        module_cls(**p)   # validate early, in the parent process
        resolved.append({"label": v.get("label") or f"v{i + 1}", "params": p})

    jobs = [
        {"kind": kind, "module": module, "experiment": experiment, "grid": grid or {},
         "params": v["params"], "sensor_params": sensor_params or {}, "seed": seed,
         "variant": v["label"], "scenarios": scenarios[i:i + chunk_size]}
        for v in resolved
        for i in range(0, len(scenarios), chunk_size)
    ]
    total = len(scenarios) * len(resolved)
    workers = workers or default_workers()
    t0 = time.time()
    records: list[dict] = []
    if not firmware and (workers <= 1 or len(jobs) <= 1):
        for job in jobs:
            records += _run_chunk(job)
            if progress:
                progress(len(records), total)
    else:
        if firmware:
            from bucky.firmware.process import firmware_pool

            pool_cm = firmware_pool(min(workers, len(jobs)))
        else:
            pool_cm = ProcessPoolExecutor(max_workers=workers)
        with pool_cm as pool:
            futures = [pool.submit(_run_chunk, job) for job in jobs]
            for fut in as_completed(futures):
                records += fut.result()
                if progress:
                    progress(len(records), total)

    by_variant = {v["label"]: [] for v in resolved}
    for r in records:
        by_variant[r["variant"]].append(r)
    results = []
    for v in resolved:
        recs = sorted(by_variant[v["label"]], key=lambda r: r["scenario"]["id"])
        results.append({"label": v["label"], "params": v["params"],
                        "records": recs, **exp.aggregate(recs)})
    return {
        "module": module, "kind": kind, "experiment": experiment, "grid": exp.p.__dict__,
        "n_scenarios": len(scenarios), "elapsed_s": round(time.time() - t0, 2),
        "variants": results,
    }


def replay(module: str, scenario: dict, *, kind: str = "drive",
           experiment: str = "drive_approach", params: dict | None = None,
           grid: dict | None = None, sensor_params: dict | None = None, seed: int = 0) -> dict:
    """Re-run one scenario with tracing on (same seed as in the sweep → identical result).
    Firmware programs replay in a fresh, killable process (see :mod:`bucky.firmware.process`)."""
    if kind in FIRMWARE_KINDS:
        from bucky.firmware.build import ensure_built
        from bucky.firmware.process import run_isolated

        ensure_built()
        return run_isolated(_replay_local, module, scenario, kind=kind, experiment=experiment,
                            params=params, grid=grid, sensor_params=sensor_params, seed=seed)
    return _replay_local(module, scenario, kind=kind, experiment=experiment, params=params,
                         grid=grid, sensor_params=sensor_params, seed=seed)


def _replay_local(module: str, scenario: dict, *, kind: str, experiment: str,
                  params: dict | None, grid: dict | None, sensor_params: dict | None,
                  seed: int) -> dict:
    registry.discover()
    exp = get_experiment(experiment)(**(grid or {}))
    ex = build_executor(kind, module, params, sensor_params, seed=seed + int(scenario.get("id", 0)))
    res = exp.run_episode(ex, scenario, record=True)
    return {"scenario": scenario, "dt": ex.physics.dt, **res}
