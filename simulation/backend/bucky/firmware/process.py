"""Run firmware work in a fresh, killable process.

A firmware program that spins without ever reading the clock cannot be interrupted in-process
(the watchdog only flags it, and that process can no longer run firmware). Anything that runs
firmware on behalf of a long-lived process — the web server's replays, sweep workers — goes
through here: a ``spawn``ed worker (never ``fork``: the parent may hold a parked firmware
thread) that is discarded after each task.
"""
from __future__ import annotations

import multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor
from concurrent.futures import TimeoutError as FutureTimeout
from typing import Any, Callable


def firmware_pool(workers: int) -> ProcessPoolExecutor:
    """A pool whose workers each run one task in a fresh process."""
    return ProcessPoolExecutor(max_workers=workers, mp_context=mp.get_context("spawn"),
                               max_tasks_per_child=1)


def run_isolated(fn: Callable[..., Any], *args, timeout: float | None = 120.0, **kwargs) -> Any:
    """``fn(*args, **kwargs)`` in a fresh process; raises TimeoutError (and kills the worker) if
    it takes longer than ``timeout`` seconds. ``fn`` must be importable (module-level)."""
    pool = firmware_pool(1)
    fut = pool.submit(fn, *args, **kwargs)
    try:
        return fut.result(timeout=timeout)
    except FutureTimeout:
        for p in list(getattr(pool, "_processes", {}).values()):
            p.kill()
        raise TimeoutError(f"firmware task did not finish within {timeout} s") from None
    finally:
        pool.shutdown(wait=False, cancel_futures=True)
