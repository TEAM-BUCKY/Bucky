"""Bucky Lab sweep jobs for the web UI.

Sweeps run in a background thread (which fans out over a process pool, see
:mod:`bucky.lab.runner`). Only one sweep runs at a time and the worker count is capped, so the
production trainer on this machine keeps its CPU. Results stay in memory (last few jobs only).
The UI polls ``GET /api/lab/sweep/{id}`` for progress.
"""
from __future__ import annotations

import asyncio
import logging
import threading
import time
import uuid
from collections import OrderedDict
from typing import Any

from bucky.lab.runner import default_workers, run_sweep

log = logging.getLogger(__name__)

_KEEP = 5


class LabJobs:
    def __init__(self) -> None:
        self._jobs: OrderedDict[str, dict[str, Any]] = OrderedDict()
        self._lock = threading.Lock()

    def running(self) -> dict | None:
        with self._lock:
            return next((j for j in self._jobs.values() if j["state"] == "running"), None)

    def start(self, req: dict, actor: str) -> dict:
        if self.running():
            raise RuntimeError("a lab sweep is already running")
        job_id = uuid.uuid4().hex[:12]
        job = {"id": job_id, "state": "running", "done": 0, "total": 0, "error": None,
               "result": None, "request": req, "actor": actor, "started": time.time()}
        with self._lock:
            self._jobs[job_id] = job
            while len(self._jobs) > _KEEP:
                self._jobs.popitem(last=False)

        def progress(done: int, total: int) -> None:
            job["done"], job["total"] = done, total

        workers = min(int(req.get("workers") or default_workers()), default_workers())

        async def run() -> None:
            try:
                job["result"] = await asyncio.to_thread(
                    run_sweep, req["module"], kind=req["kind"], experiment=req["experiment"],
                    params=req.get("params"), variants=req.get("variants"), grid=req.get("grid"),
                    sensor_params=req.get("sensor_params"), workers=workers, seed=req["seed"],
                    progress=progress,
                )
                job["state"] = "done"
            except Exception as e:  # noqa: BLE001 — user code errors are shown in the UI
                log.exception("lab sweep %s failed", job_id)
                job["state"], job["error"] = "error", f"{type(e).__name__}: {e}"

        job["_task"] = asyncio.get_running_loop().create_task(run())
        log.info("lab sweep %s started by %s: %s/%s", job_id, actor, req["kind"], req["module"])
        return self.view(job_id, include_result=False)

    def view(self, job_id: str, include_result: bool = True) -> dict | None:
        job = self._jobs.get(job_id)
        if job is None:
            return None
        out = {k: v for k, v in job.items() if not k.startswith("_") and k != "result"}
        if include_result and job["state"] == "done":
            out["result"] = job["result"]
        return out
