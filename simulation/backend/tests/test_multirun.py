"""Multi-run registry, per-node slots, multi-job devices, and remote kill.

These exercise the coordinator bookkeeping without spawning real trainer subprocesses:
the queue/lease/heartbeat/complete path and the cancel signalling are all pure async
state transitions on the JobManager.
"""
import asyncio

import pytest

from app.broadcast import Broadcaster
from app.config import Settings
from app.jobs import JobManager


@pytest.fixture
def manager(tmp_path):
    return JobManager(Broadcaster(), Settings(), base_dir=str(tmp_path))


def _device(manager):
    _record, token = manager.devices.register("test-pc")
    return manager.devices.verify(token)


def _enqueue_train(manager, **cfg):
    cfg.setdefault("mode", "train")
    cfg.setdefault("stage", "APPROACH_STATIC_BALL")
    return asyncio.run(manager.enqueue(cfg, None))["item"]


# ── lease registers / completion clears the active registry ─────────────────────
def test_lease_registers_active_run(manager):
    device_id = _device(manager)
    _enqueue_train(manager)

    job = asyncio.run(manager.lease_for_worker(device_id))
    assert job is not None
    run_name = job["run_name"]

    runs = manager.active_runs_msg()["runs"]
    assert any(r["run_name"] == run_name and r["device"] == device_id for r in runs)
    # The device now reports the run as one of its current jobs.
    assert run_name in manager.devices.public(device_id)["current_jobs"]

    asyncio.run(manager.worker_complete(device_id, run_name, "done"))
    assert manager.active_runs_msg()["runs"] == []
    assert manager.devices.public(device_id)["current_jobs"] == []


def test_device_runs_multiple_jobs(manager):
    """One device leasing two jobs tracks both (multi-slot worker)."""
    device_id = _device(manager)
    _enqueue_train(manager, seed=1)
    _enqueue_train(manager, seed=2)

    j1 = asyncio.run(manager.lease_for_worker(device_id))
    j2 = asyncio.run(manager.lease_for_worker(device_id))
    assert j1 and j2 and j1["run_name"] != j2["run_name"]

    jobs = manager.devices.public(device_id)["current_jobs"]
    assert j1["run_name"] in jobs and j2["run_name"] in jobs
    assert len(manager.active_runs_msg()["runs"]) == 2


# ── remote kill via the heartbeat cancel signal ─────────────────────────────────
def test_remote_kill_signals_via_heartbeat(manager):
    device_id = _device(manager)
    _enqueue_train(manager)
    job = asyncio.run(manager.lease_for_worker(device_id))
    run_name = job["run_name"]

    # Before cancel, a heartbeat renews the lease and reports nothing to stop.
    assert asyncio.run(manager.worker_heartbeat(device_id, run_name))["cancel"] is False

    # Requesting a stop on a remote run flags it (the lease is kept so it can save).
    result = asyncio.run(manager.kill(run_name))
    assert result["ok"] and "cancel" in result["message"]
    entry = next(r for r in manager.active_runs_msg()["runs"] if r["run_name"] == run_name)
    assert entry["cancel_requested"] is True and entry["state"] == "stopping"

    # The worker learns of the stop on its next heartbeat.
    assert asyncio.run(manager.worker_heartbeat(device_id, run_name))["cancel"] is True

    # Completing the (stopped) run frees the slot.
    asyncio.run(manager.worker_complete(device_id, run_name, "stopped"))
    assert manager.active_runs_msg()["runs"] == []


def test_kill_unknown_run(manager):
    assert asyncio.run(manager.kill("does-not-exist"))["ok"] is False


# ── local slots ────────────────────────────────────────────────────────────────
def test_local_slot_cap_rejects_when_full(tmp_path, monkeypatch):
    monkeypatch.setenv("LOCAL_SLOTS", "1")
    mgr = JobManager(Broadcaster(), Settings(), base_dir=str(tmp_path))
    # Simulate a local run already occupying the only slot.
    mgr._active["busy"] = {"run_name": "busy", "device": "server", "state": "running"}

    result = asyncio.run(mgr.launch({"mode": "train", "stage": "APPROACH_STATIC_BALL"}))
    assert result["ok"] is False and "slot" in result["message"].lower()
    assert mgr.active_runs_msg()["local_slots"] == 1
    assert mgr.active_runs_msg()["local_active"] == 1
