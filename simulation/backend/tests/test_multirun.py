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
    """One device leasing two jobs tracks both (multi-slot worker).

    The worker reports it can handle 2 concurrent jobs (cores/capacity ride the lease),
    so the server's per-device cap lets both through."""
    device_id = _device(manager)
    _enqueue_train(manager, seed=1)
    _enqueue_train(manager, seed=2)

    j1 = asyncio.run(manager.lease_for_worker(device_id, cores=16, capacity=2))
    j2 = asyncio.run(manager.lease_for_worker(device_id, cores=16, capacity=2))
    assert j1 and j2 and j1["run_name"] != j2["run_name"]

    jobs = manager.devices.public(device_id)["current_jobs"]
    assert j1["run_name"] in jobs and j2["run_name"] in jobs
    assert len(manager.active_runs_msg()["runs"]) == 2


# ── per-device concurrency cap (worker capacity → admin slider) ──────────────────
def test_reported_capacity_is_default_budget(manager):
    """With no admin override, the worker's reported capacity caps concurrency and is
    echoed back to the worker as its slot budget."""
    device_id = _device(manager)
    for s in (1, 2, 3):
        _enqueue_train(manager, seed=s)

    j1 = asyncio.run(manager.lease_for_worker(device_id, cores=16, capacity=2))
    j2 = asyncio.run(manager.lease_for_worker(device_id, cores=16, capacity=2))
    j3 = asyncio.run(manager.lease_for_worker(device_id, cores=16, capacity=2))

    assert j1 and j2 and j3 is None  # capacity 2 holds the third back
    assert j1["max_slots"] == 2      # budget echoed so the worker sizes its loop
    pub = manager.devices.public(device_id)
    assert pub["reported_cores"] == 16 and pub["effective_slots"] == 2


def test_admin_max_slots_caps_leasing(manager):
    """An admin cap of 2 lets two jobs through and holds the third until one finishes."""
    device_id = _device(manager)
    manager.devices.touch(device_id, cores=16, capacity=2)
    assert manager.devices.set_max_slots(device_id, 2) is True
    for s in (1, 2, 3):
        _enqueue_train(manager, seed=s)

    j1 = asyncio.run(manager.lease_for_worker(device_id, cores=16, capacity=2))
    j2 = asyncio.run(manager.lease_for_worker(device_id, cores=16, capacity=2))
    j3 = asyncio.run(manager.lease_for_worker(device_id, cores=16, capacity=2))
    assert j1 and j2 and j3 is None

    # Freeing a slot lets the held job lease.
    asyncio.run(manager.worker_complete(device_id, j1["run_name"], "done"))
    j3b = asyncio.run(manager.lease_for_worker(device_id, cores=16, capacity=2))
    assert j3b is not None


def test_set_max_slots_clamps_and_clears(manager):
    device_id = _device(manager)
    manager.devices.touch(device_id, cores=8, capacity=1)

    manager.devices.set_max_slots(device_id, 99)            # clamp to cores
    assert manager.devices.public(device_id)["max_slots"] == 8
    manager.devices.set_max_slots(device_id, 0)             # clamp up to 1
    assert manager.devices.public(device_id)["max_slots"] == 1

    manager.devices.set_max_slots(device_id, None)          # clear → capacity default
    pub = manager.devices.public(device_id)
    assert pub["max_slots"] is None and pub["effective_slots"] == 1

    assert manager.devices.set_max_slots("nope", 2) is False


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
