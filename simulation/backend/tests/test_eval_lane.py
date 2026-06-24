"""Evaluation drills run in their own lane, off the training slots.

These exercise the JobManager bookkeeping without spawning a real eval subprocess: ``_spawn``
is faked, so we assert the lane separation (eval never touches ``_active`` / ``local_slots``),
the per-lane cap, and validation — the pure async state transitions of ``launch_eval``.
"""
import asyncio

import pytest

from app.broadcast import Broadcaster
from app.config import Settings
from app.jobs import JobManager


class _FakeProc:
    """Stand-in for a spawned subprocess that reports as still running."""

    def __init__(self) -> None:
        self.pid = 4242
        self.returncode = None

    def poll(self):
        return None


@pytest.fixture
def manager(tmp_path, monkeypatch):
    mgr = JobManager(Broadcaster(), Settings(), base_dir=str(tmp_path))
    monkeypatch.setattr(mgr, "_spawn", lambda args: (_FakeProc(), None))
    return mgr


def _make_ckpt(mgr, run="RunA", ckpt="final_model.zip"):
    d = mgr._base_dir / "checkpoints" / run
    d.mkdir(parents=True, exist_ok=True)
    (d / ckpt).write_bytes(b"x")
    return run, ckpt


def test_eval_runs_in_separate_lane(manager):
    run, ckpt = _make_ckpt(manager)
    res = asyncio.run(manager.launch_eval(
        {"run": run, "checkpoint": ckpt, "stage": "APPROACH_STATIC_BALL", "n_episodes": 2}))
    assert res["ok"] and res["run_name"].startswith("eval_")
    # The eval lives in the eval lane, never the training registry.
    assert res["run_name"] in manager._eval_active
    assert manager._active == {}
    # Training slots are untouched — analysis doesn't eat a trainer slot.
    msg = manager.active_runs_msg()
    assert msg["local_active"] == 0 and msg["runs"] == []
    # eval_status surfaces it without the unpicklable proc handle.
    ev = manager.eval_status_msg()["eval"]
    assert ev and ev["run_name"] == res["run_name"] and "proc" not in ev


def test_eval_not_blocked_by_full_training_slots(tmp_path, monkeypatch):
    monkeypatch.setenv("LOCAL_SLOTS", "1")
    mgr = JobManager(Broadcaster(), Settings(), base_dir=str(tmp_path))
    monkeypatch.setattr(mgr, "_spawn", lambda args: (_FakeProc(), None))
    # The only training slot is busy.
    mgr._active["busy"] = {"run_name": "busy", "device": "server", "state": "running"}
    run, ckpt = _make_ckpt(mgr)
    res = asyncio.run(mgr.launch_eval({"run": run, "checkpoint": ckpt, "stage": "SELF_PLAY_1V1"}))
    assert res["ok"]  # eval launches anyway — different lane


def test_eval_slot_cap_rejects_second(manager):
    run, ckpt = _make_ckpt(manager)
    a = asyncio.run(manager.launch_eval({"run": run, "checkpoint": ckpt, "stage": "AIM_AND_KICK"}))
    b = asyncio.run(manager.launch_eval({"run": run, "checkpoint": ckpt, "stage": "AIM_AND_KICK"}))
    assert a["ok"] and not b["ok"] and "already running" in b["message"].lower()


def test_eval_rejects_bad_stage_and_missing_checkpoint(manager):
    run, ckpt = _make_ckpt(manager)
    bad = asyncio.run(manager.launch_eval({"run": run, "checkpoint": ckpt, "stage": "FULL_TRAINING"}))
    assert not bad["ok"] and "stage" in bad["message"].lower()
    missing = asyncio.run(manager.launch_eval(
        {"run": run, "checkpoint": "nope.zip", "stage": "AIM_AND_KICK"}))
    assert not missing["ok"] and "not found" in missing["message"].lower()


def test_eval_control_no_active_is_rejected(manager):
    # Transport control with nothing running is a clean rejection, not a crash.
    res = asyncio.run(manager.eval_control({"paused": True}))
    assert not res["ok"] and "No active evaluation" in res["message"]


def test_stop_eval(manager, monkeypatch):
    monkeypatch.setattr(manager, "_terminate", lambda proc: None)
    run, ckpt = _make_ckpt(manager)
    res = asyncio.run(manager.launch_eval({"run": run, "checkpoint": ckpt, "stage": "APPROACH_STATIC_BALL"}))
    assert res["ok"]
    stop = asyncio.run(manager.stop_eval())
    assert stop["ok"]
    assert manager._eval_active[res["run_name"]]["state"] == "stopping"
    # Stopping with nothing running is a no-op, not an error.
    manager._eval_active.clear()
    assert asyncio.run(manager.stop_eval())["ok"]
