"""Devices and queue now persist in SQLite and survive a restart, with one-time
import of the legacy JSON files."""
import json

import pytest

from app.broadcast import Broadcaster
from app.config import Settings
from app.db import Database
from app.devices import DeviceRegistry
from app.jobs import JobManager


# ── devices ─────────────────────────────────────────────────────────────────────
def test_device_survives_reopen(tmp_path):
    db = Database(tmp_path / "bucky.db")
    reg = DeviceRegistry(db)
    record, token = reg.register("lab-pc")
    db.close()

    db2 = Database(tmp_path / "bucky.db")
    reg2 = DeviceRegistry(db2)
    assert reg2.verify(token) == record["id"]
    assert reg2.public(record["id"])["name"] == "lab-pc"
    db2.close()


def test_device_legacy_json_import(tmp_path):
    legacy = {
        "devices": [
            {"id": "d1", "name": "old-pc", "token_hash": "abc",
             "created_at": 1.0, "last_seen": None, "current_jobs": []}
        ]
    }
    path = tmp_path / "devices.json"
    path.write_text(json.dumps(legacy))

    db = Database(tmp_path / "bucky.db")
    reg = DeviceRegistry(db)
    assert reg.import_legacy_json(path) == 1
    assert reg.public("d1")["name"] == "old-pc"
    # Second import is a no-op (table no longer empty).
    assert reg.import_legacy_json(path) == 0
    db.close()


# ── queue ───────────────────────────────────────────────────────────────────────
@pytest.fixture
def manager(tmp_path):
    return JobManager(Broadcaster(), Settings(), base_dir=str(tmp_path))


def test_queue_persists_across_restart(tmp_path):
    import asyncio

    m1 = JobManager(Broadcaster(), Settings(), base_dir=str(tmp_path))
    asyncio.run(m1.enqueue({"mode": "train", "stage": "APPROACH_STATIC_BALL", "seed": 3}, None))
    assert len(m1._queue) == 1
    m1.db.close()

    # New manager over the same base_dir reads the queue back from the DB.
    m2 = JobManager(Broadcaster(), Settings(), base_dir=str(tmp_path))
    assert len(m2._queue) == 1
    assert m2._queue[0]["config"]["seed"] == 3
    m2.db.close()


def test_queue_legacy_json_import(tmp_path):
    state = tmp_path / "state"
    state.mkdir()
    (state / "queue.json").write_text(json.dumps({
        "seq": 5,
        "items": [{"id": "q5", "mode": "train", "status": "pending",
                   "config": {"stage": "APPROACH_STATIC_BALL"}}],
    }))
    m = JobManager(Broadcaster(), Settings(), base_dir=str(tmp_path))
    assert len(m._queue) == 1
    assert m._queue[0]["id"] == "q5"
    assert m._queue_seq == 5
    m.db.close()
