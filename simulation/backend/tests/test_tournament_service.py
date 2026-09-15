"""Backend tournament service: launch a headless round-robin, stream + persist standings."""
import asyncio

import pytest

from app.broadcast import Broadcaster
from app.config import Settings
from app.jobs import JobManager


@pytest.fixture
def manager(tmp_path):
    return JobManager(Broadcaster(), Settings(), base_dir=str(tmp_path))


def test_tournaments_table_created(manager):
    # v3 migration ran → the table exists and is queryable.
    rows = manager.db.query(
        "SELECT name FROM sqlite_master WHERE type='table' AND name='tournaments'")
    assert len(rows) == 1


def test_launch_requires_two_entrants(manager):
    res = asyncio.run(manager.launch_tournament({"entrants": [{"run": "classical"}]}))
    assert res["ok"] is False and "at least 2" in res["message"]


def test_round_robin_of_classical_controllers(manager):
    async def run():
        res = await manager.launch_tournament({
            "entrants": [
                {"run": "classical", "label": "ctrl-a"},
                {"run": "classical", "label": "ctrl-b"},
                {"run": "classical", "label": "ctrl-c"},
            ],
            "matches_per_pairing": 1,
            "max_steps": 150,   # tiny matches so the test is fast
            "seed": 0,
        })
        assert res["ok"] is True
        tid = res["id"]
        # Wait for the background tournament task to finish (bounded).
        for _ in range(600):
            state = manager.get_tournament(tid)
            if state and state["status"] in ("done", "stopped", "error"):
                break
            await asyncio.sleep(0.05)
        return manager.get_tournament(tid)

    state = asyncio.run(run())
    assert state["status"] == "done", state.get("error")
    standings = state["standings"]["standings"]
    assert len(standings) == 3
    # Every entrant played both others once (round-robin over 3 → 2 games each).
    assert all(s["played"] == 2 for s in standings)
    assert state["standings"]["complete"] is True
    # Persisted to the DB (survives a fresh manager over the same base_dir / DB file).
    row = manager.db.query_one("SELECT status FROM tournaments WHERE id=?", (state["id"],))
    assert row["status"] == "done"
