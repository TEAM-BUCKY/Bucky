"""Online play-by-game-code room bookkeeping (no real match subprocess spawned)."""
import asyncio

import pytest

from app.broadcast import Broadcaster
from app.config import Settings
from app.jobs import JobManager


@pytest.fixture
def manager(tmp_path):
    return JobManager(Broadcaster(), Settings(), base_dir=str(tmp_path))


def _make_room(manager, code="TEST", mode="casual", run_name="match"):
    """Register a room directly, bypassing the subprocess launch."""
    import time
    now = time.time()
    room = {
        "code": code, "run_name": run_name, "mode": mode,
        "created_at": now, "last_active": now,
        "slots": {s: {"token": None, "claimed": False, "last_seen": None} for s in ("a", "b")},
    }
    manager._rooms[code] = room
    manager._rooms_by_run[run_name] = code
    return room


def test_create_game_registers_room_and_claims_side(manager, monkeypatch):
    async def fake_launch(cfg, actor=None):
        return {"ok": True, "run_name": "match"}
    monkeypatch.setattr(manager, "launch", fake_launch)

    pol = {"run": "r", "checkpoint": "final_model.zip"}
    res = asyncio.run(manager.create_game(mode="casual", claim_side="a",
                                          policy_a=pol, policy_b=pol))
    assert res["ok"] and res["mode"] == "casual"
    assert res["your_side"] == "a" and res["side_token"]
    code = res["code"]
    assert manager.game_run(code) == "match"
    # The claimed side's token authorises control; a wrong token does not.
    assert manager.verify_game_token(code, "a", res["side_token"]) is True
    assert manager.verify_game_token(code, "a", "nope") is False
    # The other side is still free.
    assert manager.game_state(code)["slots"]["b"]["claimed"] is False


def test_create_game_without_policy_is_rejected(manager, monkeypatch):
    # No checkpoints under the tmp base_dir → no default policy → friendly rejection.
    res = asyncio.run(manager.create_game(mode="casual"))
    assert res["ok"] is False
    assert "model" in res["message"].lower()


def test_join_takes_free_sides_then_spectator(manager):
    _make_room(manager)
    j1 = asyncio.run(manager.join_game("TEST"))
    assert j1["ok"] and j1["your_side"] == "a" and not j1["opponent_present"]
    j2 = asyncio.run(manager.join_game("TEST"))
    assert j2["ok"] and j2["your_side"] == "b" and j2["opponent_present"]
    # Both sides taken → third joiner is a spectator (no token, no side).
    j3 = asyncio.run(manager.join_game("TEST"))
    assert j3["ok"] and j3.get("spectator") is True and "side_token" not in j3


def test_join_unknown_code_404(manager):
    res = asyncio.run(manager.join_game("ZZZZ"))
    assert res["ok"] is False and res["status"] == 404


def test_leave_frees_slot_and_requires_token(manager):
    _make_room(manager)
    j = asyncio.run(manager.join_game("TEST", side="a"))
    token = j["side_token"]
    # Wrong token can't release someone else's slot.
    bad = asyncio.run(manager.leave_game("TEST", "a", "wrong"))
    assert bad["ok"] is False and bad["status"] == 403
    # Correct token frees it; the side can then be re-claimed.
    ok = asyncio.run(manager.leave_game("TEST", "a", token))
    assert ok["ok"] is True
    assert manager.game_state("TEST")["slots"]["a"]["claimed"] is False
    rejoin = asyncio.run(manager.join_game("TEST", side="a"))
    assert rejoin["ok"] and rejoin["your_side"] == "a"


def test_codes_are_unique(manager):
    seen = set()
    for i in range(30):
        _make_room(manager, code=manager._new_game_code(), run_name=f"r{i}")
    assert len(manager._rooms) == 30  # no collisions overwrote an existing room


def test_default_policy_prefers_selfplay_run(manager, tmp_path):
    ckpt = tmp_path / "checkpoints" / "selfplay_run"
    ckpt.mkdir(parents=True)
    (ckpt / "opponent_snapshot.zip").write_bytes(b"x")
    (ckpt / "final_model.zip").write_bytes(b"x")
    # A non-self-play run (no opponent_snapshot) should be ignored.
    other = tmp_path / "checkpoints" / "single_agent"
    other.mkdir(parents=True)
    (other / "best_model.zip").write_bytes(b"x")
    pol = manager._default_policy()
    assert pol == {"run": "selfplay_run", "checkpoint": "final_model.zip"}
