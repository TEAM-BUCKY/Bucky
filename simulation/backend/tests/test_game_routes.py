"""Integration tests for the open-guest play-by-game-code HTTP endpoints."""
from fastapi import FastAPI
from fastapi.testclient import TestClient

from app.broadcast import Broadcaster
from app.config import settings
from app.jobs import JobManager
from app.routes import build_router


def build_app(tmp_path, monkeypatch):
    bc = Broadcaster()
    manager = JobManager(bc, settings, base_dir=str(tmp_path))

    # Don't spawn a real match subprocess; pretend a run launched and a sink is connected.
    async def fake_launch(cfg, actor=None):
        return {"ok": True, "run_name": "match"}

    async def fake_push(run, msg):
        manager.last_control = msg  # capture for assertions
        return {"ok": True}

    monkeypatch.setattr(manager, "launch", fake_launch)
    monkeypatch.setattr(manager, "push_control", fake_push)
    monkeypatch.setattr(manager, "_default_policy",
                        lambda: {"run": "r", "checkpoint": "final_model.zip"})

    app = FastAPI()
    app.include_router(build_router(manager, bc))
    return app, manager


def test_reward_defaults_match_reward_config(tmp_path, monkeypatch):
    # The web UI seeds its reward editor from this endpoint, so it must return exactly the
    # Python RewardConfig defaults (single source of truth — no stale hardcoded UI copy).
    from bucky.rewards import RewardConfig

    app, _ = build_app(tmp_path, monkeypatch)
    client = TestClient(app)

    res = client.get("/api/reward-defaults")
    assert res.status_code == 200
    weights = res.json()["weights"]

    cfg = RewardConfig()
    assert res.json()["profile"] == "pws"
    assert weights == cfg.weight_dict()
    # The five PWS weights (§9.3) + rule penalties, all whole numbers.
    for k in ("w_goal", "w_ball_distance", "w_ball_position", "w_kick_goal", "w_smoothness",
              "w_goal_against", "w_out_of_bounds"):
        assert k in weights and float(weights[k]).is_integer()
    assert "w_front_align" not in weights   # legacy weights are off in the default profile

    # Term metadata lets the UI group the weights; the legacy profile is selectable.
    groups = {t["key"]: t["group"] for t in res.json()["terms"]}
    assert groups["w_goal"] == "pws" and groups["w_goal_against"] == "rule"
    assert set(res.json()["profiles"]) >= {"pws", "legacy"}
    legacy = client.get("/api/reward-defaults", params={"profile": "legacy"}).json()
    assert legacy["profile"] == "legacy" and "w_front_align" in legacy["weights"]
    assert client.get("/api/reward-defaults", params={"profile": "nope"}).status_code == 400


def test_create_join_and_control_flow(tmp_path, monkeypatch):
    app, manager = build_app(tmp_path, monkeypatch)
    client = TestClient(app)

    # Host a game (no auth — open guest play).
    res = client.post("/api/games", json={"mode": "casual", "claim_side": "a"})
    assert res.status_code == 200
    created = res.json()
    code, token_a = created["code"], created["side_token"]
    assert created["your_side"] == "a" and created["mode"] == "casual"

    # A second player joins and takes side B.
    j = client.post(f"/api/games/{code}/join", json={}).json()
    assert j["your_side"] == "b" and j["opponent_present"] is True
    token_b = j["side_token"]

    # Public room view shows both sides claimed, never leaks tokens.
    state = client.get(f"/api/games/{code}").json()
    assert state["slots"]["a"]["claimed"] and state["slots"]["b"]["claimed"]
    assert "token" not in str(state)

    # Each player's token authorises control of their side; the side tag is forwarded.
    ok = client.post(f"/api/games/{code}/control",
                     json={"side": "b", "token": token_b, "action": [1, 0, 0, 0], "mode": "human"})
    assert ok.status_code == 200
    assert manager.last_control == {"side": "b", "action": [1.0, 0.0, 0.0, 0.0], "mode": "human"}

    # A wrong token is rejected.
    bad = client.post(f"/api/games/{code}/control",
                      json={"side": "a", "token": "nope", "action": [0, 0, 0, 0]})
    assert bad.status_code == 403


def test_control_action_is_clipped(tmp_path, monkeypatch):
    app, manager = build_app(tmp_path, monkeypatch)
    client = TestClient(app)
    created = client.post("/api/games", json={"claim_side": "a"}).json()
    code, token = created["code"], created["side_token"]
    client.post(f"/api/games/{code}/control",
                json={"side": "a", "token": token, "action": [5, -5, 0.5, 2]})
    assert manager.last_control["action"] == [1.0, -1.0, 0.5, 1.0]


def test_join_unknown_code_is_404(tmp_path, monkeypatch):
    app, _ = build_app(tmp_path, monkeypatch)
    client = TestClient(app)
    assert client.post("/api/games/ZZZZ/join", json={}).status_code == 404
    assert client.get("/api/games/ZZZZ").status_code == 404
