"""Bucky Lab HTTP routes: module listing, a sweep job round-trip, and replay."""
import time

from fastapi import FastAPI
from fastapi.testclient import TestClient

from app.auth import require_control
from app.broadcast import Broadcaster
from app.config import settings
from app.jobs import JobManager
from app.routes import build_router
from app.users import UserStore


def _client(tmp_path):
    bc = Broadcaster()
    manager = JobManager(bc, settings, base_dir=str(tmp_path))
    app = FastAPI()
    app.state.user_store = UserStore(manager.db, session_ttl_days=7)
    app.state.oauth = None
    app.include_router(build_router(manager, bc))
    app.dependency_overrides[require_control] = lambda: "tester"
    return TestClient(app)


def test_lab_modules_and_experiments(tmp_path):
    c = _client(tmp_path)
    mods = c.get("/api/lab/modules", params={"kind": "drive"}).json()
    names = [m["name"] for m in mods["modules"]]
    assert "bisector" in names
    bis = next(m for m in mods["modules"] if m["name"] == "bisector")
    assert "behind_dist" in bis["params"] and "speed" in bis["params"]
    assert "def target" in bis["source"]
    assert {s["name"] for s in mods["sensors"]} >= {"ball", "compass", "sonar", "line"}
    exps = c.get("/api/lab/experiments").json()["experiments"]
    assert any(e["name"] == "drive_approach" for e in exps)


def test_lab_sweep_and_replay(tmp_path):
    with _client(tmp_path) as c:
        grid = {"ball_step_cm": 60, "ring_radii_cm": [50], "ring_angles": 4}
        r = c.post("/api/lab/sweep", json={"module": "bisector", "grid": grid, "workers": 1})
        assert r.status_code == 200, r.text
        job_id = r.json()["id"]
        for _ in range(200):
            view = c.get(f"/api/lab/sweep/{job_id}").json()
            if view["state"] != "running":
                break
            time.sleep(0.05)
        assert view["state"] == "done", view
        rec = view["result"]["variants"][0]["records"][0]
        rep = c.post("/api/lab/replay", json={"module": "bisector", "scenario": rec["scenario"],
                                              "grid": grid})
        assert rep.status_code == 200, rep.text
        assert rep.json()["metrics"] == rec["metrics"]
        bad = c.post("/api/lab/replay", json={"module": "nope", "scenario": rec["scenario"]})
        assert bad.status_code == 400
