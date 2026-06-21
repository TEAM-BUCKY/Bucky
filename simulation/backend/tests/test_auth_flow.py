"""Integration tests for control auth: OAuth session flow and password fallback."""
import base64

from fastapi import FastAPI
from fastapi.testclient import TestClient

from app.broadcast import Broadcaster
from app.config import settings
from app.jobs import JobManager
from app.routes import build_router
from app.users import UserStore


class FakeOAuth:
    """Stand-in for app.oauth.GitHubOAuth with no real network calls."""

    def __init__(self, member=True, user=None):
        self.member = member
        self.user = user or {"id": 1, "login": "koen", "name": "Koen", "avatar_url": "a"}

    def authorize_url(self, redirect_uri, state):
        return f"https://github.com/login/oauth/authorize?state={state}"

    def exchange_code(self, code, redirect_uri):
        return "tok"

    def is_org_member(self, token):
        return self.member

    def fetch_user(self, token):
        return self.user


def build_app(tmp_path, oauth=None):
    bc = Broadcaster()
    manager = JobManager(bc, settings, base_dir=str(tmp_path))
    store = UserStore(manager.db, session_ttl_days=7)
    app = FastAPI()
    app.state.user_store = store
    app.state.oauth = oauth
    app.include_router(build_router(manager, bc))
    return app, manager


def enable_oauth(monkeypatch):
    monkeypatch.setattr(settings, "github_client_id", "cid")
    monkeypatch.setattr(settings, "github_client_secret", "secret")
    monkeypatch.setattr(settings, "github_org", "bucky")


def disable_oauth_use_password(monkeypatch, password="pw"):
    monkeypatch.setattr(settings, "github_client_id", "")
    monkeypatch.setattr(settings, "github_client_secret", "")
    monkeypatch.setattr(settings, "github_org", "")
    monkeypatch.setattr(settings, "username", "admin")
    monkeypatch.setattr(settings, "password", password)


# ── OAuth mode ────────────────────────────────────────────────────────────────
def test_oauth_full_login_unlocks_control(tmp_path, monkeypatch):
    enable_oauth(monkeypatch)
    app, _ = build_app(tmp_path, oauth=FakeOAuth(member=True))
    client = TestClient(app, follow_redirects=False)

    # Unauthenticated control is rejected.
    assert client.post("/api/jobs/stop").status_code == 401

    # /login sets a state cookie and redirects to GitHub.
    login = client.get("/api/auth/login")
    assert login.status_code == 307
    assert "github.com" in login.headers["location"]
    state = client.cookies.get("bucky_oauth_state")
    assert state

    # /callback verifies state + membership, sets a session cookie.
    cb = client.get(f"/api/auth/callback?code=abc&state={state}")
    assert cb.status_code == 307
    assert client.cookies.get("bucky_session")

    # Now control is allowed and /me reports the user.
    me = client.get("/api/auth/me").json()
    assert me["oauth"] is True and me["user"]["login"] == "koen"
    assert client.post("/api/jobs/stop").status_code == 200


def test_oauth_rejects_bad_state(tmp_path, monkeypatch):
    enable_oauth(monkeypatch)
    app, _ = build_app(tmp_path, oauth=FakeOAuth(member=True))
    client = TestClient(app, follow_redirects=False)
    client.get("/api/auth/login")
    resp = client.get("/api/auth/callback?code=abc&state=forged")
    assert resp.status_code == 400
    assert client.cookies.get("bucky_session") is None


def test_oauth_non_member_denied(tmp_path, monkeypatch):
    enable_oauth(monkeypatch)
    app, _ = build_app(tmp_path, oauth=FakeOAuth(member=False))
    client = TestClient(app, follow_redirects=False)
    client.get("/api/auth/login")
    state = client.cookies.get("bucky_oauth_state")
    cb = client.get(f"/api/auth/callback?code=abc&state={state}")
    assert cb.status_code == 307
    assert "auth_error=not_member" in cb.headers["location"]
    assert client.cookies.get("bucky_session") is None
    assert client.post("/api/jobs/stop").status_code == 401


def test_oauth_logout_revokes_session(tmp_path, monkeypatch):
    enable_oauth(monkeypatch)
    app, _ = build_app(tmp_path, oauth=FakeOAuth(member=True))
    client = TestClient(app, follow_redirects=False)
    client.get("/api/auth/login")
    state = client.cookies.get("bucky_oauth_state")
    client.get(f"/api/auth/callback?code=abc&state={state}")
    assert client.post("/api/jobs/stop").status_code == 200
    client.post("/api/auth/logout")
    # The cookie may linger client-side but the server session is gone.
    assert client.post("/api/jobs/stop").status_code == 401


def test_oauth_ignores_basic_password(tmp_path, monkeypatch):
    """With OAuth on, a Basic header must NOT grant control (password is removed)."""
    enable_oauth(monkeypatch)
    monkeypatch.setattr(settings, "username", "admin")
    monkeypatch.setattr(settings, "password", "pw")
    app, _ = build_app(tmp_path, oauth=FakeOAuth(member=True))
    client = TestClient(app)
    basic = base64.b64encode(b"admin:pw").decode()
    resp = client.post("/api/jobs/stop", headers={"Authorization": f"Basic {basic}"})
    assert resp.status_code == 401


# ── Password fallback mode ──────────────────────────────────────────────────────
def test_password_mode_basic_auth_works(tmp_path, monkeypatch):
    disable_oauth_use_password(monkeypatch, password="pw")
    app, _ = build_app(tmp_path, oauth=None)
    client = TestClient(app)
    assert client.post("/api/jobs/stop").status_code == 401
    basic = base64.b64encode(b"admin:pw").decode()
    resp = client.post("/api/jobs/stop", headers={"Authorization": f"Basic {basic}"})
    assert resp.status_code == 200
    me = client.get("/api/auth/me").json()
    assert me["oauth"] is False


def test_password_mode_rejects_wrong_password(tmp_path, monkeypatch):
    disable_oauth_use_password(monkeypatch, password="pw")
    app, _ = build_app(tmp_path, oauth=None)
    client = TestClient(app)
    basic = base64.b64encode(b"admin:nope").decode()
    resp = client.post("/api/jobs/stop", headers={"Authorization": f"Basic {basic}"})
    assert resp.status_code == 401


# ── audit ───────────────────────────────────────────────────────────────────────
def test_control_action_is_audited(tmp_path, monkeypatch):
    disable_oauth_use_password(monkeypatch, password="pw")
    app, manager = build_app(tmp_path, oauth=None)
    client = TestClient(app)
    basic = base64.b64encode(b"admin:pw").decode()
    client.post("/api/jobs/stop", headers={"Authorization": f"Basic {basic}"})
    # kill() on an idle server still returns ok but logs nothing; enqueue a job to audit.
    client.post(
        "/api/queue",
        headers={"Authorization": f"Basic {basic}", "Content-Type": "application/json"},
        json={"mode": "train", "stage": "APPROACH_STATIC_BALL", "seed": 1},
    )
    events = manager.recent_activity()
    assert any(e["action"] == "enqueue" and e["actor"] == "admin" for e in events)
