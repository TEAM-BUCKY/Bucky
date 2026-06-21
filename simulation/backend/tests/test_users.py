"""Tests for the user + session store."""

import pytest

from app.db import Database
from app.users import UserStore


@pytest.fixture
def store(tmp_path):
    db = Database(tmp_path / "bucky.db")
    yield UserStore(db, session_ttl_days=7)
    db.close()


def test_upsert_creates_then_refreshes(store):
    u = store.upsert_user(42, "koen", "Koen", "http://avatar")
    assert u["login"] == "koen"
    # Re-login with a renamed account updates in place (no duplicate row).
    u2 = store.upsert_user(42, "koen1711", "Koen V", "http://avatar2")
    assert u2["id"] == 42
    assert u2["login"] == "koen1711"


def test_session_round_trip(store):
    store.upsert_user(1, "a", None, None)
    token = store.create_session(1)
    assert store.user_for_session(token)["login"] == "a"


def test_session_logout_revokes(store):
    store.upsert_user(1, "a", None, None)
    token = store.create_session(1)
    store.delete_session(token)
    assert store.user_for_session(token) is None


def test_expired_session_is_rejected_and_purged(tmp_path):
    db = Database(tmp_path / "bucky.db")
    store = UserStore(db, session_ttl_days=-1)  # already-expired sessions
    store.upsert_user(1, "a", None, None)
    token = store.create_session(1)
    assert store.user_for_session(token) is None
    assert store.purge_expired() == 1
    db.close()


def test_unknown_token_is_none(store):
    assert store.user_for_session("nope") is None
    assert store.user_for_session("") is None
