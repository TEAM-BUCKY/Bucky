"""Tests for the SQLite storage layer: migrations, durability, concurrency."""
import threading

import pytest

from app.db import Database


@pytest.fixture
def db(tmp_path):
    d = Database(tmp_path / "bucky.db")
    yield d
    d.close()


def test_migrations_create_tables_and_set_version(db):
    names = {
        row["name"]
        for row in db.query("SELECT name FROM sqlite_master WHERE type='table'")
    }
    assert {"users", "sessions", "devices", "queue", "jobs"} <= names
    assert db.query_one("PRAGMA user_version")[0] >= 1


def test_migrations_are_idempotent(tmp_path):
    path = tmp_path / "bucky.db"
    a = Database(path)
    v = a.query_one("PRAGMA user_version")[0]
    a.close()
    # Re-opening an already-migrated DB must not error or change the version.
    b = Database(path)
    assert b.query_one("PRAGMA user_version")[0] == v
    b.close()


def test_data_persists_across_reopen(tmp_path):
    path = tmp_path / "bucky.db"
    a = Database(path)
    a.execute(
        "INSERT INTO users (id, login, name, created_at, last_login) VALUES (?,?,?,?,?)",
        (1, "koen", "Koen", 1.0, 1.0),
    )
    a.close()
    b = Database(path)
    assert b.query_one("SELECT login FROM users WHERE id=1")["login"] == "koen"
    b.close()


def test_concurrent_writes_do_not_clobber(db):
    """WAL + busy_timeout: many threads inserting must all land (no lost writes)."""

    def worker(n):
        for i in range(20):
            db.execute(
                "INSERT INTO jobs (ts, action, run, actor, source) VALUES (?,?,?,?,?)",
                (float(i), "launch", f"run-{n}-{i}", "tester", "test"),
            )

    threads = [threading.Thread(target=worker, args=(n,)) for n in range(5)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert db.query_one("SELECT COUNT(*) AS c FROM jobs")["c"] == 100


def test_row_factory_returns_mappings(db):
    db.execute(
        "INSERT INTO jobs (ts, action, run, actor, source) VALUES (?,?,?,?,?)",
        (1.0, "launch", "r1", "koen", "server"),
    )
    row = db.query_one("SELECT action, run FROM jobs")
    assert row["action"] == "launch"
    assert row["run"] == "r1"
