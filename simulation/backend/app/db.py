"""SQLite storage layer for durable server state.

A single ``bucky.db`` file (kept under ``state/`` so it lands in the persistent
volume) replaces the old flat-JSON stores. Opened in WAL mode with a ``busy_timeout``
so the local worker and guest-device check-ins can write concurrently without the
lost-update races the JSON files had.

The connection is shared across threads (FastAPI runs sync deps and ``to_thread``
work on a pool), so every statement goes through a single lock — SQLite itself
serialises writers, the lock just keeps Python's cursor usage tidy. Schema changes
are applied by :meth:`_migrate`, keyed on ``PRAGMA user_version`` so each migration
runs exactly once.
"""
from __future__ import annotations

import sqlite3
import threading
from pathlib import Path
from typing import Any, Iterable

# Ordered schema migrations. The list index + 1 is the resulting ``user_version``;
# only migrations beyond the current version run, so this is append-only.
_MIGRATIONS: list[str] = [
    # v1 — initial schema.
    """
    CREATE TABLE IF NOT EXISTS users (
        id          INTEGER PRIMARY KEY,         -- GitHub numeric user id
        login       TEXT    NOT NULL UNIQUE,
        name        TEXT,
        avatar_url  TEXT,
        created_at  REAL    NOT NULL,
        last_login  REAL    NOT NULL
    );

    CREATE TABLE IF NOT EXISTS sessions (
        token_hash  TEXT    PRIMARY KEY,
        user_id     INTEGER NOT NULL REFERENCES users(id) ON DELETE CASCADE,
        created_at  REAL    NOT NULL,
        expires_at  REAL    NOT NULL
    );
    CREATE INDEX IF NOT EXISTS idx_sessions_expires ON sessions(expires_at);

    CREATE TABLE IF NOT EXISTS devices (
        id            TEXT  PRIMARY KEY,
        name          TEXT  NOT NULL,
        token_hash    TEXT  NOT NULL,
        created_at    REAL  NOT NULL,
        last_seen     REAL,
        current_jobs  TEXT  NOT NULL DEFAULT '[]'   -- JSON array of run names
    );

    CREATE TABLE IF NOT EXISTS queue (
        id        TEXT    PRIMARY KEY,
        position  INTEGER NOT NULL,                 -- explicit ordering
        seq       INTEGER NOT NULL,                 -- monotonic id counter snapshot
        status    TEXT    NOT NULL,
        target    TEXT,
        data      TEXT    NOT NULL                  -- full item JSON
    );
    CREATE INDEX IF NOT EXISTS idx_queue_position ON queue(position);

    CREATE TABLE IF NOT EXISTS jobs (
        id      INTEGER PRIMARY KEY AUTOINCREMENT,
        ts      REAL    NOT NULL,
        action  TEXT    NOT NULL,                   -- launch | stop | delete | enqueue | ...
        run     TEXT,
        actor   TEXT,                               -- github login, "local-password", device id
        source  TEXT,                               -- server | device | queue
        detail  TEXT
    );
    CREATE INDEX IF NOT EXISTS idx_jobs_ts ON jobs(ts);
    """,
    # v2 — per-device concurrency: admin-set cap + worker-reported capability.
    # max_slots NULL means "no admin override → run at the worker's reported capacity".
    """
    ALTER TABLE devices ADD COLUMN max_slots          INTEGER;  -- admin slider; NULL = use capacity
    ALTER TABLE devices ADD COLUMN reported_cores      INTEGER;  -- worker os.cpu_count()
    ALTER TABLE devices ADD COLUMN reported_capacity   INTEGER;  -- worker recommended concurrency
    """,
]


class Database:
    """Thread-safe handle around one SQLite file with migrations applied on open."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._conn = sqlite3.connect(
            str(self.path),
            check_same_thread=False,   # guarded by self._lock instead
            timeout=30.0,              # wait on a busy DB rather than raising immediately
        )
        self._conn.row_factory = sqlite3.Row
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._conn.execute("PRAGMA busy_timeout=30000")
        self._conn.execute("PRAGMA foreign_keys=ON")
        self._migrate()

    # ── schema ────────────────────────────────────────────────────────────────
    def _migrate(self) -> None:
        with self._lock:
            version = self._conn.execute("PRAGMA user_version").fetchone()[0]
            for i in range(version, len(_MIGRATIONS)):
                self._conn.executescript(_MIGRATIONS[i])
                # user_version takes a literal, not a bound parameter.
                self._conn.execute(f"PRAGMA user_version={i + 1}")
                self._conn.commit()

    # ── statements ──────────────────────────────────────────────────────────────
    def execute(self, sql: str, params: Iterable[Any] = ()) -> sqlite3.Cursor:
        """Run a writing statement and commit. Returns the cursor (lastrowid, etc.)."""
        with self._lock:
            cur = self._conn.execute(sql, tuple(params))
            self._conn.commit()
            return cur

    def executemany(self, sql: str, seq_of_params: Iterable[Iterable[Any]]) -> None:
        with self._lock:
            self._conn.executemany(sql, [tuple(p) for p in seq_of_params])
            self._conn.commit()

    def query(self, sql: str, params: Iterable[Any] = ()) -> list[sqlite3.Row]:
        with self._lock:
            return self._conn.execute(sql, tuple(params)).fetchall()

    def query_one(self, sql: str, params: Iterable[Any] = ()) -> sqlite3.Row | None:
        with self._lock:
            return self._conn.execute(sql, tuple(params)).fetchone()

    def transaction(self):
        """Context manager for a multi-statement atomic write under the lock.

        Usage::

            with db.transaction() as conn:
                conn.execute(...); conn.execute(...)
        """
        return _Transaction(self)

    def close(self) -> None:
        with self._lock:
            self._conn.close()


class _Transaction:
    def __init__(self, db: Database) -> None:
        self._db = db

    def __enter__(self) -> sqlite3.Connection:
        self._db._lock.acquire()
        return self._db._conn

    def __exit__(self, exc_type, exc, tb) -> None:
        try:
            if exc_type is None:
                self._db._conn.commit()
            else:
                self._db._conn.rollback()
        finally:
            self._db._lock.release()
