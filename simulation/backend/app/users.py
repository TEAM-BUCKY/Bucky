"""User + session store, backed by the SQLite :class:`~app.db.Database`.

A *user* mirrors a GitHub account that has proved org membership at least once.
A *session* is a server-issued opaque token (only its SHA-256 hash is stored, like
the device tokens) carried in an httpOnly cookie. Sessions expire and are revocable,
which is what makes "log out" and "kick everyone" real operations rather than just
forgetting a credential client-side.
"""
from __future__ import annotations

import hashlib
import secrets
import time

from .db import Database


def _hash(token: str) -> str:
    return hashlib.sha256(token.encode()).hexdigest()


class UserStore:
    def __init__(self, db: Database, session_ttl_days: float = 14.0) -> None:
        self._db = db
        self._ttl = session_ttl_days * 86400.0

    # ── users ───────────────────────────────────────────────────────────────────
    def upsert_user(self, gh_id: int, login: str, name: str | None, avatar_url: str | None) -> dict:
        """Insert or refresh a user from a successful GitHub login."""
        now = time.time()
        self._db.execute(
            """
            INSERT INTO users (id, login, name, avatar_url, created_at, last_login)
            VALUES (?, ?, ?, ?, ?, ?)
            ON CONFLICT(id) DO UPDATE SET
                login=excluded.login,
                name=excluded.name,
                avatar_url=excluded.avatar_url,
                last_login=excluded.last_login
            """,
            (gh_id, login, name, avatar_url, now, now),
        )
        return self.get_user(gh_id)

    def get_user(self, gh_id: int) -> dict | None:
        row = self._db.query_one("SELECT * FROM users WHERE id=?", (gh_id,))
        return dict(row) if row else None

    # ── sessions ────────────────────────────────────────────────────────────────
    def create_session(self, gh_id: int) -> str:
        """Create a session for a user and return the plaintext token (shown once)."""
        token = secrets.token_urlsafe(32)
        now = time.time()
        self._db.execute(
            "INSERT INTO sessions (token_hash, user_id, created_at, expires_at) VALUES (?,?,?,?)",
            (_hash(token), gh_id, now, now + self._ttl),
        )
        return token

    def user_for_session(self, token: str) -> dict | None:
        """Resolve a session token to its user, or ``None`` if missing/expired."""
        if not token:
            return None
        row = self._db.query_one(
            """
            SELECT u.* FROM sessions s
            JOIN users u ON u.id = s.user_id
            WHERE s.token_hash=? AND s.expires_at > ?
            """,
            (_hash(token), time.time()),
        )
        return dict(row) if row else None

    def delete_session(self, token: str) -> None:
        if token:
            self._db.execute("DELETE FROM sessions WHERE token_hash=?", (_hash(token),))

    def purge_expired(self) -> int:
        cur = self._db.execute("DELETE FROM sessions WHERE expires_at <= ?", (time.time(),))
        return cur.rowcount
