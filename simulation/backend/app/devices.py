"""Per-device registry for distributed guest training.

A *device* is a trusted machine (e.g. the user's local PC) that leases training
jobs from the server's shared queue, trains them locally, and uploads the resulting
checkpoints back here. Each device registers once and receives its own token; the
plaintext token is shown to the admin a single time and only its SHA-256 hash is
persisted, so a leaked registry does not reveal usable credentials. Tokens are
individually revocable.

Persisted to the ``devices`` table of the shared SQLite database (previously a
flat ``state/devices.json`` — :meth:`import_legacy_json` migrates that file once).
"""
from __future__ import annotations

import hashlib
import json
import logging
import secrets
import time
from pathlib import Path

from .db import Database

log = logging.getLogger(__name__)

# A device is "online" if it has checked in within this window (seconds).
ONLINE_TTL = 30.0


def _hash(token: str) -> str:
    return hashlib.sha256(token.encode()).hexdigest()


class DeviceRegistry:
    def __init__(self, db: Database) -> None:
        self._db = db

    # ── persistence helpers ─────────────────────────────────────────────────────
    def _row_to_dict(self, row) -> dict:
        d = dict(row)
        try:
            d["current_jobs"] = json.loads(d.get("current_jobs") or "[]")
        except (TypeError, json.JSONDecodeError):
            d["current_jobs"] = []
        return d

    def _get(self, device_id: str) -> dict | None:
        row = self._db.query_one("SELECT * FROM devices WHERE id=?", (device_id,))
        return self._row_to_dict(row) if row else None

    # ── registration ─────────────────────────────────────────────────────────
    def register(self, name: str) -> tuple[dict, str]:
        """Create a device and return ``(public_record, plaintext_token)``.

        The plaintext token is returned only here — afterwards only its hash is kept.
        """
        device_id = secrets.token_urlsafe(8)
        token = secrets.token_urlsafe(24)
        self._db.execute(
            """
            INSERT INTO devices (id, name, token_hash, created_at, last_seen, current_jobs)
            VALUES (?, ?, ?, ?, NULL, '[]')
            """,
            (device_id, (name or "device").strip() or "device", _hash(token), time.time()),
        )
        return self.public(device_id), token

    def revoke(self, device_id: str) -> bool:
        cur = self._db.execute("DELETE FROM devices WHERE id=?", (device_id,))
        return cur.rowcount > 0

    def rotate(self, device_id: str) -> str | None:
        """Issue a fresh token for an existing device, returning the plaintext.

        The previous token stops working immediately. Like ``register()``, only the
        hash is persisted. ``None`` if the device is unknown.
        """
        if not self._get(device_id):
            return None
        token = secrets.token_urlsafe(24)
        self._db.execute(
            "UPDATE devices SET token_hash=? WHERE id=?", (_hash(token), device_id)
        )
        return token

    # ── auth ─────────────────────────────────────────────────────────────────
    def verify(self, token: str) -> str | None:
        """Return the device id for a valid token, else ``None``."""
        if not token:
            return None
        h = _hash(token)
        # Compare against every stored hash with a constant-time check (the set is tiny).
        for row in self._db.query("SELECT id, token_hash FROM devices"):
            if secrets.compare_digest(row["token_hash"], h):
                return row["id"]
        return None

    # ── status ───────────────────────────────────────────────────────────────
    def touch(
        self,
        device_id: str,
        *,
        add_job: str | None = None,
        remove_job: str | None = None,
    ) -> None:
        """Record a check-in (``last_seen``) and optionally add/remove a current job."""
        dev = self._get(device_id)
        if not dev:
            return
        jobs = set(dev.get("current_jobs") or [])
        if add_job:
            jobs.add(add_job)
        if remove_job:
            jobs.discard(remove_job)
        self._db.execute(
            "UPDATE devices SET last_seen=?, current_jobs=? WHERE id=?",
            (time.time(), json.dumps(sorted(jobs)), device_id),
        )

    def public(self, device_id: str) -> dict:
        dev = self._get(device_id)
        if not dev:
            raise KeyError(device_id)
        last_seen = dev.get("last_seen")
        online = bool(last_seen and (time.time() - last_seen) < ONLINE_TTL)
        jobs = list(dev.get("current_jobs") or [])
        return {
            "id": dev["id"],
            "name": dev["name"],
            "created_at": dev["created_at"],
            "last_seen": last_seen,
            "current_jobs": jobs,
            # Back-compat single field: the first current job (or null when idle).
            "current_job": jobs[0] if jobs else None,
            "online": online,
        }

    def list_public(self) -> list[dict]:
        return [
            self.public(row["id"])
            for row in self._db.query("SELECT id FROM devices ORDER BY created_at")
        ]

    # ── one-time migration ────────────────────────────────────────────────────
    def import_legacy_json(self, path: Path) -> int:
        """Import an old ``devices.json`` once, if the table is empty. Returns count."""
        if self._db.query_one("SELECT COUNT(*) AS c FROM devices")["c"] > 0:
            return 0
        if not path.is_file():
            return 0
        try:
            data = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError):
            return 0
        rows = []
        for d in data.get("devices", []):
            if "id" not in d or "token_hash" not in d:
                continue
            jobs = d.get("current_jobs")
            if jobs is None:
                old = d.get("current_job")
                jobs = [old] if old else []
            rows.append((
                d["id"], d.get("name", "device"), d["token_hash"],
                d.get("created_at") or time.time(), d.get("last_seen"),
                json.dumps(jobs),
            ))
        if rows:
            self._db.executemany(
                "INSERT OR IGNORE INTO devices "
                "(id, name, token_hash, created_at, last_seen, current_jobs) "
                "VALUES (?, ?, ?, ?, ?, ?)",
                rows,
            )
            log.info("Imported %d device(s) from legacy %s", len(rows), path)
        return len(rows)
