"""Per-device registry for distributed guest training.

A *device* is a trusted machine (e.g. the user's local PC) that leases training
jobs from the server's shared queue, trains them locally, and uploads the resulting
checkpoints back here. Each device registers once and receives its own token; the
plaintext token is shown to the admin a single time and only its SHA-256 hash is
persisted, so a leaked ``devices.json`` does not reveal usable credentials. Tokens
are individually revocable.

Persisted to ``state/devices.json`` (same lightweight JSON pattern as the queue).
"""
from __future__ import annotations

import hashlib
import json
import logging
import secrets
import time
from pathlib import Path

log = logging.getLogger(__name__)

# A device is "online" if it has checked in within this window (seconds).
ONLINE_TTL = 30.0


def _hash(token: str) -> str:
    return hashlib.sha256(token.encode()).hexdigest()


class DeviceRegistry:
    def __init__(self, state_dir: Path) -> None:
        self._path = state_dir / "devices.json"
        self._state_dir = state_dir
        self._devices: dict[str, dict] = {}
        self._load()

    # ── persistence ──────────────────────────────────────────────────────────
    def _load(self) -> None:
        try:
            if self._path.is_file():
                data = json.loads(self._path.read_text())
                self._devices = {d["id"]: d for d in data.get("devices", []) if "id" in d}
                # Migrate the old single ``current_job`` field to the ``current_jobs`` list.
                for dev in self._devices.values():
                    if "current_jobs" not in dev:
                        old = dev.pop("current_job", None)
                        dev["current_jobs"] = [old] if old else []
        except Exception:  # noqa: BLE001 — a corrupt registry must not crash startup
            log.warning("Could not read devices %s; starting empty", self._path)
            self._devices = {}

    def _save(self) -> None:
        try:
            self._state_dir.mkdir(parents=True, exist_ok=True)
            self._path.write_text(
                json.dumps({"devices": list(self._devices.values())}, indent=2)
            )
        except Exception as exc:  # noqa: BLE001
            log.warning("Could not persist devices: %s", exc)

    # ── registration ─────────────────────────────────────────────────────────
    def register(self, name: str) -> tuple[dict, str]:
        """Create a device and return ``(public_record, plaintext_token)``.

        The plaintext token is returned only here — afterwards only its hash is kept.
        """
        device_id = secrets.token_urlsafe(8)
        token = secrets.token_urlsafe(24)
        self._devices[device_id] = {
            "id": device_id,
            "name": (name or "device").strip() or "device",
            "token_hash": _hash(token),
            "created_at": time.time(),
            "last_seen": None,
            # A device may train several leased runs at once (its own --slots), so the
            # current job is a list. Kept sorted for stable display.
            "current_jobs": [],
        }
        self._save()
        return self.public(device_id), token

    def revoke(self, device_id: str) -> bool:
        if device_id in self._devices:
            del self._devices[device_id]
            self._save()
            return True
        return False

    def rotate(self, device_id: str) -> str | None:
        """Issue a fresh token for an existing device, returning the plaintext.

        The previous token stops working immediately. Like ``register()``, the
        plaintext is returned only here; only its hash is persisted. ``None`` if
        the device is unknown.
        """
        dev = self._devices.get(device_id)
        if not dev:
            return None
        token = secrets.token_urlsafe(24)
        dev["token_hash"] = _hash(token)
        self._save()
        return token

    # ── auth ─────────────────────────────────────────────────────────────────
    def verify(self, token: str) -> str | None:
        """Return the device id for a valid token, else ``None``."""
        if not token:
            return None
        h = _hash(token)
        for dev in self._devices.values():
            if secrets.compare_digest(dev["token_hash"], h):
                return dev["id"]
        return None

    # ── status ───────────────────────────────────────────────────────────────
    def touch(
        self,
        device_id: str,
        *,
        add_job: str | None = None,
        remove_job: str | None = None,
    ) -> None:
        """Record a check-in (``last_seen``) and optionally add/remove a current job.

        A bare ``touch(id)`` is just a check-in — used on every lease poll and streamed
        frame — so it never churns the job list. ``add_job``/``remove_job`` maintain the
        set of runs a device is currently training (it may hold several at once)."""
        dev = self._devices.get(device_id)
        if not dev:
            return
        dev["last_seen"] = time.time()
        jobs = set(dev.get("current_jobs") or [])
        if add_job:
            jobs.add(add_job)
        if remove_job:
            jobs.discard(remove_job)
        dev["current_jobs"] = sorted(jobs)
        self._save()

    def public(self, device_id: str) -> dict:
        dev = self._devices[device_id]
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
        return [self.public(i) for i in self._devices]
