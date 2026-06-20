"""Runtime configuration, read from environment variables (and an optional .env).

The frontend is public; control actions (launching/stopping jobs) are gated behind
``APP_USERNAME`` / ``APP_PASSWORD``. The trainer subprocess reaches the internal
ingest endpoint with a shared ``INGEST_TOKEN`` (generated per-process if unset).
"""
from __future__ import annotations

import os
import secrets
from dataclasses import dataclass, field

try:  # convenience for local dev: load a .env in the working dir if present
    from dotenv import load_dotenv

    load_dotenv()
except Exception:  # noqa: BLE001 — dotenv is optional
    pass


def _origins(raw: str) -> list[str]:
    raw = (raw or "").strip()
    if not raw or raw == "*":
        return ["*"]
    return [o.strip() for o in raw.split(",") if o.strip()]


@dataclass
class Settings:
    # Credentials that gate control endpoints. Control is disabled until a password is set.
    username: str = os.getenv("APP_USERNAME", "admin")
    password: str = os.getenv("APP_PASSWORD", "")
    # Shared secret the trainer subprocess uses to reach /api/ingest (localhost only).
    ingest_token: str = os.getenv("INGEST_TOKEN") or secrets.token_urlsafe(24)
    # Port uvicorn listens on — the trainer connects to ws://localhost:PORT/api/ingest.
    port: int = int(os.getenv("PORT", "8000"))
    # CORS origins for the public stream/read endpoints (only needed for split-origin dev).
    cors_origins: list[str] = field(
        default_factory=lambda: _origins(os.getenv("CORS_ORIGINS", "*"))
    )
    # When true, the server runs its own in-process worker: it leases queued jobs and
    # trains them locally (today's single-machine behaviour). Disable to make the
    # server a pure coordinator/store that only registered guest devices train for.
    enable_local_worker: bool = os.getenv("ENABLE_LOCAL_WORKER", "1") not in ("0", "false", "False")

    @property
    def control_enabled(self) -> bool:
        return bool(self.password)


settings = Settings()
