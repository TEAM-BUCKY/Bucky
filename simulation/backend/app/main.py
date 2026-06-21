"""FastAPI entrypoint.

Run locally:  uvicorn app.main:app --host 0.0.0.0 --port 8000
(run from the backend/ directory so ``bucky`` and ``scripts/`` resolve.)
"""
from __future__ import annotations

import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .broadcast import Broadcaster
from .config import settings
from .jobs import JobManager
from .oauth import GitHubOAuth
from .routes import build_router
from .users import UserStore

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("bucky")

broadcaster = Broadcaster()
manager = JobManager(broadcaster, settings)
# Users + sessions share the manager's SQLite database.
user_store = UserStore(manager.db, settings.session_ttl_days)
oauth = (
    GitHubOAuth(settings.github_client_id, settings.github_client_secret, settings.github_org)
    if settings.oauth_enabled
    else None
)


@asynccontextmanager
async def lifespan(app: FastAPI):
    manager.start_background()
    if settings.oauth_enabled:
        log.info("GitHub OAuth enabled — control is gated by membership of org '%s'.",
                 settings.github_org)
    elif settings.control_enabled:
        log.info("Password control auth enabled (set GITHUB_CLIENT_ID/SECRET/ORG for OAuth).")
    else:
        log.warning("Control disabled: set APP_PASSWORD or configure GitHub OAuth.")
    user_store.purge_expired()
    try:
        yield
    finally:
        await manager.shutdown()


app = FastAPI(title="Bucky", version="0.1.0", lifespan=lifespan)
# Make the user store + OAuth client reachable from the auth dependency and routes.
app.state.user_store = user_store
app.state.oauth = oauth
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    # Cookies are only sent same-origin (behind Traefik); cross-origin dev uses Basic.
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)
app.include_router(build_router(manager, broadcaster))
