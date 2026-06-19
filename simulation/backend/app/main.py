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
from .routes import build_router

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("bucky")

broadcaster = Broadcaster()
manager = JobManager(broadcaster, settings)


@asynccontextmanager
async def lifespan(app: FastAPI):
    manager.start_background()
    if not settings.control_enabled:
        log.warning("APP_PASSWORD not set — control endpoints are disabled until you set it.")
    try:
        yield
    finally:
        await manager.shutdown()


app = FastAPI(title="Bucky", version="0.1.0", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)
app.include_router(build_router(manager, broadcaster))
