"""HTTP + WebSocket routes.

Public (no auth): GET /api/status, GET /api/runs, GET /api/health, WS /api/stream.
Gated (Basic auth): POST /api/jobs, POST /api/jobs/stop.
Internal (token):   WS /api/ingest  (the trainer subprocess pushes frames here).
"""
from __future__ import annotations

import logging
import secrets
from typing import Literal, Optional

from fastapi import APIRouter, Depends, HTTPException, Query, WebSocket, WebSocketDisconnect
from pydantic import BaseModel

from .auth import require_control
from .broadcast import Broadcaster
from .jobs import JobManager

log = logging.getLogger(__name__)


class PolicyRef(BaseModel):
    run: str
    checkpoint: str


class StopCondition(BaseModel):
    # steps: train for N timesteps; duration: N wall-clock seconds; until: epoch seconds.
    kind: Literal["steps", "duration", "until"] = "steps"
    value: float


class LaunchRequest(BaseModel):
    mode: Literal["train", "play"] = "train"
    stage: Optional[str] = None
    timesteps: Optional[int] = None
    n_envs: Optional[int] = None
    seed: int = 0
    domain_rand: Optional[bool] = None
    stop: Optional[StopCondition] = None
    resume_from: Optional[PolicyRef] = None
    policy_a: Optional[PolicyRef] = None
    policy_b: Optional[PolicyRef] = None


class QueueAddRequest(LaunchRequest):
    # Epoch seconds the run should start at/after (null = as soon as the queue reaches it).
    start_at: Optional[float] = None


def build_router(manager: JobManager, broadcaster: Broadcaster) -> APIRouter:
    router = APIRouter(prefix="/api")

    @router.get("/health")
    async def health() -> dict:
        return {"ok": True}

    @router.get("/status")
    async def status() -> dict:
        return manager.status_msg()

    @router.get("/runs")
    async def runs() -> dict:
        return manager.runs_msg()

    @router.get("/queue")
    async def get_queue() -> dict:
        return manager.queue_msg()

    @router.post("/jobs")
    async def create_job(req: LaunchRequest, _user: str = Depends(require_control)) -> dict:
        result = await manager.launch(req.model_dump())
        if not result.get("ok"):
            raise HTTPException(status_code=400, detail=result.get("message", "launch failed"))
        return result

    @router.post("/jobs/stop")
    async def stop_job(_user: str = Depends(require_control)) -> dict:
        return await manager.kill()

    @router.post("/queue")
    async def add_to_queue(req: QueueAddRequest, _user: str = Depends(require_control)) -> dict:
        data = req.model_dump()
        start_at = data.pop("start_at", None)
        result = await manager.enqueue(data, start_at)
        if not result.get("ok"):
            raise HTTPException(status_code=400, detail=result.get("message", "enqueue failed"))
        return result

    @router.delete("/queue/{item_id}")
    async def remove_from_queue(item_id: str, _user: str = Depends(require_control)) -> dict:
        return await manager.remove_queue_item(item_id)

    @router.post("/queue/clear")
    async def clear_the_queue(_user: str = Depends(require_control)) -> dict:
        return await manager.clear_queue()

    @router.websocket("/stream")
    async def stream(ws: WebSocket) -> None:
        """Public, read-only stream: step/metrics/status/heartbeat/runs frames."""
        await broadcaster.register(ws)
        try:
            await broadcaster.send(ws, manager.status_msg())
            await broadcaster.send(ws, manager.runs_msg())
            await broadcaster.send(ws, manager.queue_msg())
            # The stream is read-only; we still read (and ignore) to detect disconnects.
            while True:
                await ws.receive_text()
        except WebSocketDisconnect:
            pass
        except Exception:  # noqa: BLE001 — client dropped
            pass
        finally:
            broadcaster.unregister(ws)

    @router.websocket("/ingest")
    async def ingest(ws: WebSocket, token: str = Query(default="")) -> None:
        """Internal: the trainer subprocess streams frames here (token-gated)."""
        if not secrets.compare_digest(token, manager.ingest_token):
            await ws.close(code=1008)
            return
        await ws.accept()
        await manager.on_ingest_connect()
        try:
            while True:
                raw = await ws.receive_text()
                await manager.handle_ingest_raw(raw)
        except WebSocketDisconnect:
            pass
        except Exception:  # noqa: BLE001
            pass
        finally:
            await manager.on_ingest_disconnect()

    return router
