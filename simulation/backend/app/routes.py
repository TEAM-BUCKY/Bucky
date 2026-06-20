"""HTTP + WebSocket routes.

Public (no auth): GET /api/status, GET /api/runs, GET /api/health, WS /api/stream.
Gated (Basic auth): POST /api/jobs, POST /api/jobs/stop.
Internal (token):   WS /api/ingest  (the trainer subprocess pushes frames here).
"""
from __future__ import annotations

import logging
import secrets
from typing import Literal, Optional

from fastapi import (
    APIRouter,
    Depends,
    File,
    Form,
    Header,
    HTTPException,
    Query,
    UploadFile,
    WebSocket,
    WebSocketDisconnect,
)
from fastapi.responses import FileResponse, Response
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
    # Where the run executes: None/"any" = any worker, "server" = local in-process
    # worker only, otherwise a specific device id.
    target: Optional[str] = None
    # Per-model config (optional — when omitted, legacy <stage>_seed<N> behaviour).
    name: Optional[str] = None
    version: Optional[str] = None
    hyperparams: Optional[dict] = None
    net_arch: Optional[list[int]] = None
    reward_weights: Optional[dict] = None
    save_step_checkpoints: Optional[bool] = None


class QueueAddRequest(LaunchRequest):
    # Epoch seconds the run should start at/after (null = as soon as the queue reaches it).
    start_at: Optional[float] = None


class DeviceRegisterRequest(BaseModel):
    name: str = "device"


class WorkerCompleteRequest(BaseModel):
    run_name: str
    status: str = "done"
    timesteps_trained: Optional[int] = None
    best_eval: Optional[float] = None


class WorkerHeartbeatRequest(BaseModel):
    run_name: str


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

    # ── models (admin panel) ────────────────────────────────────────────────
    @router.get("/models")
    async def models() -> dict:
        return manager.models_msg()

    @router.get("/models/{run}/{checkpoint}/download")
    async def download_model(
        run: str, checkpoint: str, _user: str = Depends(require_control)
    ) -> FileResponse:
        path = manager.checkpoint_path(run, checkpoint)
        if path is None:
            raise HTTPException(status_code=404, detail="Checkpoint not found.")
        return FileResponse(
            path, media_type="application/zip", filename=f"{run}_{checkpoint}"
        )

    @router.delete("/models/{run}")
    async def delete_model(run: str, _user: str = Depends(require_control)) -> dict:
        result = await manager.delete_model(run)
        if not result.get("ok"):
            raise HTTPException(status_code=404, detail=result.get("message", "delete failed"))
        return result

    @router.delete("/models/{run}/{checkpoint}")
    async def delete_checkpoint(
        run: str, checkpoint: str, _user: str = Depends(require_control)
    ) -> dict:
        result = await manager.delete_checkpoint(run, checkpoint)
        if not result.get("ok"):
            raise HTTPException(status_code=404, detail=result.get("message", "delete failed"))
        return result

    @router.post("/models/{run}/prune-steps")
    async def prune_steps(run: str, _user: str = Depends(require_control)) -> dict:
        result = await manager.prune_step_checkpoints(run)
        if not result.get("ok"):
            raise HTTPException(status_code=404, detail=result.get("message", "prune failed"))
        return result

    # ── devices (admin-gated registry) ──────────────────────────────────────
    @router.get("/devices")
    async def devices() -> dict:
        return manager.devices_msg()

    @router.post("/devices")
    async def register_device(
        req: DeviceRegisterRequest, _user: str = Depends(require_control)
    ) -> dict:
        record, token = manager.devices.register(req.name)
        await broadcaster.broadcast(manager.devices_msg())
        # The plaintext token is returned exactly once.
        return {"ok": True, "device": record, "token": token}

    @router.delete("/devices/{device_id}")
    async def revoke_device(device_id: str, _user: str = Depends(require_control)) -> dict:
        ok = manager.devices.revoke(device_id)
        await broadcaster.broadcast(manager.devices_msg())
        if not ok:
            raise HTTPException(status_code=404, detail="Device not found.")
        return {"ok": True}

    @router.post("/devices/{device_id}/token")
    async def rotate_device_token(
        device_id: str, _user: str = Depends(require_control)
    ) -> dict:
        token = manager.devices.rotate(device_id)
        if token is None:
            raise HTTPException(status_code=404, detail="Device not found.")
        await broadcaster.broadcast(manager.devices_msg())
        # The new plaintext token is returned exactly once; the old one is now dead.
        return {"ok": True, "token": token}

    # ── worker protocol (device-token gated) ────────────────────────────────
    def require_device(x_device_token: str = Header(default="")) -> str:
        device_id = manager.devices.verify(x_device_token)
        if not device_id:
            raise HTTPException(status_code=401, detail="Invalid device token.")
        return device_id

    @router.post("/worker/lease")
    async def worker_lease(device_id: str = Depends(require_device)):
        job = await manager.lease_for_worker(device_id)
        if job is None:
            return Response(status_code=204)  # nothing ready
        return job

    @router.post("/worker/heartbeat")
    async def worker_heartbeat(
        req: WorkerHeartbeatRequest, device_id: str = Depends(require_device)
    ) -> dict:
        return await manager.worker_heartbeat(device_id, req.run_name)

    @router.post("/worker/complete")
    async def worker_complete(
        req: WorkerCompleteRequest, device_id: str = Depends(require_device)
    ) -> dict:
        return await manager.worker_complete(
            device_id, req.run_name, req.status, req.timesteps_trained, req.best_eval
        )

    @router.post("/models/{run}/upload")
    async def upload_checkpoint(
        run: str,
        file: UploadFile = File(...),
        filename: str = Form(...),
        device_id: str = Depends(require_device),
    ) -> dict:
        data = await file.read()
        result = await manager.save_uploaded_checkpoint(device_id, run, filename, data)
        if not result.get("ok"):
            raise HTTPException(status_code=400, detail=result.get("message", "upload failed"))
        return result

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
            await broadcaster.send(ws, manager.models_msg())
            await broadcaster.send(ws, manager.devices_msg())
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
    async def ingest(
        ws: WebSocket,
        token: str = Query(default=""),
        device: str = Query(default=""),
        run: str = Query(default=""),
    ) -> None:
        """Trainer → hub frame stream.

        The in-process local trainer authenticates with the shared ``INGEST_TOKEN``.
        Remote guest workers authenticate with their device token and pass ``device``
        + ``run`` so their frames are tagged for the browser's per-run selector.

        The token is read from the ``X-Device-Token`` handshake header when present
        (so it doesn't leak into proxy/access logs via the query string); the ``token``
        query param remains a fallback for the loopback local trainer."""
        header_token = ws.headers.get("x-device-token", "")
        effective = header_token or token
        is_local = secrets.compare_digest(effective, manager.ingest_token)
        device_id = None if is_local else manager.devices.verify(effective)
        if not is_local and not device_id:
            await ws.close(code=1008)
            return
        await ws.accept()
        src = "server" if is_local else device_id
        if is_local:
            await manager.on_ingest_connect()
        try:
            while True:
                raw = await ws.receive_text()
                await manager.handle_ingest_raw(raw, device=src, run=run)
        except WebSocketDisconnect:
            pass
        except Exception:  # noqa: BLE001
            pass
        finally:
            if is_local:
                await manager.on_ingest_disconnect()

    return router
