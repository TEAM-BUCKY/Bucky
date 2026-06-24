"""HTTP + WebSocket routes.

Public (no auth): GET /api/status, GET /api/runs, GET /api/health, WS /api/stream.
Gated (Basic auth): POST /api/jobs, POST /api/jobs/stop.
Internal (token):   WS /api/ingest  (the trainer subprocess pushes frames here).
"""
from __future__ import annotations

import asyncio
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
    Request,
    UploadFile,
    WebSocket,
    WebSocketDisconnect,
)
from fastapi.responses import FileResponse, JSONResponse, RedirectResponse, Response
from pydantic import BaseModel

from .auth import OAUTH_STATE_COOKIE, SESSION_COOKIE, current_user, require_control
from .broadcast import Broadcaster
from .config import settings
from .jobs import JobManager
from .oauth import OAuthError

log = logging.getLogger(__name__)


def _base_url(request: Request) -> str:
    """Public origin for building the OAuth redirect URI.

    Prefer the explicitly-configured PUBLIC_URL (correct behind a TLS proxy where the
    request's own scheme/host may be the internal one); fall back to the request origin
    for local dev."""
    if settings.public_url:
        return settings.public_url
    return str(request.base_url).rstrip("/")


def _cookie_secure(request: Request) -> bool:
    """Only mark cookies Secure over HTTPS, so they still work on http://localhost."""
    return _base_url(request).startswith("https://")


def _public_user(user: dict | None) -> dict | None:
    if not user:
        return None
    return {"login": user["login"], "name": user.get("name"), "avatar_url": user.get("avatar_url")}


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
    # Play mode: let a human drive red (robot A) via the control sink instead of its policy.
    manual_red: Optional[bool] = None
    # Where the run executes: None/"any" = any worker, "server" = local in-process
    # worker only, otherwise a specific device id.
    target: Optional[str] = None
    # Animate the live field in the UI (off by default — headless trains faster).
    viz: Optional[bool] = None
    # Split one run across devices via FedAvg. Either even shards
    # {"shards": N, "sync_every": steps} or a per-device map
    # {"devices": [{"target": id|"server", "n_envs": N}], "sync_every": steps}.
    distributed: Optional[dict] = None
    # FULL_TRAINING only: per-phase budget fractions (APPROACH / PUSH / SELF_PLAY). The trainer
    # normalizes them; None → the default 0.15 / 0.25 / 0.60 split.
    full_training_split: Optional[list[float]] = None
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


class EvalRequest(BaseModel):
    # Run a checkpoint through a drill (curriculum stage), streaming the reward build-up.
    run: str
    checkpoint: str
    stage: str
    n_episodes: int = 10
    seed: int = 999
    deterministic: bool = True


class ControlRequest(BaseModel):
    # Manual control for a play match's red robot. ``run`` is the match run name.
    run: str
    action: Optional[list[float]] = None     # [vx, vy, omega, kick], each in [-1, 1]
    red_mode: Optional[Literal["human", "ai"]] = None


class CreateGameRequest(BaseModel):
    # Online human-vs-human game. Open guest play: no login — the returned per-side token
    # is the credential for sending control. "casual" = endless; "match" = full timed match.
    mode: Literal["casual", "match"] = "casual"
    seed: int = 0
    claim_side: Optional[Literal["a", "b"]] = "a"
    # Optional AI policies for the unclaimed/idle sides; defaults to a server-chosen baseline.
    policy_a: Optional[PolicyRef] = None
    policy_b: Optional[PolicyRef] = None


class JoinGameRequest(BaseModel):
    side: Optional[Literal["a", "b"]] = None   # None → first free side, or spectator if full


class GameControlRequest(BaseModel):
    side: Literal["a", "b"]
    token: str
    action: Optional[list[float]] = None        # [vx, vy, omega, kick], each in [-1, 1]
    mode: Optional[Literal["human", "ai"]] = None


class LeaveGameRequest(BaseModel):
    side: Literal["a", "b"]
    token: str


class DeviceRegisterRequest(BaseModel):
    name: str = "device"


class DeviceSlotsRequest(BaseModel):
    # Admin concurrency cap for a device; null clears it back to the reported capacity.
    max_slots: Optional[int] = None


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

    @router.get("/reward-defaults")
    async def reward_defaults() -> dict:
        """The reward-weight defaults from the Python ``RewardConfig`` dataclass.

        This is the single source of truth for reward weights: the web UI loads these to
        seed its editor so editing ``bucky/rewards.py`` (and restarting the server) updates
        the website's defaults — instead of the UI shipping a stale hardcoded copy that
        would silently override the code values at launch. Field order follows the dataclass.
        """
        from dataclasses import fields as _fields

        from bucky.rewards import RewardConfig

        cfg = RewardConfig()
        return {"weights": {f.name: getattr(cfg, f.name) for f in _fields(cfg)}}

    # ── auth (GitHub-org OAuth, or password fallback) ────────────────────────────
    @router.get("/auth/me")
    async def auth_me(request: Request) -> dict:
        """Login state for the SPA: which auth mode is active and the current user."""
        if not settings.oauth_enabled:
            return {"oauth": False, "user": None, "control_enabled": settings.control_enabled}
        return {
            "oauth": True,
            "org": settings.github_org,
            "user": _public_user(current_user(request)),
            "control_enabled": True,
        }

    @router.get("/auth/login")
    async def auth_login(request: Request) -> RedirectResponse:
        """Begin the GitHub OAuth flow (redirect to GitHub with a CSRF ``state``)."""
        oauth = getattr(request.app.state, "oauth", None)
        if oauth is None:
            raise HTTPException(status_code=404, detail="OAuth is not enabled.")
        state = secrets.token_urlsafe(24)
        redirect_uri = _base_url(request) + "/api/auth/callback"
        resp = RedirectResponse(oauth.authorize_url(redirect_uri, state), status_code=307)
        resp.set_cookie(
            OAUTH_STATE_COOKIE, state, max_age=600, httponly=True,
            secure=_cookie_secure(request), samesite="lax", path="/",
        )
        return resp

    @router.get("/auth/callback")
    async def auth_callback(
        request: Request, code: str = Query(default=""), state: str = Query(default="")
    ) -> RedirectResponse:
        """Complete OAuth: verify state, check org membership, set a session cookie."""
        oauth = getattr(request.app.state, "oauth", None)
        if oauth is None:
            raise HTTPException(status_code=404, detail="OAuth is not enabled.")
        expected = request.cookies.get(OAUTH_STATE_COOKIE, "")
        if not state or not expected or not secrets.compare_digest(state, expected):
            raise HTTPException(status_code=400, detail="Invalid OAuth state.")
        redirect_uri = _base_url(request) + "/api/auth/callback"
        try:
            token = await asyncio.to_thread(oauth.exchange_code, code, redirect_uri)
            member = await asyncio.to_thread(oauth.is_org_member, token)
            gh = await asyncio.to_thread(oauth.fetch_user, token)
        except OAuthError as exc:
            raise HTTPException(status_code=502, detail=f"GitHub login failed: {exc}")

        home = _base_url(request) + "/"
        if not member:
            # Bounce back to the SPA with an error it can surface, rather than a raw 403.
            resp = RedirectResponse(home + f"?auth_error=not_member&org={settings.github_org}",
                                    status_code=307)
            resp.delete_cookie(OAUTH_STATE_COOKIE, path="/")
            return resp

        store = request.app.state.user_store
        store.upsert_user(gh["id"], gh["login"], gh.get("name"), gh.get("avatar_url"))
        session = store.create_session(gh["id"])
        manager._audit("login", actor=gh["login"], source="oauth")
        resp = RedirectResponse(home, status_code=307)
        resp.set_cookie(
            SESSION_COOKIE, session, max_age=int(settings.session_ttl_days * 86400),
            httponly=True, secure=_cookie_secure(request), samesite="lax", path="/",
        )
        resp.delete_cookie(OAUTH_STATE_COOKIE, path="/")
        return resp

    @router.post("/auth/logout")
    async def auth_logout(request: Request) -> JSONResponse:
        token = request.cookies.get(SESSION_COOKIE, "")
        store = getattr(request.app.state, "user_store", None)
        if store is not None:
            store.delete_session(token)
        resp = JSONResponse({"ok": True})
        resp.delete_cookie(SESSION_COOKIE, path="/")
        return resp

    @router.get("/activity")
    async def activity(limit: int = Query(default=50, ge=1, le=500)) -> dict:
        """Recent control events (who launched/stopped/deleted what) from the audit log."""
        return {"type": "activity", "events": manager.recent_activity(limit)}

    @router.get("/status")
    async def status() -> dict:
        return manager.status_msg()

    @router.get("/active")
    async def active() -> dict:
        return manager.active_runs_msg()

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
        result = await manager.delete_model(run, actor=_user)
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

    @router.post("/devices/{device_id}/slots")
    async def set_device_slots(
        device_id: str,
        req: DeviceSlotsRequest,
        _user: str = Depends(require_control),
    ) -> dict:
        """Set how many jobs a device may train concurrently (admin slider).

        ``max_slots`` null clears the override so the device runs at its reported
        capacity. The new cap takes effect on the worker's next poll/heartbeat."""
        ok = manager.devices.set_max_slots(device_id, req.max_slots)
        if not ok:
            raise HTTPException(status_code=404, detail="Device not found.")
        await broadcaster.broadcast(manager.devices_msg())
        return {"ok": True}

    # ── worker protocol (device-token gated) ────────────────────────────────
    def require_device(x_device_token: str = Header(default="")) -> str:
        device_id = manager.devices.verify(x_device_token)
        if not device_id:
            raise HTTPException(status_code=401, detail="Invalid device token.")
        return device_id

    @router.post("/worker/lease")
    async def worker_lease(
        device_id: str = Depends(require_device),
        x_worker_cores: Optional[int] = Header(default=None),
        x_worker_capacity: Optional[int] = Header(default=None),
    ):
        # The worker reports its capability (CPU cores + recommended concurrency) on
        # each poll so the admin UI can size the concurrency slider.
        job = await manager.lease_for_worker(
            device_id, cores=x_worker_cores, capacity=x_worker_capacity
        )
        if job is None:
            return Response(status_code=204)  # nothing ready (or device at capacity)
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

    # ── federated distributed training (shard ⇆ coordinator) ────────────────────
    def require_fed(x_fed_token: str = Header(default="")) -> bool:
        """A fed shard authenticates with the local ingest token or a device token."""
        if x_fed_token and secrets.compare_digest(x_fed_token, manager.ingest_token):
            return True
        if manager.devices.verify(x_fed_token):
            return True
        raise HTTPException(status_code=401, detail="Invalid fed token.")

    @router.post("/dist/{group}/push")
    async def dist_push(
        group: str,
        round: int = Query(...),
        shard: str = Query(...),
        shards: int = Query(...),
        weight: float = Query(1.0),  # shard's env count for weighted FedAvg
        file: UploadFile = File(...),
        _ok: bool = Depends(require_fed),
    ) -> dict:
        data = await file.read()
        return await manager.dist_push(group, round, shard, shards, data, weight)

    @router.get("/dist/{group}/pull")
    async def dist_pull(
        group: str, round: int = Query(...), _ok: bool = Depends(require_fed)
    ) -> Response:
        data = await manager.dist_pull(group, round)
        if data is None:
            return Response(status_code=204)  # round not ready yet — shard retries
        return Response(content=data, media_type="application/octet-stream")

    @router.post("/jobs")
    async def create_job(req: LaunchRequest, _user: str = Depends(require_control)) -> dict:
        result = await manager.launch(req.model_dump(), actor=_user)
        if not result.get("ok"):
            raise HTTPException(status_code=400, detail=result.get("message", "launch failed"))
        return result

    @router.post("/eval")
    async def create_eval(req: EvalRequest, _user: str = Depends(require_control)) -> dict:
        """Launch an evaluation drill (separate lane — does not consume training slots)."""
        result = await manager.launch_eval(req.model_dump(), actor=_user)
        if not result.get("ok"):
            raise HTTPException(status_code=400, detail=result.get("message", "eval failed"))
        return result

    @router.post("/eval/stop")
    async def stop_eval(_user: str = Depends(require_control)) -> dict:
        return await manager.stop_eval(actor=_user)

    @router.post("/control")
    async def control_match(req: ControlRequest, _user: str = Depends(require_control)) -> dict:
        """Send a manual control command (human action / red mode) to a running match.

        Forwarded to the match's play.py subprocess over its control sink. The action is
        validated to 4 floats clipped to [-1, 1] so a malformed payload can't drive the robot
        out of range."""
        msg: dict = {}
        if req.action is not None:
            if len(req.action) != 4:
                raise HTTPException(status_code=400, detail="action must have 4 elements")
            try:
                msg["action"] = [max(-1.0, min(1.0, float(x))) for x in req.action]
            except (TypeError, ValueError):
                raise HTTPException(status_code=400, detail="action must be numeric")
        if req.red_mode is not None:
            msg["red_mode"] = req.red_mode
        if not msg:
            raise HTTPException(status_code=400, detail="nothing to send")
        result = await manager.push_control(req.run, msg)
        if not result.get("ok"):
            raise HTTPException(status_code=409, detail=result.get("message", "control failed"))
        return result

    # ── online play-by-game-code (open guest play, no login) ─────────────────────
    # Lightweight per-IP throttle on the (subprocess-spawning) create/join calls so an
    # anonymous endpoint can't be used to flood the server with games. Best-effort,
    # in-memory; control messages (high-frequency) are NOT throttled here.
    _game_hits: dict[str, list[float]] = {}

    def _throttle_game(request: Request, limit: int = 12, window: float = 60.0) -> None:
        ip = request.client.host if request.client else "?"
        now = asyncio.get_event_loop().time()
        hits = [t for t in _game_hits.get(ip, []) if now - t < window]
        if len(hits) >= limit:
            raise HTTPException(status_code=429, detail="Too many games; slow down a moment.")
        hits.append(now)
        _game_hits[ip] = hits

    def _clip_action(action: list[float] | None) -> list[float] | None:
        if action is None:
            return None
        if len(action) != 4:
            raise HTTPException(status_code=400, detail="action must have 4 elements")
        try:
            return [max(-1.0, min(1.0, float(x))) for x in action]
        except (TypeError, ValueError):
            raise HTTPException(status_code=400, detail="action must be numeric")

    @router.post("/games")
    async def create_game(req: CreateGameRequest, request: Request) -> dict:
        _throttle_game(request)
        result = await manager.create_game(
            mode=req.mode, seed=req.seed, claim_side=req.claim_side,
            policy_a=req.policy_a.model_dump() if req.policy_a else None,
            policy_b=req.policy_b.model_dump() if req.policy_b else None,
        )
        if not result.get("ok"):
            raise HTTPException(status_code=400, detail=result.get("message", "could not start game"))
        return result

    @router.post("/games/{code}/join")
    async def join_game(code: str, req: JoinGameRequest, request: Request) -> dict:
        _throttle_game(request)
        result = await manager.join_game(code, side=req.side)
        if not result.get("ok"):
            raise HTTPException(status_code=result.get("status", 400),
                                detail=result.get("message", "could not join game"))
        return result

    @router.get("/games/{code}")
    async def get_game(code: str) -> dict:
        state = manager.game_state(code)
        if state is None:
            raise HTTPException(status_code=404, detail="Game not found.")
        return {"ok": True, **state}

    @router.post("/games/{code}/leave")
    async def leave_game(code: str, req: LeaveGameRequest) -> dict:
        result = await manager.leave_game(code, req.side, req.token)
        if not result.get("ok"):
            raise HTTPException(status_code=result.get("status", 400),
                                detail=result.get("message", "could not leave game"))
        return result

    @router.post("/games/{code}/control")
    async def game_control(code: str, req: GameControlRequest) -> dict:
        """Send a player's control to their side of a game. The game code + side token are
        the credential — no login needed (open guest play)."""
        if not manager.verify_game_token(code, req.side, req.token):
            raise HTTPException(status_code=403, detail="Invalid game credentials.")
        run = manager.game_run(code)
        if run is None:
            raise HTTPException(status_code=404, detail="Game not found.")
        msg: dict = {"side": req.side}
        action = _clip_action(req.action)
        if action is not None:
            msg["action"] = action
        if req.mode is not None:
            msg["mode"] = req.mode
        result = await manager.push_control(run, msg)
        if not result.get("ok"):
            raise HTTPException(status_code=409, detail=result.get("message", "control failed"))
        return result

    @router.post("/jobs/stop")
    async def stop_job(_user: str = Depends(require_control)) -> dict:
        # Legacy: stop the primary local run (single-run UI). Per-run stop below.
        return await manager.kill(actor=_user)

    @router.post("/jobs/{run_name}/stop")
    async def stop_run(run_name: str, _user: str = Depends(require_control)) -> dict:
        result = await manager.kill(run_name, actor=_user)
        if not result.get("ok"):
            raise HTTPException(status_code=404, detail=result.get("message", "stop failed"))
        return result

    @router.post("/queue")
    async def add_to_queue(req: QueueAddRequest, _user: str = Depends(require_control)) -> dict:
        data = req.model_dump()
        start_at = data.pop("start_at", None)
        result = await manager.enqueue(data, start_at, actor=_user)
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
            await broadcaster.send(ws, manager.active_runs_msg())
            await broadcaster.send(ws, manager.runs_msg())
            await broadcaster.send(ws, manager.queue_msg())
            await broadcaster.send(ws, manager.models_msg())
            await broadcaster.send(ws, manager.devices_msg())
            await broadcaster.send(ws, manager.eval_status_msg())  # recover an in-flight eval
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

    @router.websocket("/control_sink")
    async def control_sink(
        ws: WebSocket,
        token: str = Query(default=""),
        run: str = Query(default=""),
    ) -> None:
        """Hub → play.py manual-control stream.

        The match subprocess connects here (local ingest token) to receive the browser's
        control messages for its run. The hub pushes; the subprocess only reads. We keep the
        socket registered for ``run`` and clean it up on disconnect."""
        header_token = ws.headers.get("x-device-token", "")
        effective = header_token or token
        if not secrets.compare_digest(effective, manager.ingest_token):
            await ws.close(code=1008)
            return
        await ws.accept()
        manager.register_control_sink(run, ws)
        try:
            # The subprocess doesn't send; reading just detects disconnect.
            while True:
                await ws.receive_text()
        except WebSocketDisconnect:
            pass
        except Exception:  # noqa: BLE001 — client dropped
            pass
        finally:
            manager.unregister_control_sink(run, ws)

    return router
