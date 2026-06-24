"""Job orchestration: spawn/kill/monitor training & match subprocesses.

Ported from the old ``hub.py``. The FastAPI process is long-lived; it spawns
``scripts/train.py`` / ``scripts/play.py`` as isolated subprocesses, which stream
frames back to the internal ``/api/ingest`` WebSocket, and the broadcaster fans them
out to public browser clients. Control methods return a result dict so the HTTP layer
can map failures to status codes; status changes are also broadcast to the stream.
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import secrets
import signal
import subprocess
import sys
import time
from pathlib import Path

from bucky.curriculum import Stage

from .broadcast import Broadcaster
from .config import Settings
from .db import Database
from .devices import DeviceRegistry
from .models import (
    ModelConfig,
    launch_meta,
    list_models,
    run_name_for,
    write_config,
    write_meta,
)

# How long a remote worker's lease on a job survives without a heartbeat before the
# coordinator reclaims it (returns the job to pending so another worker can take it).
LEASE_TTL = 45.0
MAX_LEASE_ATTEMPTS = 3

# Online "play by game code" rooms.
GAME_CODE_ALPHABET = "ABCDEFGHJKMNPQRSTUVWXYZ23456789"  # no ambiguous O/0/I/1/L
GAME_CODE_LEN = 4
ROOM_IDLE_TTL = 120.0   # a room nobody is playing/watching is reaped after this many seconds
MAX_ROOMS = 20          # safety cap on concurrent games

log = logging.getLogger(__name__)


def _detect_preload_allocator() -> str | None:
    """Locate a faster malloc (tcmalloc/jemalloc) to LD_PRELOAD into trainer
    subprocesses. Multi-process NumPy rollout collection is allocation-heavy, and
    these allocators reuse memory better than glibc malloc under that load. Returns
    the .so path, or None when none is installed (then LD_PRELOAD is left untouched
    so dev boxes without it are unaffected).
    """
    import ctypes.util

    # Explicit distro paths first (these runtime-only libs aren't always resolvable
    # via the linker name search), then fall back to ctypes' own lookup.
    for path in (
        "/usr/lib/x86_64-linux-gnu/libtcmalloc_minimal.so.4",
        "/usr/lib/x86_64-linux-gnu/libtcmalloc.so.4",
        "/usr/lib/x86_64-linux-gnu/libjemalloc.so.2",
        "/usr/lib/libtcmalloc_minimal.so.4",
        "/usr/lib/libjemalloc.so.2",
    ):
        if os.path.exists(path):
            return path
    for name in ("tcmalloc_minimal", "tcmalloc", "jemalloc"):
        found = ctypes.util.find_library(name)
        if found:
            return found
    return None


# Detected once at import: faster allocator to preload into trainer subprocesses.
_PRELOAD_ALLOCATOR = _detect_preload_allocator()

VALID_STAGES = {s.value for s in Stage}
# Drills the eval tool can run: the four foundational stages, each a concrete scenario
# (excludes the FULL_TRAINING meta-stage and 2v2, which the eval script doesn't build).
EVAL_STAGES = {
    Stage.APPROACH_STATIC_BALL.value,
    Stage.PUSH_TO_EMPTY_GOAL.value,
    Stage.AIM_AND_KICK.value,
    Stage.SELF_PLAY_1V1.value,
}


class JobManager:
    """Spawns one job at a time and relays its frames/status to the broadcaster."""

    def __init__(
        self,
        broadcaster: Broadcaster,
        settings: Settings,
        base_dir: str | None = None,
        db: Database | None = None,
    ) -> None:
        self._bc = broadcaster
        self._settings = settings
        # app/jobs.py -> parent is app/, parent.parent is the backend root (scripts/ + checkpoints/)
        self._base_dir = Path(base_dir) if base_dir else Path(__file__).resolve().parent.parent
        # One SQLite DB backs the queue, devices and audit log. Resolve its path under
        # base_dir when relative, so a test that passes a tmp base_dir gets its own DB.
        if db is None:
            db_path = Path(settings.db_path)
            if not db_path.is_absolute():
                db_path = self._base_dir / db_path
            db = Database(db_path)
        self._db = db
        self._launch_lock = asyncio.Lock()
        # Unified registry of in-flight runs, keyed by run_name. Covers both the server's
        # own local trainers (device == "server", entry carries a live ``proc``) and runs
        # leased to remote guest devices (device == device_id, no ``proc``). Each entry is
        # a plain dict broadcast to the UI (minus the unpicklable ``proc``). Replaces the
        # old single-run ``_proc``/``_state``/``_run_meta`` so several runs coexist.
        self._active: dict[str, dict] = {}
        # Evaluation drills run in a *separate* lane, keyed by run_name. Kept out of
        # ``_active`` so eval never counts against ``local_slots`` or appears in the training
        # multi-run panel — analysis can run while the production trainer keeps going.
        self._eval_active: dict[str, dict] = {}
        # Ref-count of connected local trainer ingest sockets → ``trainer_connected``.
        self._ingest_conns = 0
        self._tasks: list[asyncio.Task] = []

        # Manual-match control: one live downstream socket per match run_name. The play.py
        # subprocess connects to /api/control_sink and we forward the browser's control
        # messages (human action / red mode toggle) to it. Best-effort; dropped if absent.
        self._control_sinks: dict = {}

        # Online human-vs-human "play by game code" rooms. Ephemeral and in-memory: a code
        # maps to a running match (run_name) plus two side slots, each with a secret join
        # token that authorises that browser's control without a login. Lost on restart
        # (acceptable — a game is a transient session, not a saved artifact).
        self._rooms: dict[str, dict] = {}
        self._rooms_by_run: dict[str, str] = {}
        self._rooms_lock = asyncio.Lock()

        # Distributed training: registered guest devices lease train jobs from the
        # same queue. A lock serialises lease/heartbeat/complete against the local
        # scheduler so two workers can't claim the same item.
        self._state_dir = self._base_dir / "state"  # home of any legacy JSON to import
        self._devices = DeviceRegistry(self._db)
        self._devices.import_legacy_json(self._state_dir / "devices.json")
        self._lease_lock = asyncio.Lock()

        # Persisted job queue: items run back-to-back when the manager is idle, each
        # with an optional `start_at` (epoch) so runs can be scheduled while away.
        self._queue: list[dict] = []
        self._queue_seq = 0
        self._load_queue()

        # Federated distributed training: per-group, per-round weight-averaging buffers.
        # group -> {"rounds": {round: {"pushes": {shard: arrays}, "averaged": bytes|None,
        # "first_at": epoch}}}. Kept in memory only (transient sync state, not a result).
        self._dist: dict[str, dict] = {}
        self._dist_lock = asyncio.Lock()

    @property
    def ingest_token(self) -> str:
        return self._settings.ingest_token

    @property
    def devices(self) -> DeviceRegistry:
        return self._devices

    @property
    def db(self) -> Database:
        return self._db

    # ── audit / history ──────────────────────────────────────────────────────────
    def _audit(
        self,
        action: str,
        run: str | None = None,
        actor: str | None = None,
        source: str = "server",
        detail: str | None = None,
    ) -> None:
        """Record a control event in the ``jobs`` table (best-effort, never raises)."""
        try:
            self._db.execute(
                "INSERT INTO jobs (ts, action, run, actor, source, detail) "
                "VALUES (?, ?, ?, ?, ?, ?)",
                (time.time(), action, run, actor, source, detail),
            )
        except Exception:  # noqa: BLE001 — audit must never break a control action
            log.warning("Could not write audit row for %s", action, exc_info=True)

    def recent_activity(self, limit: int = 50) -> list[dict]:
        """Most recent audit rows, newest first (for the activity panel)."""
        rows = self._db.query(
            "SELECT ts, action, run, actor, source, detail FROM jobs "
            "ORDER BY id DESC LIMIT ?",
            (int(limit),),
        )
        return [dict(r) for r in rows]

    # ── lifecycle ──────────────────────────────────────────────────────────────
    def start_background(self) -> None:
        self._tasks = [
            asyncio.create_task(self._heartbeat_task()),
            asyncio.create_task(self._monitor_task()),
            asyncio.create_task(self._scheduler_task()),
            asyncio.create_task(self._lease_reaper_task()),
        ]

    async def shutdown(self) -> None:
        for t in self._tasks:
            t.cancel()
        loop = asyncio.get_running_loop()
        for entry in [*self._local_active(), *self._eval_active.values()]:
            proc = entry.get("proc")
            if proc is not None and proc.poll() is None:
                await loop.run_in_executor(None, self._terminate, proc)

    # ── active-run registry helpers ──────────────────────────────────────────────
    def _local_active(self) -> list[dict]:
        """Runs executing in the server's own process (have a live ``proc``)."""
        return [e for e in self._active.values() if e.get("device") == "server"]

    @staticmethod
    def _public_run(entry: dict) -> dict:
        """A broadcast-safe copy of a run entry (drops the unpicklable ``proc``)."""
        return {k: v for k, v in entry.items() if k != "proc"}

    def active_runs_msg(self) -> dict:
        return {
            "type": "active_runs",
            "runs": [self._public_run(e) for e in self._active.values()],
            "local_slots": self._settings.local_slots,
            "local_active": len(self._local_active()),
        }

    # ── ingest (trainer → hub) ───────────────────────────────────────────────────
    async def on_ingest_connect(self) -> None:
        self._ingest_conns += 1
        log.info("Trainer connected to /api/ingest (%d open)", self._ingest_conns)
        await self._bc.broadcast(self.status_msg())

    async def on_ingest_disconnect(self) -> None:
        self._ingest_conns = max(0, self._ingest_conns - 1)
        log.info("Trainer disconnected from /api/ingest (%d open)", self._ingest_conns)
        await self._bc.broadcast(self.status_msg())

    async def handle_ingest_raw(self, raw: str, device: str = "server", run: str = "") -> None:
        """Relay a trainer frame to browsers, tagged with its source device/run.

        Every frame carries the source device (``"server"`` for the in-process local
        trainer) and its run name, so the browser keys streams uniformly and the active
        registry tracks each run's phase/steps regardless of where it runs."""
        try:
            msg = json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            return

        # Eval drills live in their own lane. A status frame updates the eval entry's phase;
        # eval_step / eval_summary frames fall through to the generic fan-out below (tagged
        # with run + device) so they reach /api/stream without touching the training lists.
        eval_entry = self._eval_active.get(run)
        if eval_entry is not None and msg.get("type") == "trainer_status":
            if msg.get("phase") is not None:
                eval_entry["phase"] = msg["phase"]
            if "message" in msg:
                eval_entry["message"] = msg["message"]
            await self._bc.broadcast(self.eval_status_msg())
            return

        entry = self._active.get(run)
        if msg.get("type") == "trainer_status":
            if entry is not None:
                if msg.get("phase") is not None:
                    entry["phase"] = msg.get("phase")
                if "num_timesteps" in msg:
                    entry["num_timesteps"] = msg["num_timesteps"]
                if "message" in msg:
                    entry["message"] = msg["message"]
                if msg.get("run_type"):
                    entry["run_type"] = msg["run_type"]
            await self._bc.broadcast(self.status_msg())
            await self._bc.broadcast(self.active_runs_msg())
            if device != "server":
                # Back-compat per-device status frame for any existing consumer.
                await self._bc.broadcast({
                    "type": "remote_status", "device": device, "run": run,
                    "phase": msg.get("phase"), "num_timesteps": msg.get("num_timesteps"),
                })
                self._devices.touch(device)
            return

        # step / metrics frame: tag with source + run and fan out (a bare check-in for
        # remote devices — no job-list churn, that's maintained by lease/heartbeat).
        msg["device"] = device
        msg["run"] = run
        if device != "server":
            self._devices.touch(device)
        # Track step progress on the run entry. Metrics frames are infrequent (once per
        # PPO update), so it's cheap to refresh the active list from them — this is the
        # only step signal for headless runs (which emit no per-step viz frames).
        if entry is not None and isinstance(msg.get("num_timesteps"), (int, float)):
            entry["num_timesteps"] = int(msg["num_timesteps"])
            if msg.get("type") == "metrics":
                await self._bc.broadcast(self.active_runs_msg())
        await self._bc.broadcast(msg)

    # ── manual-match control (browser → hub → play.py) ───────────────────────────
    def register_control_sink(self, run: str, ws) -> None:
        """A play.py subprocess subscribed to receive manual control for ``run``."""
        self._control_sinks[run] = ws
        log.info("Control sink connected for %s", run)

    def unregister_control_sink(self, run: str, ws) -> None:
        if self._control_sinks.get(run) is ws:
            del self._control_sinks[run]
            log.info("Control sink disconnected for %s", run)

    async def push_control(self, run: str, msg: dict) -> dict:
        """Forward a browser control message to the match's play.py sink (best-effort)."""
        ws = self._control_sinks.get(run)
        if ws is None:
            return {"ok": False, "message": "No manual match is accepting control."}
        try:
            await ws.send_text(json.dumps(msg))
        except Exception:  # noqa: BLE001 — sink dropped; let the WS handler clean it up
            return {"ok": False, "message": "Control channel is not connected."}
        return {"ok": True}

    # ── online play-by-code rooms (browser ⇆ hub ⇆ play.py) ──────────────────────
    def _new_game_code(self) -> str:
        for _ in range(50):
            code = "".join(secrets.choice(GAME_CODE_ALPHABET) for _ in range(GAME_CODE_LEN))
            if code not in self._rooms:
                return code
        raise RuntimeError("could not allocate a unique game code")

    def _default_policy(self) -> dict | None:
        """Pick a trained opponent-aware checkpoint to seed a game's AI sides.

        A match needs 21-dim self-play policies; ``opponent_snapshot.zip`` marks a self-play
        run, so prefer those (newest first), and within a run ``final_model.zip`` then
        ``best_model.zip``. Returns a ``{run, checkpoint}`` ref, or None if none exists."""
        ckpt_root = self._base_dir / "checkpoints"
        if not ckpt_root.is_dir():
            return None
        candidates = []
        for d in ckpt_root.iterdir():
            if not d.is_dir() or not (d / "opponent_snapshot.zip").is_file():
                continue
            for name in ("final_model.zip", "best_model.zip"):
                if (d / name).is_file():
                    candidates.append((d.stat().st_mtime, d.name, name))
                    break
        if not candidates:
            return None
        candidates.sort(reverse=True)
        _mtime, run, ckpt = candidates[0]
        return {"run": run, "checkpoint": ckpt}

    def _claim_slot(self, room: dict, side: str) -> str:
        """Mint a fresh join token for ``side`` and mark it claimed."""
        token = secrets.token_urlsafe(18)
        room["slots"][side].update(token=token, claimed=True, last_seen=time.time())
        room["last_active"] = time.time()
        return token

    async def create_game(self, mode: str = "casual", seed: int = 0,
                          claim_side: str | None = "a", policy_a: dict | None = None,
                          policy_b: dict | None = None, actor: str | None = None) -> dict:
        """Spawn a match and register a shareable game-code room for it.

        ``claim_side`` (a|b|None) optionally reserves a side for the creator and returns its
        secret token. AI policies fill any unclaimed side until a human takes over."""
        if mode not in ("casual", "match"):
            return self._reject("Unknown game mode.")
        if claim_side not in ("a", "b", None):
            return self._reject("Invalid side.")
        if len(self._rooms) >= MAX_ROOMS:
            return self._reject("Too many games are running right now; try again shortly.")
        pol_a = policy_a or self._default_policy()
        pol_b = policy_b or self._default_policy()
        if not pol_a or not pol_b:
            return self._reject("No trained model is available to host a game yet.")
        cfg = {"mode": "play", "seed": seed, "policy_a": pol_a, "policy_b": pol_b,
               "control": True, "game_mode": mode}
        result = await self.launch(cfg, actor=actor)
        if not result.get("ok"):
            return result
        run_name = result.get("run_name")
        if not run_name:
            return self._reject("Game launch did not report a run name.")
        async with self._rooms_lock:
            code = self._new_game_code()
            now = time.time()
            room = {
                "code": code, "run_name": run_name, "mode": mode,
                "created_at": now, "last_active": now,
                "slots": {s: {"token": None, "claimed": False, "last_seen": None}
                          for s in ("a", "b")},
            }
            self._rooms[code] = room
            self._rooms_by_run[run_name] = code
            your_side, side_token = None, None
            if claim_side in ("a", "b"):
                side_token = self._claim_slot(room, claim_side)
                your_side = claim_side
        self._audit("game_created", run_name, actor, detail=f"code={code} mode={mode}")
        return {"ok": True, "code": code, "run_name": run_name, "mode": mode,
                "your_side": your_side, "side_token": side_token}

    async def join_game(self, code: str, side: str | None = None,
                        actor: str | None = None) -> dict:
        """Claim a free side in a game by its code (or become a spectator if full)."""
        code = (code or "").strip().upper()
        if side not in ("a", "b", None):
            return {"ok": False, "status": 400, "message": "Invalid side."}
        async with self._rooms_lock:
            room = self._rooms.get(code)
            if room is None:
                return {"ok": False, "status": 404, "message": "Game not found."}
            slots = room["slots"]
            if side is None:
                side = ("a" if not slots["a"]["claimed"]
                        else "b" if not slots["b"]["claimed"] else None)
            if side is None or slots[side]["claimed"]:
                # Both sides taken (or the requested one is) → watch as a spectator.
                return {"ok": True, "spectator": True, "run_name": room["run_name"],
                        "mode": room["mode"]}
            token = self._claim_slot(room, side)
            opp = "b" if side == "a" else "a"
            opponent_present = slots[opp]["claimed"]
            run_name, game_mode = room["run_name"], room["mode"]
        self._audit("game_joined", run_name, actor, detail=f"code={code} side={side}")
        return {"ok": True, "run_name": run_name, "your_side": side, "side_token": token,
                "mode": game_mode, "opponent_present": opponent_present}

    def verify_game_token(self, code: str, side: str, token: str) -> bool:
        """True iff ``token`` is the live join token for ``side`` of ``code`` (constant-time)."""
        room = self._rooms.get((code or "").strip().upper())
        if room is None or side not in ("a", "b") or not token:
            return False
        stored = room["slots"][side].get("token")
        if not stored or not secrets.compare_digest(stored, token):
            return False
        room["slots"][side]["last_seen"] = time.time()
        room["last_active"] = time.time()
        return True

    def game_run(self, code: str) -> str | None:
        room = self._rooms.get((code or "").strip().upper())
        return room["run_name"] if room else None

    def game_state(self, code: str, touch: bool = True) -> dict | None:
        """Public room view (no tokens) for the lobby/spectator. Touching keeps it alive."""
        room = self._rooms.get((code or "").strip().upper())
        if room is None:
            return None
        if touch:
            room["last_active"] = time.time()   # an open lobby/spectator keeps the room alive
        return {
            "code": room["code"], "run_name": room["run_name"], "mode": room["mode"],
            "slots": {s: {"claimed": room["slots"][s]["claimed"]} for s in ("a", "b")},
        }

    async def leave_game(self, code: str, side: str, token: str,
                        actor: str | None = None) -> dict:
        code = (code or "").strip().upper()
        async with self._rooms_lock:
            room = self._rooms.get(code)
            if room is None:
                return {"ok": True}   # already gone
            if not self.verify_game_token(code, side, token):
                return {"ok": False, "status": 403, "message": "Invalid game credentials."}
            room["slots"][side].update(token=None, claimed=False, last_seen=None)
            room["last_active"] = time.time()
            run_name = room["run_name"]
        self._audit("game_left", run_name, actor, detail=f"code={code} side={side}")
        return {"ok": True}

    def _drop_room_for_run(self, run_name: str) -> str | None:
        """Forget the room bound to ``run_name`` (its match has ended). Returns the code."""
        code = self._rooms_by_run.pop(run_name, None)
        if code is not None:
            self._rooms.pop(code, None)
        return code

    # ── control ──────────────────────────────────────────────────────────────────
    async def launch(self, cfg: dict, actor: str | None = None) -> dict:
        # A distributed run isn't a single local job — fan it out into shard queue items
        # that workers (incl. the local one) pick up and train together via FedAvg.
        dist = cfg.get("distributed")
        if str(cfg.get("mode", "train")) == "train" and dist and int(dist.get("shards") or 1) > 1:
            return await self._enqueue_distributed(cfg, dist, None, actor=actor)

        async with self._launch_lock:
            # Concurrency cap: the in-process worker runs up to ``local_slots`` runs at
            # once (each its own subprocess). Both training and match runs count.
            if len(self._local_active()) >= self._settings.local_slots:
                return self._reject("All local training slots are busy.")

            if str(cfg.get("mode", "train")) == "play":
                return await self._launch_play(cfg, actor=actor)

            stage = str(cfg.get("stage") or "APPROACH_STATIC_BALL")
            if stage not in VALID_STAGES:
                return self._reject(f"Unknown stage: {stage}")
            try:
                timesteps = max(1, int(cfg.get("timesteps") or 200_000))
                n_envs = max(1, int(cfg.get("n_envs") or 16))
                seed = int(cfg.get("seed") or 0)
            except (TypeError, ValueError):
                return self._reject("Invalid numeric config.")
            dr = cfg.get("domain_rand")
            domain_rand = True if dr is None else bool(dr)

            # Optional: continue from an existing checkpoint, forking a new run so the
            # source run's files stay intact. resume_from references a run + checkpoint
            # this server itself listed (validated here against path traversal).
            resume_path = None
            resume = cfg.get("resume_from")
            if resume:
                src_run = str(resume.get("run", ""))
                src_ckpt = str(resume.get("checkpoint", ""))
                if not self._is_safe_name(src_run) or not self._is_safe_name(src_ckpt):
                    return self._reject("Invalid checkpoint reference.")
                cand = self._base_dir / "checkpoints" / src_run / src_ckpt
                if not cand.is_file():
                    return self._reject(f"Checkpoint not found: {src_run}/{src_ckpt}")
                resume_path = cand

            # Stop condition: by steps (default) or by wall-clock (duration/until).
            stop_args, stop_meta = self._resolve_stop(cfg.get("stop"), timesteps)
            if stop_args is None:
                return self._reject(stop_meta)  # stop_meta is the error message here

            # Run identity: a user-named/versioned model gets `<name>_v<version>`;
            # otherwise fall back to the legacy `<stage>_seed<N>` scheme. Either way
            # the run gets config.json + meta.json sidecars so the admin panel can
            # show what kind of model it is.
            name = (cfg.get("name") or "").strip()
            version = (cfg.get("version") or "1").strip() or "1"
            if name:
                run_name = self._unique_run_name(run_name_for(name, version))
            else:
                base_name = f"{stage}_seed{seed}"
                run_name = self._unique_run_name(f"{base_name}_cont") if resume_path else base_name

            model_cfg = ModelConfig(
                name=name or run_name,
                version=version,
                stage=stage,
                full_training_split=cfg.get("full_training_split"),
                n_envs=n_envs,
                seed=seed,
                domain_rand=domain_rand,
                stop=cfg.get("stop"),
                hyperparams=cfg.get("hyperparams") or {},
                net_arch=cfg.get("net_arch") or [64, 64],
                reward_weights=cfg.get("reward_weights") or {},
                save_step_checkpoints=bool(cfg.get("save_step_checkpoints", False)),
                resume_from=resume if resume_path else None,
            )
            ckpt_root = self._base_dir / "checkpoints"
            config_path = write_config(ckpt_root, run_name, model_cfg.model_dump())
            write_meta(ckpt_root, run_name, launch_meta(model_cfg, run_name, created_by="server"))

            args = [
                sys.executable, "scripts/train.py",
                "--stage", stage,
                "--n-envs", str(n_envs),
                "--seed", str(seed),
                "--run-name", run_name,
                "--config", str(config_path),
                "--stream-url", self._ingest_url(run_name),
                *stop_args,
            ]
            if not domain_rand:
                args.append("--no-domain-rand")
            # Torch device for the in-process trainer. Opt-in per launch (config "device")
            # or per host (BUCKY_DEVICE env); defaults to cpu since the GPU is usually
            # slower than CPU for this tiny MLP. Unknown values fall back to cpu rather
            # than failing the launch on train.py's argparse choices.
            device = str(cfg.get("device") or os.getenv("BUCKY_DEVICE") or "cpu")
            if device not in ("cpu", "cuda", "auto"):
                device = "cpu"
            args += ["--device", device]
            # Headless by default: only animate the live field when the launch opted in.
            viz = bool(cfg.get("viz"))
            if viz:
                args.append("--viz")
            # Federated shard: sync weights with its group through the local coordinator.
            dist_group = cfg.get("dist_group")
            if dist_group:
                args += [
                    "--fed-server", f"http://localhost:{self._settings.port}/api",
                    "--fed-token", self._settings.ingest_token,
                    "--fed-group", str(dist_group),
                    "--fed-every", str(int(cfg.get("dist_sync_every") or 50_000)),
                    "--fed-shards", str(int(cfg.get("dist_shards") or 1)),
                ]
            if resume_path:
                args += ["--resume-from", str(resume_path)]

            entry = {
                "run_name": run_name, "device": "server", "stage": stage,
                "run_type": "train", "state": "launching",
                "model_name": model_cfg.name, "model_version": version,
                "n_envs": n_envs, "seed": seed, "domain_rand": domain_rand, "viz": viz,
                "phase": "launching", "num_timesteps": 0,
                "started_at": time.time(), "cancel_requested": False,
                "dist_group": cfg.get("dist_group"),
                "resumed_from": f"{resume['run']}/{resume['checkpoint']}" if resume_path else None,
                **stop_meta,
            }
            proc, spawn_err = self._spawn(args)
            if spawn_err is not None:
                return self._reject(f"Launch failed: {spawn_err}")
            entry["proc"] = proc
            entry["state"] = "running"
            self._active[run_name] = entry
            log.info("Launched %s (pid %d)", run_name, proc.pid)
            self._audit("launch", run_name, actor, source="server",
                        detail=f"stage={stage} seed={seed}")
            await self._bc.broadcast(self.status_msg())
            await self._bc.broadcast(self.active_runs_msg())
            return {"ok": True, "message": "launched", "status": self.status_msg()}

    async def _launch_play(self, cfg: dict, actor: str | None = None) -> dict:
        """Spawn a 1v1 match (scripts/play.py) between two checkpointed policies.

        Caller already holds the launch lock and verified no run is active.
        ``policy_a``/``policy_b`` are ``{run, checkpoint}`` pairs validated against path
        traversal and resolved under ``checkpoints/``.
        """
        try:
            seed = int(cfg.get("seed") or 0)
        except (TypeError, ValueError):
            return self._reject("Invalid seed.")
        dr = cfg.get("domain_rand")
        domain_rand = False if dr is None else bool(dr)

        resolved = {}
        for key in ("policy_a", "policy_b"):
            pol = cfg.get(key) or {}
            run, ckpt = str(pol.get("run", "")), str(pol.get("checkpoint", ""))
            if not self._is_safe_name(run) or not self._is_safe_name(ckpt):
                return self._reject(f"Invalid {key} reference.")
            cand = self._base_dir / "checkpoints" / run / ckpt
            if not cand.is_file():
                return self._reject(f"Checkpoint not found: {run}/{ckpt}")
            resolved[key] = (cand, f"{run}/{ckpt}")

        # Several matches could run within the slot budget; key each uniquely.
        run_name = "match"
        i = 2
        while run_name in self._active:
            run_name, i = f"match{i}", i + 1

        manual_red = bool(cfg.get("manual_red"))
        # Online games wire the control sink without forcing red-human (both sides start on
        # AI and flip to human as each browser sends input). game_mode picks the rule set.
        control_enabled = bool(cfg.get("control")) or manual_red
        game_mode = "casual" if str(cfg.get("game_mode")) == "casual" else "match"

        args = [
            sys.executable, "scripts/play.py",
            "--policy-a", str(resolved["policy_a"][0]),
            "--policy-b", str(resolved["policy_b"][0]),
            "--seed", str(seed),
            "--mode", game_mode,
            "--stream-url", self._ingest_url(run_name),
        ]
        if domain_rand:
            args.append("--domain-rand")
        if control_enabled:
            # play.py subscribes to the control sink for this run; humans drive their side.
            args += ["--control-url", self._control_url(run_name)]
        if manual_red:
            args.append("--manual-red")

        entry = {
            "run_name": run_name, "device": "server", "mode": "play",
            "run_type": "match", "state": "launching",
            "policy_a": resolved["policy_a"][1], "policy_b": resolved["policy_b"][1],
            "seed": seed, "domain_rand": domain_rand, "manual_red": manual_red,
            "game_mode": game_mode,
            "phase": "launching", "num_timesteps": 0,
            "started_at": time.time(), "cancel_requested": False,
        }
        proc, spawn_err = self._spawn(args)
        if spawn_err is not None:
            return self._reject(f"Launch failed: {spawn_err}")
        entry["proc"] = proc
        entry["state"] = "running"
        self._active[run_name] = entry
        log.info("Launched %s (pid %d): %s vs %s", run_name, proc.pid,
                 resolved["policy_a"][1], resolved["policy_b"][1])
        self._audit("launch", run_name, actor, source="server", detail="mode=play")
        await self._bc.broadcast(self.status_msg())
        await self._bc.broadcast(self.active_runs_msg())
        return {"ok": True, "message": "launched", "status": self.status_msg(),
                "run_name": run_name}

    async def kill(self, run_name: str | None = None, actor: str | None = None) -> dict:
        """Stop a run by name. ``None`` stops the primary local run (legacy single-run UI).

        A local run is SIGINT-checkpointed and terminated in place. A run leased to a
        remote device can't be signalled directly, so we flag it for cancellation; the
        worker sees the flag on its next heartbeat and SIGINTs its own trainer (which
        checkpoints before exiting), then reports completion to free the slot."""
        if run_name is None:
            local = self._local_active()
            if not local:
                return {"ok": True, "message": "No active run."}
            run_name = local[0]["run_name"]

        entry = self._active.get(run_name)
        if entry is None:
            return {"ok": False, "message": "No such active run."}

        if entry.get("device") == "server":
            proc = entry.get("proc")
            if proc is None or proc.poll() is not None:
                return {"ok": True, "message": "No active run."}
            entry["state"] = "stopping"
            self._audit("stop", run_name, actor, source="server")
            await self._bc.broadcast(self.status_msg())
            await self._bc.broadcast(self.active_runs_msg())
            await asyncio.get_running_loop().run_in_executor(None, self._terminate, proc)
            return {"ok": True, "message": "stopping"}

        # Remote run: request cancellation; keep the lease so the worker can save first.
        entry["cancel_requested"] = True
        entry["state"] = "stopping"
        self._audit("stop", run_name, actor, source="device")
        async with self._lease_lock:
            for q in self._queue:
                if q.get("run_name") == run_name:
                    q["cancel_requested"] = True
            self._save_queue()
        await self._bc.broadcast(self.active_runs_msg())
        return {"ok": True, "message": "cancel requested"}

    # ── evaluation drills (separate lane, off the training slots) ────────────────
    def eval_status_msg(self) -> dict:
        """The current eval drill (or null) — drives the /eval page and recovers an
        in-flight eval after a page reload."""
        entry = next(iter(self._eval_active.values()), None)
        return {"type": "eval_status",
                "eval": self._public_run(entry) if entry is not None else None}

    async def launch_eval(self, cfg: dict, actor: str | None = None) -> dict:
        """Run a checkpoint through a drill (curriculum stage), streaming the reward build-up.

        Lives in its own lane: gated by ``eval_slots`` (not ``local_slots``), so a busy
        trainer can't block it and it never inflates the training run list."""
        async with self._launch_lock:
            if len(self._eval_active) >= self._settings.eval_slots:
                return self._reject("An evaluation is already running.")

            stage = str(cfg.get("stage") or "")
            if stage not in EVAL_STAGES:
                return self._reject(f"Eval stage must be one of {sorted(EVAL_STAGES)}.")

            run = str(cfg.get("run", ""))
            checkpoint = str(cfg.get("checkpoint", ""))
            path = self.checkpoint_path(run, checkpoint)
            if path is None:
                return self._reject(f"Checkpoint not found: {run}/{checkpoint}")

            try:
                n_episodes = min(100, max(1, int(cfg.get("n_episodes") or 10)))
                seed = int(cfg.get("seed") or 999)
            except (TypeError, ValueError):
                return self._reject("Invalid numeric config.")
            deterministic = cfg.get("deterministic")
            deterministic = True if deterministic is None else bool(deterministic)

            run_name = f"eval_{run}_{int(time.time())}"
            i = 2
            while run_name in self._eval_active or run_name in self._active:
                run_name, i = f"eval_{run}_{int(time.time())}_{i}", i + 1

            args = [
                sys.executable, "scripts/eval_drill.py",
                "--checkpoint", str(path),
                "--stage", stage,
                "--n-episodes", str(n_episodes),
                "--seed", str(seed),
                "--stream-url", self._ingest_url(run_name),
                "--deterministic" if deterministic else "--no-deterministic",
            ]
            entry = {
                "run_name": run_name, "device": "server", "run_type": "eval",
                "state": "running", "stage": stage,
                "checkpoint": f"{run}/{checkpoint}", "n_episodes": n_episodes,
                "seed": seed, "deterministic": deterministic,
                "phase": "launching", "started_at": time.time(),
            }
            proc, spawn_err = self._spawn(args)
            if spawn_err is not None:
                return self._reject(f"Eval launch failed: {spawn_err}")
            entry["proc"] = proc
            self._eval_active[run_name] = entry
            log.info("Launched eval %s (pid %d): %s on %s", run_name, proc.pid,
                     entry["checkpoint"], stage)
            self._audit("eval", run_name, actor, source="server", detail=f"stage={stage}")
            await self._bc.broadcast(self.eval_status_msg())
            return {"ok": True, "message": "launched", "run_name": run_name}

    async def stop_eval(self, run_name: str | None = None, actor: str | None = None) -> dict:
        """Stop the running eval drill (targeted SIGINT, never a broad kill)."""
        if run_name is None:
            entry = next(iter(self._eval_active.values()), None)
        else:
            entry = self._eval_active.get(run_name)
        if entry is None:
            return {"ok": True, "message": "No active evaluation."}
        proc = entry.get("proc")
        if proc is None or proc.poll() is not None:
            return {"ok": True, "message": "No active evaluation."}
        entry["state"] = "stopping"
        self._audit("eval_stop", entry["run_name"], actor, source="server")
        await self._bc.broadcast(self.eval_status_msg())
        await asyncio.get_running_loop().run_in_executor(None, self._terminate, proc)
        return {"ok": True, "message": "stopping"}

    # ── subprocess helpers ─────────────────────────────────────────────────────
    def _ingest_url(self, run_name: str) -> str:
        return (
            f"ws://localhost:{self._settings.port}/api/ingest"
            f"?token={self._settings.ingest_token}&run={run_name}"
        )

    def _control_url(self, run_name: str) -> str:
        return (
            f"ws://localhost:{self._settings.port}/api/control_sink"
            f"?token={self._settings.ingest_token}&run={run_name}"
        )

    def _spawn(self, args: list[str]) -> tuple[subprocess.Popen | None, str | None]:
        """Start a job subprocess; return ``(proc, None)`` or ``(None, error_string)``."""
        # Preload a faster allocator for the trainer when one is installed; respect an
        # operator-set LD_PRELOAD. Must be set before exec (can't be done from Python
        # inside the child), so we inject it into the child's environment here.
        env = os.environ.copy()
        if _PRELOAD_ALLOCATOR and "LD_PRELOAD" not in env:
            env["LD_PRELOAD"] = _PRELOAD_ALLOCATOR
            log.info("LD_PRELOAD=%s for %s", _PRELOAD_ALLOCATOR, args[:2])
        try:
            # start_new_session=True puts the trainer (and its SubprocVecEnv workers)
            # in their own process group, so kill can signal the whole group and a
            # terminal Ctrl-C on the server won't leak into the trainer.
            proc = subprocess.Popen(
                args, cwd=str(self._base_dir), start_new_session=True, env=env
            )
        except OSError as exc:
            return None, str(exc)
        return proc, None

    @staticmethod
    def _terminate(proc: subprocess.Popen) -> None:
        """Stop the trainer, preferring a clean checkpointing shutdown.

        SIGINT is sent to the trainer process *only* (not the group): the main process
        raises KeyboardInterrupt, saves, and closes its SubprocVecEnv workers in order.
        Signalling the whole group here would kill the workers first and break the
        training pipes before the save could run. If the trainer doesn't exit in time,
        escalate to a group SIGTERM → SIGKILL to reap any stragglers.
        """
        if proc.poll() is None:
            try:
                proc.send_signal(signal.SIGINT)
            except ProcessLookupError:
                return
            try:
                proc.wait(timeout=10)
                return
            except subprocess.TimeoutExpired:
                pass

        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            return
        for sig in (signal.SIGTERM, signal.SIGKILL):
            if proc.poll() is not None:
                return
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                return
            try:
                proc.wait(timeout=5)
                return
            except subprocess.TimeoutExpired:
                continue

    # ── background tasks ─────────────────────────────────────────────────────
    async def _monitor_task(self) -> None:
        while True:
            await asyncio.sleep(0.5)
            exited: list[tuple[str, int | None]] = []
            for run_name, entry in list(self._active.items()):
                if entry.get("device") != "server":
                    continue
                proc = entry.get("proc")
                if proc is not None and proc.poll() is not None and entry.get("state") in (
                    "launching", "running", "stopping"
                ):
                    code = proc.returncode
                    entry["proc"] = None
                    del self._active[run_name]
                    self._drop_dist_group_if_idle(entry.get("dist_group"))
                    self._drop_room_for_run(run_name)   # its game (if any) is over
                    exited.append((run_name, code))
            if exited:
                for run_name, code in exited:
                    log.info("Job %s exited with code %s", run_name, code)
                    await self._bc.broadcast({"type": "run_exited", "run": run_name, "code": code})
                await self._bc.broadcast(self.status_msg())
                await self._bc.broadcast(self.active_runs_msg())
                await self._bc.broadcast(self.runs_msg())  # new checkpoints are now resumable
                await self._bc.broadcast(self.models_msg())  # admin panel: refreshed metadata

            # Reap finished eval drills (their own lane). Short-lived: they exit when the
            # N episodes finish or on a stop SIGINT. No checkpoints/runs to refresh.
            eval_exited: list[tuple[str, int | None]] = []
            for run_name, entry in list(self._eval_active.items()):
                proc = entry.get("proc")
                if proc is not None and proc.poll() is not None:
                    entry["proc"] = None
                    del self._eval_active[run_name]
                    eval_exited.append((run_name, proc.returncode))
            if eval_exited:
                for run_name, code in eval_exited:
                    log.info("Eval %s exited with code %s", run_name, code)
                    await self._bc.broadcast({"type": "run_exited", "run": run_name, "code": code})
                await self._bc.broadcast(self.eval_status_msg())

            # Reap game rooms nobody is playing or watching anymore: drop the room first so
            # it isn't re-reaped next tick, then stop the match (its exit is handled above).
            now = time.time()
            idle_runs = [r["run_name"] for r in list(self._rooms.values())
                         if now - r["last_active"] > ROOM_IDLE_TTL]
            for run_name in idle_runs:
                log.info("Reaping idle game room for %s", run_name)
                self._drop_room_for_run(run_name)
                await self.kill(run_name)

    async def _heartbeat_task(self) -> None:
        while True:
            await asyncio.sleep(1.5)
            await self._bc.broadcast({
                "type": "heartbeat",
                "t": time.time(),
                "clients": self._bc.count,
                "trainer_connected": self._ingest_conns > 0,
            })

    # ── helpers ──────────────────────────────────────────────────────────────
    def _reject(self, message: str) -> dict:
        return {"ok": False, "message": message, "status": self.status_msg(message=message)}

    @staticmethod
    def _is_safe_name(name: str) -> bool:
        """A single path segment — no separators or parent refs."""
        return bool(name) and "/" not in name and "\\" not in name and ".." not in name

    def _unique_run_name(self, base: str) -> str:
        ckpt_root = self._base_dir / "checkpoints"
        runs_root = self._base_dir / "runs"
        name, i = base, 2
        while (ckpt_root / name).exists() or (runs_root / name).exists():
            name, i = f"{base}{i}", i + 1
        return name

    # Far above any realistic step budget — used so a time-limited run is bounded by
    # the wall-clock StopAtTime callback rather than by a step count.
    _STEP_SENTINEL = 2_000_000_000

    def _resolve_stop(self, stop: dict | None, default_timesteps: int):
        """Translate a stop condition into trainer args + status metadata.

        Returns ``(args, meta)`` on success, or ``(None, error_message)`` on a bad value.
        ``stop`` is ``{"kind": "steps"|"duration"|"until", "value": number}``.
        """
        if not stop:
            return (["--timesteps", str(default_timesteps)],
                    {"stop_kind": "steps", "timesteps": default_timesteps})
        kind = str(stop.get("kind", "steps"))
        try:
            value = float(stop.get("value"))
        except (TypeError, ValueError):
            return (None, "Invalid stop value.")

        if kind == "steps":
            ts = max(1, int(value))
            return (["--timesteps", str(ts)], {"stop_kind": "steps", "timesteps": ts})

        if kind == "duration":
            if value < 1:
                return (None, "Duration must be at least 1 second.")
            deadline = time.time() + value
        elif kind == "until":
            if value <= time.time() + 1:
                return (None, "Stop time is in the past.")
            deadline = value
        else:
            return (None, f"Unknown stop kind: {kind}")

        # Pass an absolute --until so the trainer's deadline matches the one we report.
        return (["--until", repr(deadline), "--timesteps", str(self._STEP_SENTINEL)],
                {"stop_kind": kind, "deadline": deadline})

    # ── queue / scheduler ──────────────────────────────────────────────────────
    def queue_msg(self) -> dict:
        return {"type": "queue", "items": self._queue}

    def _import_legacy_queue(self) -> None:
        """One-time import of an old ``state/queue.json`` if the DB queue is empty."""
        if self._db.query_one("SELECT COUNT(*) AS c FROM queue")["c"] > 0:
            return
        path = self._state_dir / "queue.json"
        if not path.is_file():
            return
        try:
            data = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError):
            return
        self._queue = [q for q in data.get("items", []) if isinstance(q, dict)]
        self._queue_seq = int(data.get("seq", 0))
        if self._queue:
            self._save_queue()
            log.info("Imported %d queue item(s) from legacy %s", len(self._queue), path)

    def _load_queue(self) -> None:
        try:
            self._import_legacy_queue()
            rows = self._db.query("SELECT data, seq FROM queue ORDER BY position")
            self._queue = []
            self._queue_seq = 0
            for r in rows:
                try:
                    item = json.loads(r["data"])
                except (TypeError, json.JSONDecodeError):
                    continue
                # Anything caught mid-launch when the server stopped goes back to pending.
                if item.get("status") == "running":
                    item["status"] = "pending"
                self._queue.append(item)
                self._queue_seq = max(self._queue_seq, int(r["seq"] or 0))
        except Exception:  # noqa: BLE001 — a corrupt queue must not crash startup
            log.warning("Could not read queue from DB; starting with an empty queue")
            self._queue = []

    def _save_queue(self) -> None:
        """Atomically replace the queue table with the current in-memory list.

        A full rewrite preserves the simple list-of-dicts model the scheduler mutates,
        while keeping persistence atomic (one transaction) and the rows queryable."""
        try:
            with self._db.transaction() as conn:
                conn.execute("DELETE FROM queue")
                conn.executemany(
                    "INSERT INTO queue (id, position, seq, status, target, data) "
                    "VALUES (?, ?, ?, ?, ?, ?)",
                    [
                        (
                            it["id"], pos, self._queue_seq,
                            it.get("status", "pending"), it.get("target"),
                            json.dumps(it),
                        )
                        for pos, it in enumerate(self._queue)
                    ],
                )
        except Exception as exc:  # noqa: BLE001
            log.warning("Could not persist queue: %s", exc)

    def _next_queue_id(self) -> str:
        self._queue_seq += 1
        return f"q{self._queue_seq}"

    async def enqueue(self, cfg: dict, start_at: float | None, actor: str | None = None) -> dict:
        mode = str(cfg.get("mode", "train"))
        if mode not in ("train", "play"):
            return {"ok": False, "message": f"Unknown mode: {mode}"}

        # Reject obviously-bad stop conditions up front (the rest — e.g. a checkpoint
        # produced by an earlier queued run — can only be validated at launch time).
        stop = cfg.get("stop")
        if stop:
            kind = str(stop.get("kind", "steps"))
            try:
                value = float(stop.get("value"))
            except (TypeError, ValueError):
                return {"ok": False, "message": "Invalid stop value."}
            if kind == "until" and value <= time.time():
                return {"ok": False, "message": "Stop time is in the past."}
            if kind == "duration" and value < 1:
                return {"ok": False, "message": "Duration must be at least 1 second."}
            if kind == "steps" and value < 1:
                return {"ok": False, "message": "Step count must be positive."}

        # Distributed run: fan out into shard items that train together via FedAvg —
        # either N even shards, or a per-device map ({devices: [{target, n_envs}]}).
        dist = cfg.get("distributed")
        if mode == "train" and dist and (
            len(dist.get("devices") or []) > 1 or int(dist.get("shards") or 1) > 1
        ):
            return await self._enqueue_distributed(cfg, dist, start_at, actor=actor)

        item = self._make_queue_item(cfg, start_at)
        self._queue.append(item)
        self._save_queue()
        self._audit("enqueue", item["id"], actor, source="queue", detail=f"mode={mode}")
        await self._bc.broadcast(self.queue_msg())
        return {"ok": True, "item": item}

    def _make_queue_item(self, cfg: dict, start_at: float | None) -> dict:
        return {
            "id": self._next_queue_id(),
            "mode": str(cfg.get("mode", "train")),
            "config": cfg,
            "start_at": float(start_at) if start_at else None,
            "status": "pending",
            "enqueued_at": time.time(),
            # Where the run may execute: None/"any" = any worker, "server" = local
            # in-process worker only, otherwise a specific device id.
            "target": cfg.get("target") or None,
        }

    async def _enqueue_distributed(
        self, cfg: dict, dist: dict, start_at: float | None, actor: str | None = None
    ) -> dict:
        """Split one run into FedAvg shards sharing a sync group.

        Two shapes are supported. ``{shards: N}`` makes N even shards (any worker may
        pick them up). ``{devices: [{target, n_envs}]}`` makes one shard per device,
        pinned to that ``target`` (a device id or "server") and sized to its ``n_envs``
        — so heterogeneous boxes each train at their own capacity and contribute to the
        shared model proportionally (the weighted average in ``dist_push``).

        Each shard is an ordinary train job (its own run_name, checkpoints and a
        per-shard seed for sample diversity) carrying the group's sync parameters. They
        average policy weights through ``/api/dist`` every ``sync_every`` steps, so the
        fleet trains one converging model with the combined experience."""
        sync_every = max(1, int(dist.get("sync_every") or 50_000))
        base = (cfg.get("name") or cfg.get("stage") or "dist").strip() or "dist"
        group = self._unique_run_name(f"{base}_dist")
        self._dist.setdefault(group, {"rounds": {}})

        base_seed = int(cfg.get("seed") or 0)
        base_name = (cfg.get("name") or "").strip()

        # Per-device map (heterogeneous) or even N shards. Each entry below resolves to
        # (target, n_envs); even shards keep the run's target/n_envs for every shard.
        devices = [
            d for d in (dist.get("devices") or [])
            if int(d.get("n_envs") or 0) >= 1
        ]
        if len(devices) > 1:
            assignments = [(d.get("target") or "any", int(d["n_envs"])) for d in devices]
        else:
            shards = max(2, int(dist.get("shards") or 2))
            assignments = [(cfg.get("target") or "any", int(cfg.get("n_envs") or 16))
                           for _ in range(shards)]
        shards = len(assignments)

        items = []
        for i, (target, n_envs) in enumerate(assignments):
            shard_cfg = {k: v for k, v in cfg.items() if k != "distributed"}
            shard_cfg["dist_group"] = group
            shard_cfg["dist_sync_every"] = sync_every
            shard_cfg["dist_shards"] = shards
            shard_cfg["seed"] = base_seed + i  # diverse samples per shard
            shard_cfg["target"] = target       # pin this shard to its device
            shard_cfg["n_envs"] = n_envs        # size it for that device's capacity
            if base_name:
                shard_cfg["name"] = f"{base_name}_shard{i}"
                shard_cfg["version"] = cfg.get("version") or "1"
            else:
                shard_cfg["name"] = f"{group}_shard{i}"
            items.append(self._make_queue_item(shard_cfg, start_at))

        self._queue.extend(items)
        self._save_queue()
        self._audit("enqueue", group, actor, source="queue",
                    detail=f"distributed shards={shards}")
        await self._bc.broadcast(self.queue_msg())
        return {"ok": True, "group": group, "shards": shards, "items": items}

    async def remove_queue_item(self, item_id: str) -> dict:
        self._queue = [q for q in self._queue if q["id"] != item_id]
        self._save_queue()
        await self._bc.broadcast(self.queue_msg())
        return {"ok": True}

    async def clear_queue(self) -> dict:
        self._queue = []
        self._save_queue()
        await self._bc.broadcast(self.queue_msg())
        return {"ok": True}

    def _next_ready_item(self) -> dict | None:
        """First pending item runnable by the local server worker.

        Items pinned to a specific remote device (target not in {None, "any",
        "server"}) are skipped so only their device picks them up via lease.
        A future-scheduled head-of-queue item holds the queue."""
        now = time.time()
        for q in self._queue:
            if q.get("status") != "pending":
                continue
            start_at = q.get("start_at")
            if start_at and start_at > now:
                return None  # next-in-line is scheduled later; keep order, wait for it
            if q.get("target") not in (None, "any", "server"):
                continue  # pinned to a remote device — leave it for that worker
            return q
        return None

    async def _scheduler_task(self) -> None:
        """Launch the next ready queue item locally whenever the server is idle.

        Skipped entirely when the local worker is disabled — then only registered
        guest devices consume the queue (via lease)."""
        while True:
            await asyncio.sleep(1.0)
            if not self._settings.enable_local_worker:
                continue  # server is a pure coordinator; guests train the queue
            # Fill every free local slot from the queue before sleeping again.
            while len(self._local_active()) < self._settings.local_slots:
                item = self._next_ready_item()
                if item is None:
                    break
                item["status"] = "running"
                self._save_queue()
                await self._bc.broadcast(self.queue_msg())
                result = await self.launch(item["config"], actor=f"queue:{item['id']}")
                if result.get("ok"):
                    self._queue = [q for q in self._queue if q["id"] != item["id"]]
                else:
                    item["status"] = "failed"
                    item["message"] = result.get("message", "launch failed")
                    log.warning("Queued item %s failed to launch: %s", item["id"], item["message"])
                    self._save_queue()
                    await self._bc.broadcast(self.queue_msg())
                    break  # don't spin retrying a failing item this tick
                self._save_queue()
                await self._bc.broadcast(self.queue_msg())

    # ── devices / distributed workers ────────────────────────────────────────
    def devices_msg(self) -> dict:
        return {"type": "devices", "devices": self._devices.list_public()}

    def _prepare_run(self, config: dict, created_by: str) -> tuple[str, Path, ModelConfig]:
        """Allocate a unique run name and write its config.json + launch meta.json.

        Shared by the local launcher and the remote-lease path so a leased job lands
        in the same on-disk shape as a server-trained one (the worker uploads its
        checkpoints into ``checkpoints/<run_name>/`` afterwards)."""
        stage = str(config.get("stage") or "APPROACH_STATIC_BALL")
        name = (config.get("name") or "").strip()
        version = (config.get("version") or "1").strip() or "1"
        if name:
            run_name = self._unique_run_name(run_name_for(name, version))
        else:
            run_name = self._unique_run_name(f"{stage}_seed{int(config.get('seed') or 0)}")
        dr = config.get("domain_rand")
        model_cfg = ModelConfig(
            name=name or run_name,
            version=version,
            stage=stage,
            n_envs=int(config.get("n_envs") or 16),
            seed=int(config.get("seed") or 0),
            domain_rand=True if dr is None else bool(dr),
            stop=config.get("stop"),
            hyperparams=config.get("hyperparams") or {},
            net_arch=config.get("net_arch") or [64, 64],
            reward_weights=config.get("reward_weights") or {},
            save_step_checkpoints=bool(config.get("save_step_checkpoints", False)),
            resume_from=config.get("resume_from"),
        )
        ckpt_root = self._base_dir / "checkpoints"
        config_path = write_config(ckpt_root, run_name, model_cfg.model_dump())
        write_meta(ckpt_root, run_name, launch_meta(model_cfg, run_name, created_by))
        return run_name, config_path, model_cfg

    async def lease_for_worker(
        self, device_id: str, *, cores: int | None = None, capacity: int | None = None
    ) -> dict | None:
        """Atomically hand the next ready *train* job to a guest device.

        Returns the run name, the full ModelConfig and the resolved stop-condition
        CLI args for the worker to pass to ``scripts/train.py``; ``None`` if nothing
        is ready. Play jobs are skipped (they run locally); a future-scheduled item
        is held so its slot is preserved.

        ``cores``/``capacity`` are the worker's self-reported capability (recorded on
        every poll). A device is only handed work up to its effective concurrency cap
        (the admin slider value, or the reported capacity when unset)."""
        async with self._lease_lock:
            now = time.time()
            # Every poll — even when the queue is empty and nothing is leased — is a
            # check-in. Touch first so an idle worker stays "online"; otherwise a
            # happily-polling device that never gets a job reads as "never online".
            self._devices.touch(device_id, cores=cores, capacity=capacity)
            await self._bc.broadcast(self.devices_msg())
            pub = self._devices.public(device_id) if device_id else None
            dev = pub["name"] if pub else "device"
            # Per-device concurrency cap: never lease past what the device may run at
            # once. Held jobs (current_jobs) are tracked under the same lease lock, so
            # this count is consistent with the add_job touch below.
            effective = pub["effective_slots"] if pub else 1
            if pub and len(pub["current_jobs"]) >= effective:
                return None  # at capacity — hold further work for this device
            for q in self._queue:
                if q.get("status") != "pending":
                    continue
                start_at = q.get("start_at")
                if start_at and start_at > now:
                    return None  # next-in-line is scheduled later; hold the queue
                if str(q.get("mode", "train")) != "train":
                    continue  # play jobs are local-only
                target = q.get("target")
                if target not in (None, "any") and target != device_id:
                    continue  # pinned to the server or another device
                run_name, _path, model_cfg = self._prepare_run(q["config"], created_by=dev)
                stop_args, _meta = self._resolve_stop(q["config"].get("stop"),
                                                      model_cfg.stop.value if model_cfg.stop else 200_000)
                if stop_args is None:
                    stop_args = ["--timesteps", "200000"]
                q["status"] = "running"
                q["leased_by"] = device_id
                q["lease_deadline"] = now + LEASE_TTL
                q["run_name"] = run_name
                self._save_queue()
                self._devices.touch(device_id, add_job=run_name)
                # Track the leased run alongside local runs so the UI shows/manages it.
                self._active[run_name] = {
                    "run_name": run_name, "device": device_id, "run_type": "train",
                    "state": "running", "phase": "leased", "num_timesteps": 0,
                    "stage": model_cfg.stage, "model_name": model_cfg.name,
                    "model_version": model_cfg.version, "n_envs": model_cfg.n_envs,
                    "seed": model_cfg.seed, "domain_rand": model_cfg.domain_rand,
                    "viz": bool(q["config"].get("viz")),
                    "started_at": now, "cancel_requested": False,
                    "dist_group": q["config"].get("dist_group"),
                }
                self._audit("launch", run_name, dev, source="device")
                await self._bc.broadcast(self.queue_msg())
                await self._bc.broadcast(self.devices_msg())
                await self._bc.broadcast(self.models_msg())
                await self._bc.broadcast(self.active_runs_msg())
                job = {
                    "run_name": run_name,
                    "config": model_cfg.model_dump(),
                    "stop_args": stop_args,
                    # Headless by default; the launcher's opt-in rides along to the worker.
                    "viz": bool(q["config"].get("viz")),
                    # Tell the worker its current concurrency budget so it sizes how many
                    # jobs it leases in parallel (live — picks up slider changes).
                    "max_slots": effective,
                }
                # Federated shard: tell the worker how to reach the sync group.
                if q["config"].get("dist_group"):
                    job["fed"] = {
                        "group": q["config"]["dist_group"],
                        "every": int(q["config"].get("dist_sync_every") or 50_000),
                        "shards": int(q["config"].get("dist_shards") or 1),
                    }
                return job
            return None

    async def worker_heartbeat(self, device_id: str, run_name: str) -> dict:
        """Renew a lease and tell the worker whether its run has been asked to stop."""
        cancel = False
        async with self._lease_lock:
            for q in self._queue:
                if (q.get("run_name") == run_name and q.get("leased_by") == device_id
                        and q.get("status") == "running"):
                    q["lease_deadline"] = time.time() + LEASE_TTL
                    cancel = bool(q.get("cancel_requested"))
                    self._save_queue()
                    break
        self._devices.touch(device_id, add_job=run_name)
        entry = self._active.get(run_name)
        if entry is not None and cancel:
            entry["state"] = "stopping"
        await self._bc.broadcast(self.devices_msg())
        # Echo the current concurrency budget so an already-busy worker scales up/down
        # when the admin moves the slider mid-job, without waiting for the next lease.
        effective = self._devices.public(device_id)["effective_slots"] if device_id else 1
        return {"ok": True, "cancel": cancel, "max_slots": effective}

    def _is_run_leased_by(self, run: str, device_id: str) -> bool:
        """True iff `run` is an active lease currently held by `device_id`.

        This is the authorization gate for worker writes (upload/complete): a device
        token may only touch a run it is presently leasing, never another device's
        run or a server-trained run."""
        return any(
            q.get("run_name") == run and q.get("leased_by") == device_id
            and q.get("status") == "running"
            for q in self._queue
        )

    async def worker_complete(
        self, device_id: str, run_name: str, status: str,
        timesteps_trained: int | None = None, best_eval: float | None = None,
    ) -> dict:
        """A guest finished (or failed) a leased job: finalise metadata + free the slot."""
        if not self._is_safe_name(run_name):
            return {"ok": False, "message": "Invalid run name."}
        if not self._is_run_leased_by(run_name, device_id):
            return {"ok": False, "message": "Run is not leased by this device."}
        ckpt_root = self._base_dir / "checkpoints"
        meta = {"status": status}
        if timesteps_trained is not None:
            meta["timesteps_trained"] = int(timesteps_trained)
        if best_eval is not None:
            meta["best_eval"] = float(best_eval)
        write_meta(ckpt_root, run_name, meta)
        async with self._lease_lock:
            self._queue = [
                q for q in self._queue
                if not (q.get("run_name") == run_name and q.get("leased_by") == device_id)
            ]
            self._save_queue()
        gone = self._active.pop(run_name, None)
        if gone is not None:
            self._drop_dist_group_if_idle(gone.get("dist_group"))
        self._devices.touch(device_id, remove_job=run_name)
        self._audit("complete", run_name, device_id, source="device", detail=f"status={status}")
        await self._bc.broadcast(self.queue_msg())
        await self._bc.broadcast(self.devices_msg())
        await self._bc.broadcast(self.models_msg())
        await self._bc.broadcast(self.runs_msg())
        await self._bc.broadcast(self.active_runs_msg())
        return {"ok": True}

    async def save_uploaded_checkpoint(
        self, device_id: str, run: str, filename: str, data: bytes
    ) -> dict:
        """Store a checkpoint uploaded by the guest worker that holds the run's lease.

        Authorization: only the device currently leasing `run` may upload, and only
        ``.zip`` checkpoints — so a worker cannot overwrite the server-authored
        ``meta.json``/``config.json`` sidecars or write into another device's run."""
        if not self._is_safe_name(run) or not self._is_safe_name(filename):
            return {"ok": False, "message": "Invalid reference."}
        if not filename.endswith(".zip"):
            return {"ok": False, "message": "Only .zip checkpoints may be uploaded."}
        if not self._is_run_leased_by(run, device_id):
            return {"ok": False, "message": "Run is not leased by this device."}
        run_dir = self._base_dir / "checkpoints" / run
        run_dir.mkdir(parents=True, exist_ok=True)
        try:
            (run_dir / filename).write_bytes(data)
        except OSError as exc:
            return {"ok": False, "message": str(exc)}
        await self._bc.broadcast(self.models_msg())
        await self._bc.broadcast(self.runs_msg())
        return {"ok": True}

    async def _lease_reaper_task(self) -> None:
        """Return jobs whose remote worker stopped heart-beating back to the queue."""
        while True:
            await asyncio.sleep(5.0)
            now = time.time()
            changed = False
            async with self._lease_lock:
                for q in self._queue:
                    leased_by = q.get("leased_by")
                    if (q.get("status") == "running" and leased_by
                            and q.get("lease_deadline", 0) < now):
                        q["attempts"] = q.get("attempts", 0) + 1
                        run_name = q.pop("run_name", None)
                        q.pop("leased_by", None)
                        q.pop("lease_deadline", None)
                        q.pop("cancel_requested", None)
                        if q["attempts"] >= MAX_LEASE_ATTEMPTS:
                            q["status"] = "failed"
                            q["message"] = "Lease expired (worker went away)."
                        else:
                            q["status"] = "pending"
                        if run_name:
                            gone = self._active.pop(run_name, None)
                            if gone is not None:
                                self._drop_dist_group_if_idle(gone.get("dist_group"))
                        self._devices.touch(leased_by, remove_job=run_name)
                        log.info("Reclaimed lease for %s from device %s", run_name, leased_by)
                        changed = True
                if changed:
                    self._save_queue()
            if changed:
                await self._bc.broadcast(self.queue_msg())
                await self._bc.broadcast(self.devices_msg())
                await self._bc.broadcast(self.active_runs_msg())

    # ── federated distributed training (FedAvg coordinator) ──────────────────────
    # How long a round waits for stragglers before averaging with whatever arrived, so a
    # dead shard can't wedge the group forever (the live shards just sync among themselves).
    DIST_ROUND_TIMEOUT = 120.0

    async def dist_push(
        self, group: str, rnd: int, shard: str, shards: int, data: bytes, weight: float = 1.0
    ) -> dict:
        """Store one shard's weights for a round; average once the quorum has arrived.

        Each push carries the shard's ``weight`` (its env count) so the average is
        FedAvg-weighted — a 32-env shard pulls the shared model 4× as hard as an 8-env one."""
        from bucky import fedavg

        async with self._dist_lock:
            g = self._dist.setdefault(group, {"rounds": {}})
            # Forget rounds well behind the current one — sync state is transient.
            for old in [r for r in g["rounds"] if r < rnd - 1]:
                g["rounds"].pop(old, None)
            rs = g["rounds"].setdefault(
                rnd, {"pushes": {}, "averaged": None, "first_at": time.time()}
            )
            if rs["averaged"] is None:
                # Store (arrays, weight) per shard so the mean can be env-weighted.
                rs["pushes"][shard] = (fedavg.from_npz_bytes(data), float(weight))
                if len(rs["pushes"]) >= max(1, shards):
                    rs["averaged"] = fedavg.to_npz_bytes(self._average_round(rs["pushes"]))
                    rs["pushes"] = {}  # free the per-shard copies once averaged
        return {"ok": True}

    @staticmethod
    def _average_round(pushes: dict) -> dict:
        """Weighted FedAvg over a round's collected shard pushes (arrays + weights)."""
        from bucky import fedavg

        states = [arrays for arrays, _w in pushes.values()]
        weights = [w for _a, w in pushes.values()]
        return fedavg.average_arrays(states, weights=weights)

    async def dist_pull(self, group: str, rnd: int) -> bytes | None:
        """Return the round's averaged weights, or None if not ready yet.

        Falls back to averaging whatever arrived once the round has waited past the
        straggler timeout, so the live shards aren't blocked by a missing one."""
        from bucky import fedavg

        async with self._dist_lock:
            g = self._dist.get(group)
            if not g:
                return None
            rs = g["rounds"].get(rnd)
            if not rs:
                return None
            if rs["averaged"] is not None:
                return rs["averaged"]
            if rs["pushes"] and time.time() - rs["first_at"] > self.DIST_ROUND_TIMEOUT:
                rs["averaged"] = fedavg.to_npz_bytes(self._average_round(rs["pushes"]))
                rs["pushes"] = {}
                return rs["averaged"]
            return None

    def _drop_dist_group_if_idle(self, group: str | None) -> None:
        """Free a group's sync buffers once none of its shards are still active."""
        if not group:
            return
        if any(e.get("dist_group") == group for e in self._active.values()):
            return
        self._dist.pop(group, None)

    def runs_msg(self) -> dict:
        """List existing runs and their .zip checkpoints (for the resume picker)."""
        runs = []
        ckpt_root = self._base_dir / "checkpoints"
        if ckpt_root.is_dir():
            for d in sorted(ckpt_root.iterdir()):
                if d.is_dir():
                    files = sorted(f.name for f in d.glob("*.zip"))
                    if files:
                        runs.append({"run": d.name, "checkpoints": files})
        return {"type": "runs", "runs": runs}

    # ── models (admin panel) ─────────────────────────────────────────────────
    def models_msg(self) -> dict:
        """Enriched model list (name/version/config/best-eval) for the admin panel."""
        return {
            "type": "models",
            "models": list_models(self._base_dir / "checkpoints", self._base_dir / "runs"),
        }

    async def delete_model(self, run: str, actor: str | None = None) -> dict:
        """Delete a whole run directory (all checkpoints + sidecars)."""
        if not self._is_safe_name(run):
            return {"ok": False, "message": "Invalid run name."}
        run_dir = self._base_dir / "checkpoints" / run
        if not run_dir.is_dir():
            return {"ok": False, "message": "Run not found."}
        import shutil
        shutil.rmtree(run_dir, ignore_errors=True)
        # Drop the (now orphaned) TensorBoard logs too, best-effort.
        shutil.rmtree(self._base_dir / "runs" / run, ignore_errors=True)
        self._audit("delete", run, actor, source="server")
        await self._bc.broadcast(self.models_msg())
        await self._bc.broadcast(self.runs_msg())
        return {"ok": True}

    async def delete_checkpoint(self, run: str, checkpoint: str) -> dict:
        """Delete a single checkpoint file within a run."""
        if not self._is_safe_name(run) or not self._is_safe_name(checkpoint):
            return {"ok": False, "message": "Invalid reference."}
        path = self._base_dir / "checkpoints" / run / checkpoint
        if not path.is_file():
            return {"ok": False, "message": "Checkpoint not found."}
        try:
            path.unlink()
        except OSError as exc:
            return {"ok": False, "message": str(exc)}
        await self._bc.broadcast(self.models_msg())
        await self._bc.broadcast(self.runs_msg())
        return {"ok": True}

    async def prune_step_checkpoints(self, run: str) -> dict:
        """Delete the periodic ``*_steps.zip`` snapshots in a run, keeping the
        best/final/opponent checkpoints."""
        if not self._is_safe_name(run):
            return {"ok": False, "message": "Invalid run name."}
        run_dir = self._base_dir / "checkpoints" / run
        if not run_dir.is_dir():
            return {"ok": False, "message": "Run not found."}
        removed = 0
        for f in run_dir.glob("*_steps.zip"):
            try:
                f.unlink()
                removed += 1
            except OSError:
                pass
        await self._bc.broadcast(self.models_msg())
        await self._bc.broadcast(self.runs_msg())
        return {"ok": True, "removed": removed}

    def checkpoint_path(self, run: str, checkpoint: str) -> Path | None:
        """Resolve a checkpoint to an absolute path, or None if unsafe/missing."""
        if not self._is_safe_name(run) or not self._is_safe_name(checkpoint):
            return None
        path = self._base_dir / "checkpoints" / run / checkpoint
        return path if path.is_file() else None

    def status_msg(self, message: str | None = None) -> dict:
        """Legacy single-run status: surfaces a *primary* run (a local server run if any,
        else the first active run) so the existing viewer/controls keep working while the
        ``active_runs`` message drives the multi-run management UI."""
        local = self._local_active()
        primary = local[0] if local else next(iter(self._active.values()), None)
        if primary is None:
            msg = {"type": "status", "state": "idle",
                   "trainer_connected": self._ingest_conns > 0}
        else:
            msg = {"type": "status", "trainer_connected": self._ingest_conns > 0,
                   **self._public_run(primary)}
            msg.setdefault("state", "running")
        if message is not None:
            msg["message"] = message
        return msg
