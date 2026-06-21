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
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

from bucky.curriculum import Stage

from .broadcast import Broadcaster
from .config import Settings
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

log = logging.getLogger(__name__)

VALID_STAGES = {s.value for s in Stage}


class JobManager:
    """Spawns one job at a time and relays its frames/status to the broadcaster."""

    def __init__(
        self,
        broadcaster: Broadcaster,
        settings: Settings,
        base_dir: str | None = None,
    ) -> None:
        self._bc = broadcaster
        self._settings = settings
        # app/jobs.py -> parent is app/, parent.parent is the backend root (scripts/ + checkpoints/)
        self._base_dir = Path(base_dir) if base_dir else Path(__file__).resolve().parent.parent
        self._launch_lock = asyncio.Lock()
        # Unified registry of in-flight runs, keyed by run_name. Covers both the server's
        # own local trainers (device == "server", entry carries a live ``proc``) and runs
        # leased to remote guest devices (device == device_id, no ``proc``). Each entry is
        # a plain dict broadcast to the UI (minus the unpicklable ``proc``). Replaces the
        # old single-run ``_proc``/``_state``/``_run_meta`` so several runs coexist.
        self._active: dict[str, dict] = {}
        # Ref-count of connected local trainer ingest sockets → ``trainer_connected``.
        self._ingest_conns = 0
        self._tasks: list[asyncio.Task] = []

        # Persisted job queue: items run back-to-back when the manager is idle, each
        # with an optional `start_at` (epoch) so runs can be scheduled while away.
        self._state_dir = self._base_dir / "state"
        self._queue_path = self._state_dir / "queue.json"
        self._queue: list[dict] = []
        self._queue_seq = 0
        self._load_queue()

        # Distributed training: registered guest devices lease train jobs from the
        # same queue. A lock serialises lease/heartbeat/complete against the local
        # scheduler so two workers can't claim the same item.
        self._devices = DeviceRegistry(self._state_dir)
        self._lease_lock = asyncio.Lock()

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
        for entry in self._local_active():
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

    # ── control ──────────────────────────────────────────────────────────────────
    async def launch(self, cfg: dict) -> dict:
        # A distributed run isn't a single local job — fan it out into shard queue items
        # that workers (incl. the local one) pick up and train together via FedAvg.
        dist = cfg.get("distributed")
        if str(cfg.get("mode", "train")) == "train" and dist and int(dist.get("shards") or 1) > 1:
            return await self._enqueue_distributed(cfg, dist, None)

        async with self._launch_lock:
            # Concurrency cap: the in-process worker runs up to ``local_slots`` runs at
            # once (each its own subprocess). Both training and match runs count.
            if len(self._local_active()) >= self._settings.local_slots:
                return self._reject("All local training slots are busy.")

            if str(cfg.get("mode", "train")) == "play":
                return await self._launch_play(cfg)

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
            await self._bc.broadcast(self.status_msg())
            await self._bc.broadcast(self.active_runs_msg())
            return {"ok": True, "message": "launched", "status": self.status_msg()}

    async def _launch_play(self, cfg: dict) -> dict:
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

        args = [
            sys.executable, "scripts/play.py",
            "--policy-a", str(resolved["policy_a"][0]),
            "--policy-b", str(resolved["policy_b"][0]),
            "--seed", str(seed),
            "--stream-url", self._ingest_url(run_name),
        ]
        if domain_rand:
            args.append("--domain-rand")

        entry = {
            "run_name": run_name, "device": "server", "mode": "play",
            "run_type": "match", "state": "launching",
            "policy_a": resolved["policy_a"][1], "policy_b": resolved["policy_b"][1],
            "seed": seed, "domain_rand": domain_rand,
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
        await self._bc.broadcast(self.status_msg())
        await self._bc.broadcast(self.active_runs_msg())
        return {"ok": True, "message": "launched", "status": self.status_msg()}

    async def kill(self, run_name: str | None = None) -> dict:
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
            await self._bc.broadcast(self.status_msg())
            await self._bc.broadcast(self.active_runs_msg())
            await asyncio.get_running_loop().run_in_executor(None, self._terminate, proc)
            return {"ok": True, "message": "stopping"}

        # Remote run: request cancellation; keep the lease so the worker can save first.
        entry["cancel_requested"] = True
        entry["state"] = "stopping"
        async with self._lease_lock:
            for q in self._queue:
                if q.get("run_name") == run_name:
                    q["cancel_requested"] = True
            self._save_queue()
        await self._bc.broadcast(self.active_runs_msg())
        return {"ok": True, "message": "cancel requested"}

    # ── subprocess helpers ─────────────────────────────────────────────────────
    def _ingest_url(self, run_name: str) -> str:
        return (
            f"ws://localhost:{self._settings.port}/api/ingest"
            f"?token={self._settings.ingest_token}&run={run_name}"
        )

    def _spawn(self, args: list[str]) -> tuple[subprocess.Popen | None, str | None]:
        """Start a job subprocess; return ``(proc, None)`` or ``(None, error_string)``."""
        try:
            # start_new_session=True puts the trainer (and its SubprocVecEnv workers)
            # in their own process group, so kill can signal the whole group and a
            # terminal Ctrl-C on the server won't leak into the trainer.
            proc = subprocess.Popen(
                args, cwd=str(self._base_dir), start_new_session=True
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
                    exited.append((run_name, code))
            if exited:
                for run_name, code in exited:
                    log.info("Job %s exited with code %s", run_name, code)
                    await self._bc.broadcast({"type": "run_exited", "run": run_name, "code": code})
                await self._bc.broadcast(self.status_msg())
                await self._bc.broadcast(self.active_runs_msg())
                await self._bc.broadcast(self.runs_msg())  # new checkpoints are now resumable
                await self._bc.broadcast(self.models_msg())  # admin panel: refreshed metadata

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

    def _load_queue(self) -> None:
        try:
            if self._queue_path.is_file():
                data = json.loads(self._queue_path.read_text())
                self._queue = [q for q in data.get("items", []) if isinstance(q, dict)]
                self._queue_seq = int(data.get("seq", 0))
                # Anything caught mid-launch when the server stopped goes back to pending.
                for q in self._queue:
                    if q.get("status") == "running":
                        q["status"] = "pending"
        except Exception:  # noqa: BLE001 — a corrupt queue file must not crash startup
            log.warning("Could not read queue %s; starting with an empty queue", self._queue_path)
            self._queue = []

    def _save_queue(self) -> None:
        try:
            self._state_dir.mkdir(parents=True, exist_ok=True)
            self._queue_path.write_text(
                json.dumps({"seq": self._queue_seq, "items": self._queue}, indent=2)
            )
        except Exception as exc:  # noqa: BLE001
            log.warning("Could not persist queue: %s", exc)

    def _next_queue_id(self) -> str:
        self._queue_seq += 1
        return f"q{self._queue_seq}"

    async def enqueue(self, cfg: dict, start_at: float | None) -> dict:
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

        # Distributed run: fan out into N shard items that train together via FedAvg.
        dist = cfg.get("distributed")
        if mode == "train" and dist and int(dist.get("shards") or 1) > 1:
            return await self._enqueue_distributed(cfg, dist, start_at)

        item = self._make_queue_item(cfg, start_at)
        self._queue.append(item)
        self._save_queue()
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

    async def _enqueue_distributed(self, cfg: dict, dist: dict, start_at: float | None) -> dict:
        """Split one run into N FedAvg shards sharing a sync group.

        Each shard is an ordinary train job (its own run_name, checkpoints and a
        per-shard seed for sample diversity) carrying the group's sync parameters. They
        average policy weights through ``/api/dist`` every ``sync_every`` steps, so the
        fleet trains one converging model with N× the experience."""
        shards = max(2, int(dist.get("shards") or 2))
        sync_every = max(1, int(dist.get("sync_every") or 50_000))
        base = (cfg.get("name") or cfg.get("stage") or "dist").strip() or "dist"
        group = self._unique_run_name(f"{base}_dist")
        self._dist.setdefault(group, {"rounds": {}})

        base_seed = int(cfg.get("seed") or 0)
        base_name = (cfg.get("name") or "").strip()
        items = []
        for i in range(shards):
            shard_cfg = {k: v for k, v in cfg.items() if k != "distributed"}
            shard_cfg["dist_group"] = group
            shard_cfg["dist_sync_every"] = sync_every
            shard_cfg["dist_shards"] = shards
            shard_cfg["seed"] = base_seed + i  # diverse samples per shard
            if base_name:
                shard_cfg["name"] = f"{base_name}_shard{i}"
                shard_cfg["version"] = cfg.get("version") or "1"
            else:
                shard_cfg["name"] = f"{group}_shard{i}"
            items.append(self._make_queue_item(shard_cfg, start_at))

        self._queue.extend(items)
        self._save_queue()
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
                result = await self.launch(item["config"])
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

    async def lease_for_worker(self, device_id: str) -> dict | None:
        """Atomically hand the next ready *train* job to a guest device.

        Returns the run name, the full ModelConfig and the resolved stop-condition
        CLI args for the worker to pass to ``scripts/train.py``; ``None`` if nothing
        is ready. Play jobs are skipped (they run locally); a future-scheduled item
        is held so its slot is preserved."""
        async with self._lease_lock:
            now = time.time()
            # Every poll — even when the queue is empty and nothing is leased — is a
            # check-in. Touch first so an idle worker stays "online"; otherwise a
            # happily-polling device that never gets a job reads as "never online".
            self._devices.touch(device_id)
            await self._bc.broadcast(self.devices_msg())
            dev = self._devices.public(device_id)["name"] if device_id else "device"
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
        return {"ok": True, "cancel": cancel}

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

    async def dist_push(self, group: str, rnd: int, shard: str, shards: int, data: bytes) -> dict:
        """Store one shard's weights for a round; average once the quorum has arrived."""
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
                rs["pushes"][shard] = fedavg.from_npz_bytes(data)
                if len(rs["pushes"]) >= max(1, shards):
                    rs["averaged"] = fedavg.to_npz_bytes(
                        fedavg.average_arrays(list(rs["pushes"].values()))
                    )
                    rs["pushes"] = {}  # free the per-shard copies once averaged
        return {"ok": True}

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
                rs["averaged"] = fedavg.to_npz_bytes(
                    fedavg.average_arrays(list(rs["pushes"].values()))
                )
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

    async def delete_model(self, run: str) -> dict:
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
