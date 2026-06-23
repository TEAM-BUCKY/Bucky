#!/usr/bin/env python
"""Bucky guest training worker.

Run this on any trusted machine (e.g. your local PC) to share the central server's
training load. The worker leases training jobs from the server's shared queue, runs
``scripts/train.py`` locally, streams live frames back to the server (so the browser
can watch this device train), and uploads the resulting checkpoints — so everything
ends up centrally on the server even though the compute happened here.

Setup:
    1. In the web admin panel (Devices tab) register a device and copy its token.
    2. Run, from the backend/ directory:

        uv run python scripts/worker.py \
            --server https://your-bucky-host \
            --token  <device-token>

       (or set BUCKY_SERVER / BUCKY_DEVICE_TOKEN in the environment.)

The worker polls for work, so it can be left running; it picks up jobs as they are
queued and is safe to Ctrl-C between jobs.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import requests

BASE_DIR = Path(__file__).resolve().parent.parent  # backend/
POLL_INTERVAL = 5.0       # seconds between lease attempts when the queue is empty
# Heartbeats both renew the lease and carry the server's stop signal back, so keep them
# brisk — this also bounds how quickly a remote "Stop" reaches the trainer.
HEARTBEAT_INTERVAL = 5.0


def _api_base(server: str) -> str:
    base = server.rstrip("/")
    if not base.endswith("/api"):
        base += "/api"
    return base


def _ws_ingest(api_base: str, run: str) -> str:
    # The device token is sent as a handshake header (--stream-token), not in the
    # URL, so it never reaches proxy/access logs. Only the run tag rides the query.
    ws = api_base.replace("https://", "wss://").replace("http://", "ws://")
    return f"{ws}/ingest?run={run}"


class Worker:
    def __init__(self, server: str, token: str, slots: int | None = None,
                 device: str = "cpu") -> None:
        self.api = _api_base(server)
        self.token = token
        # Self-reported capability: raw core count (the slider's ceiling in the UI) and a
        # recommended concurrent-job count. A training job grabs ~16 env-workers, so beyond
        # a couple of jobs the CPU oversubscribes — cores//8 is a conservative default the
        # admin can override upward with the slider, up to the full core count.
        self.cores = os.cpu_count() or 1
        self.capacity = max(1, self.cores // 8)
        self.headers = {
            "X-Device-Token": token,
            "X-Worker-Cores": str(self.cores),
            "X-Worker-Capacity": str(self.capacity),
        }
        # Local hard ceiling on concurrency (machine owner's safety cap). Defaults to the
        # core count, i.e. effectively "no extra cap" — the server/slider is the primary
        # control. The server's per-device budget is the live authority below this.
        self.slots_ceiling = max(1, slots) if slots else self.cores
        # Concurrency budget last reported by the server (None until the first lease/beat).
        self._server_slots: int | None = None
        # Torch device for the local trainer. Defaults to cpu — for this tiny [64,64] MLP
        # the GPU's per-step host<->device copies usually outweigh its compute, so cuda is
        # opt-in per machine (this machine owns the GPU, so the worker decides).
        self.device = device

    def _effective_slots(self) -> int:
        """How many jobs to run at once: the server's budget, capped by the local ceiling."""
        budget = self._server_slots if self._server_slots else self.capacity
        return max(1, min(self.slots_ceiling, budget))

    # ── server calls ──────────────────────────────────────────────────────────
    def lease(self) -> dict | None:
        r = requests.post(f"{self.api}/worker/lease", headers=self.headers, timeout=30)
        if r.status_code == 204:
            return None
        r.raise_for_status()
        return r.json()

    def heartbeat(self, run_name: str) -> dict | None:
        """Renew the lease; return the server's response ({'cancel': bool}) or None."""
        try:
            r = requests.post(f"{self.api}/worker/heartbeat", headers=self.headers,
                              json={"run_name": run_name}, timeout=15)
            return r.json() if r.ok else None
        except (requests.RequestException, ValueError):
            return None  # transient; the lease has slack before it expires

    def complete(self, run_name: str, status: str, steps, best_eval) -> None:
        requests.post(f"{self.api}/worker/complete", headers=self.headers, json={
            "run_name": run_name, "status": status,
            "timesteps_trained": steps, "best_eval": best_eval,
        }, timeout=30)

    def upload(self, run_name: str, path: Path) -> None:
        with path.open("rb") as fh:
            requests.post(
                f"{self.api}/models/{run_name}/upload",
                headers=self.headers,
                files={"file": (path.name, fh, "application/octet-stream")},
                data={"filename": path.name},
                timeout=300,
            ).raise_for_status()

    # ── job execution ─────────────────────────────────────────────────────────
    def run_job(self, job: dict) -> None:
        run_name = job["run_name"]
        config = job["config"]
        stop_args = job.get("stop_args") or ["--timesteps", "200000"]
        print(f"[worker] leased job {run_name}", flush=True)

        # Write the ModelConfig locally for train.py (the server keeps its own copy).
        run_dir = BASE_DIR / "checkpoints" / run_name
        run_dir.mkdir(parents=True, exist_ok=True)
        cfg_path = run_dir / "config.json"
        cfg_path.write_text(json.dumps(config, indent=2))

        ingest = _ws_ingest(self.api, run_name)
        args = [
            sys.executable, "scripts/train.py",
            "--config", str(cfg_path),
            "--run-name", run_name,
            "--stream-url", ingest,
            "--stream-token", self.token,
            "--device", self.device,
            *stop_args,
        ]
        # Headless by default; animate the field only when the launch opted in.
        if job.get("viz"):
            args.append("--viz")
        # Federated shard: sync weights with its group through the server coordinator.
        fed = job.get("fed")
        if fed:
            args += [
                "--fed-server", self.api,
                "--fed-token", self.token,
                "--fed-group", str(fed["group"]),
                "--fed-every", str(fed.get("every", 50000)),
                "--fed-shards", str(fed.get("shards", 1)),
            ]
        proc = subprocess.Popen(args, cwd=str(BASE_DIR), start_new_session=True)

        # Heartbeat in the background so the server keeps our lease while we train. The
        # heartbeat response also carries the server's stop signal: on cancel we SIGINT the
        # trainer (it checkpoints before exiting) and keep beating so the lease survives
        # the save. train.py exits 0 after an interrupt, so its checkpoint still uploads.
        stop_hb = threading.Event()
        cancelled = threading.Event()

        def _beat() -> None:
            while not stop_hb.wait(HEARTBEAT_INTERVAL):
                resp = self.heartbeat(run_name)
                if resp and resp.get("max_slots"):
                    # Pick up slider changes mid-job so we scale up/down without a restart.
                    self._server_slots = int(resp["max_slots"])
                if resp and resp.get("cancel") and not cancelled.is_set():
                    cancelled.set()
                    print(f"[worker] stop requested for {run_name}; signalling trainer", flush=True)
                    try:
                        proc.send_signal(signal.SIGINT)
                    except ProcessLookupError:
                        pass

        hb = threading.Thread(target=_beat, daemon=True)
        hb.start()
        try:
            code = proc.wait()
        finally:
            stop_hb.set()

        if cancelled.is_set():
            status = "stopped"
        else:
            status = "done" if code == 0 else "failed"
        steps, best = self._read_local_meta(run_dir)
        # A clean exit (incl. an interrupt-and-save) leaves uploadable checkpoints.
        if code == 0:
            self._upload_checkpoints(run_name, run_dir)
        try:
            self.complete(run_name, status, steps, best)
        except requests.RequestException as exc:
            print(f"[worker] complete failed for {run_name}: {exc}", flush=True)
        print(f"[worker] job {run_name} finished ({status})", flush=True)

    def _read_local_meta(self, run_dir: Path):
        try:
            meta = json.loads((run_dir / "meta.json").read_text())
            return meta.get("timesteps_trained"), meta.get("best_eval")
        except Exception:  # noqa: BLE001
            return None, None

    def _upload_checkpoints(self, run_name: str, run_dir: Path) -> None:
        for zip_path in sorted(run_dir.glob("*.zip")):
            try:
                self.upload(run_name, zip_path)
                print(f"[worker] uploaded {zip_path.name}", flush=True)
            except requests.RequestException as exc:
                print(f"[worker] upload failed for {zip_path.name}: {exc}", flush=True)

    def _safe_run(self, job: dict) -> None:
        try:
            self.run_job(job)
        except Exception as exc:  # noqa: BLE001 — never let one bad job kill the worker
            print(f"[worker] job error: {exc}", flush=True)

    # ── main loop ─────────────────────────────────────────────────────────────
    def serve_forever(self) -> None:
        print(f"[worker] polling {self.api} for jobs… "
              f"({self.cores} cores, recommend {self.capacity} slot(s); "
              f"local ceiling {self.slots_ceiling}; server sets the live cap)",
              flush=True)
        active: dict[str, threading.Thread] = {}
        while True:
            # Reap finished jobs so their slots free up.
            for rn in [rn for rn, t in active.items() if not t.is_alive()]:
                del active[rn]

            # Concurrency is the server's live budget, capped by the local ceiling. The
            # server also refuses to lease past it, so this is mostly to avoid pointless
            # polling once we're full.
            if len(active) >= self._effective_slots():
                time.sleep(POLL_INTERVAL)
                continue

            try:
                job = self.lease()
            except requests.RequestException as exc:
                print(f"[worker] lease error: {exc}; retrying", flush=True)
                time.sleep(POLL_INTERVAL)
                continue
            if job is None:
                time.sleep(POLL_INTERVAL)
                continue

            # The lease carries our current concurrency budget — adopt it so we fill up
            # to (or back off to) whatever the admin slider currently allows.
            if job.get("max_slots"):
                self._server_slots = int(job["max_slots"])

            run_name = job["run_name"]
            t = threading.Thread(target=self._safe_run, args=(job,), daemon=True)
            active[run_name] = t
            t.start()
            # Loop straight back to fill any remaining free slots before sleeping.


def main() -> None:
    parser = argparse.ArgumentParser(description="Bucky distributed training worker")
    parser.add_argument("--server", default=os.getenv("BUCKY_SERVER", ""),
                        help="Server base URL, e.g. https://bucky.example.com")
    parser.add_argument("--token", default=os.getenv("BUCKY_DEVICE_TOKEN", ""),
                        help="Device token from the admin panel (Devices tab)")
    parser.add_argument("--slots", type=int, default=int(os.getenv("BUCKY_SLOTS", "0") or "0"),
                        help="Local hard ceiling on concurrent jobs (a safety cap for this "
                             "machine). Default 0 = no extra cap (use the core count); the "
                             "admin's per-device slider on the server is the primary, live "
                             "control and is honoured up to this ceiling.")
    parser.add_argument("--device", default=os.getenv("BUCKY_DEVICE", "cpu"),
                        choices=["cpu", "cuda", "auto"],
                        help="Torch device for this machine's trainer (default cpu, or "
                             "BUCKY_DEVICE). 'cuda'/'auto' opt this worker's GPU in — note "
                             "that for the tiny [64,64] MLP the GPU is often *slower* than "
                             "CPU because per-step host<->device copies dominate; benchmark "
                             "before relying on it.")
    args = parser.parse_args()
    if not args.server or not args.token:
        sys.exit("Set --server and --token (or BUCKY_SERVER / BUCKY_DEVICE_TOKEN).")

    try:
        Worker(args.server, args.token, slots=args.slots, device=args.device).serve_forever()
    except KeyboardInterrupt:
        print("\n[worker] stopped.", flush=True)


if __name__ == "__main__":
    main()
