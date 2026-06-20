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
import subprocess
import sys
import threading
import time
from pathlib import Path

import requests

BASE_DIR = Path(__file__).resolve().parent.parent  # backend/
POLL_INTERVAL = 5.0       # seconds between lease attempts when the queue is empty
HEARTBEAT_INTERVAL = 15.0  # seconds between lease-renewing heartbeats while training


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
    def __init__(self, server: str, token: str) -> None:
        self.api = _api_base(server)
        self.token = token
        self.headers = {"X-Device-Token": token}

    # ── server calls ──────────────────────────────────────────────────────────
    def lease(self) -> dict | None:
        r = requests.post(f"{self.api}/worker/lease", headers=self.headers, timeout=30)
        if r.status_code == 204:
            return None
        r.raise_for_status()
        return r.json()

    def heartbeat(self, run_name: str) -> None:
        try:
            requests.post(f"{self.api}/worker/heartbeat", headers=self.headers,
                          json={"run_name": run_name}, timeout=15)
        except requests.RequestException:
            pass  # transient; the lease has slack before it expires

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
            *stop_args,
        ]
        proc = subprocess.Popen(args, cwd=str(BASE_DIR), start_new_session=True)

        # Heartbeat in the background so the server keeps our lease while we train.
        stop_hb = threading.Event()

        def _beat() -> None:
            while not stop_hb.wait(HEARTBEAT_INTERVAL):
                self.heartbeat(run_name)

        hb = threading.Thread(target=_beat, daemon=True)
        hb.start()
        try:
            code = proc.wait()
        finally:
            stop_hb.set()

        status = "done" if code == 0 else "failed"
        steps, best = self._read_local_meta(run_dir)
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

    # ── main loop ─────────────────────────────────────────────────────────────
    def serve_forever(self) -> None:
        print(f"[worker] polling {self.api} for jobs…", flush=True)
        while True:
            try:
                job = self.lease()
            except requests.RequestException as exc:
                print(f"[worker] lease error: {exc}; retrying", flush=True)
                time.sleep(POLL_INTERVAL)
                continue
            if job is None:
                time.sleep(POLL_INTERVAL)
                continue
            try:
                self.run_job(job)
            except Exception as exc:  # noqa: BLE001 — never let one bad job kill the worker
                print(f"[worker] job error: {exc}", flush=True)
                time.sleep(POLL_INTERVAL)


def main() -> None:
    parser = argparse.ArgumentParser(description="Bucky distributed training worker")
    parser.add_argument("--server", default=os.getenv("BUCKY_SERVER", ""),
                        help="Server base URL, e.g. https://bucky.example.com")
    parser.add_argument("--token", default=os.getenv("BUCKY_DEVICE_TOKEN", ""),
                        help="Device token from the admin panel (Devices tab)")
    args = parser.parse_args()
    if not args.server or not args.token:
        sys.exit("Set --server and --token (or BUCKY_SERVER / BUCKY_DEVICE_TOKEN).")

    try:
        Worker(args.server, args.token).serve_forever()
    except KeyboardInterrupt:
        print("\n[worker] stopped.", flush=True)


if __name__ == "__main__":
    main()
