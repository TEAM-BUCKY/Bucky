"""Trainer-side WebSocket client that streams JSON messages to the viz hub.

The training loop calls ``send()`` from the main (learn) thread; it never blocks.
A background daemon thread owns an asyncio loop that maintains the connection to
``ws://localhost:<port>/ingest`` (auto-reconnect with backoff) and drains a bounded
queue, dropping messages while disconnected or when the queue is full. This means a
hub restart can never stall or kill the training run.

Mirror of the old ``VizServer`` thread/queue pattern, but as a *client* — the hub
(``app/jobs.py``) is now the long-lived server the browser connects to.
"""
from __future__ import annotations
import asyncio
import json
import logging
import queue
import threading
import time

import numpy as np

log = logging.getLogger(__name__)


def _json_default(o):
    """Coerce numpy scalars/arrays so reward terms etc. serialize cleanly."""
    if isinstance(o, np.generic):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(f"Object of type {type(o).__name__} is not JSON serializable")


class StreamClient:
    """Thread-safe, non-blocking WS client for streaming viz frames to the hub."""

    def __init__(self, url: str, max_queue: int = 128, headers: dict | None = None) -> None:
        self._url = url
        # Optional handshake headers (e.g. an auth token), kept out of the URL so the
        # secret never reaches proxy/access logs via the query string.
        self._headers = headers or None
        self._queue: queue.Queue[str] = queue.Queue(maxsize=max_queue)
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._loop is not None:
            self._loop.call_soon_threadsafe(lambda: None)  # wake the loop

    def send(self, msg: dict) -> None:
        """Non-blocking; drops the message if the queue is full."""
        try:
            self._queue.put_nowait(json.dumps(msg, default=_json_default))
        except queue.Full:
            pass

    # ── background thread ────────────────────────────────────────────────────
    def _run(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._main())
        finally:
            self._loop.close()

    async def _main(self) -> None:
        try:
            import websockets
        except ImportError:
            log.error("websockets not installed — pip install websockets")
            return

        def _open():
            """Open the connection, passing headers under whichever keyword the
            installed ``websockets`` supports (additional_headers ≥ v14, else
            extra_headers)."""
            base = dict(ping_interval=20, ping_timeout=20)
            if self._headers:
                for kw in ("additional_headers", "extra_headers"):
                    try:
                        return websockets.connect(self._url, **{kw: self._headers}, **base)
                    except TypeError:
                        continue
            return websockets.connect(self._url, **base)

        backoff = 0.5
        while not self._stop.is_set():
            try:
                async with _open() as ws:
                    log.info("StreamClient connected to %s", self._url)
                    backoff = 0.5
                    await self._pump(ws)
            except Exception as exc:  # noqa: BLE001 — connect failed or dropped
                if self._stop.is_set():
                    break
                log.debug("StreamClient disconnected (%s); retrying in %.1fs", exc, backoff)
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, 5.0)

    async def _pump(self, ws) -> None:
        """Drain the thread-safe queue and forward each message over the socket."""
        while not self._stop.is_set():
            try:
                msg = self._queue.get_nowait()
            except queue.Empty:
                await asyncio.sleep(0.005)
                continue
            await ws.send(msg)


class ControlClient:
    """Receive-only WS client: subscribes to manual-control messages from the hub.

    The mirror image of :class:`StreamClient` — a background daemon thread owns an asyncio
    loop that connects (auto-reconnect with backoff) to the hub's ``/api/control_sink``
    endpoint and consumes ``{"action": [...], "red_mode": "human"|"ai"}`` messages. The match
    loop calls :meth:`latest` each tick (thread-safe, never blocks) to fetch the human's
    current command for robot A. If no message has arrived within ``stale_after`` seconds the
    action decays to zeros so a dropped browser can't leave the robot driving on its own.
    """

    def __init__(self, url: str, headers: dict | None = None, stale_after: float = 0.5) -> None:
        self._url = url
        self._headers = headers or None
        self._stale_after = stale_after
        self._lock = threading.Lock()
        self._action: list[float] = [0.0, 0.0, 0.0, 0.0]
        self._red_mode = "human"   # manual match starts under human control by default
        self._last_recv = 0.0
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._loop is not None:
            self._loop.call_soon_threadsafe(lambda: None)

    def latest(self) -> dict:
        """The current control command for robot A (``{"action", "red_mode"}``)."""
        with self._lock:
            action = list(self._action)
            red_mode = self._red_mode
            last = self._last_recv
        if last > 0.0 and (time.monotonic() - last) > self._stale_after:
            action = [0.0, 0.0, 0.0, 0.0]   # stale → stop driving (keep the last mode)
        return {"action": action, "red_mode": red_mode}

    # ── background thread ────────────────────────────────────────────────────
    def _run(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._main())
        finally:
            self._loop.close()

    async def _main(self) -> None:
        try:
            import websockets
        except ImportError:
            log.error("websockets not installed — pip install websockets")
            return

        def _open():
            base = dict(ping_interval=20, ping_timeout=20)
            if self._headers:
                for kw in ("additional_headers", "extra_headers"):
                    try:
                        return websockets.connect(self._url, **{kw: self._headers}, **base)
                    except TypeError:
                        continue
            return websockets.connect(self._url, **base)

        backoff = 0.5
        while not self._stop.is_set():
            try:
                async with _open() as ws:
                    log.info("ControlClient connected to %s", self._url)
                    backoff = 0.5
                    await self._consume(ws)
            except Exception as exc:  # noqa: BLE001 — connect failed or dropped
                if self._stop.is_set():
                    break
                log.debug("ControlClient disconnected (%s); retrying in %.1fs", exc, backoff)
                await asyncio.sleep(backoff)
                backoff = min(backoff * 2, 5.0)

    async def _consume(self, ws) -> None:
        async for raw in ws:
            if self._stop.is_set():
                break
            try:
                msg = json.loads(raw)
            except (json.JSONDecodeError, TypeError):
                continue
            with self._lock:
                action = msg.get("action")
                if isinstance(action, (list, tuple)) and len(action) >= 4:
                    self._action = [float(x) for x in action[:4]]
                red_mode = msg.get("red_mode")
                if red_mode in ("human", "ai"):
                    self._red_mode = red_mode
                self._last_recv = time.monotonic()
