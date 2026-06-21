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
    """Receive-only WS client: subscribes to per-side manual-control messages from the hub.

    The mirror image of :class:`StreamClient` — a background daemon thread owns an asyncio
    loop that connects (auto-reconnect with backoff) to the hub's ``/api/control_sink``
    endpoint and consumes ``{"side": "a"|"b", "action": [...], "mode": "human"|"ai"}`` messages
    (the legacy ``{"action", "red_mode"}`` shape is accepted and treated as side "b"). Two
    independent control streams — one per robot — ride this one socket, demultiplexed by ``side``,
    so two browsers (one per side, online human-vs-human) feed the same match.

    The match loop calls :meth:`latest` each tick (thread-safe, never blocks) to fetch both
    sides' current commands as ``{"a": {"action", "mode"}, "b": {...}}``. Two staleness
    safeguards per side: after ``stale_after`` seconds with no message the action decays to
    zeros (a dropped browser can't leave a robot driving on its own); after
    ``ai_fallback_after`` seconds the side reverts to ``"ai"`` so the policy takes back over
    when a player disconnects and the match keeps going.
    """

    _SIDES = ("a", "b")

    def __init__(self, url: str, headers: dict | None = None, stale_after: float = 0.5,
                 ai_fallback_after: float = 5.0, default_modes: dict | None = None) -> None:
        self._url = url
        self._headers = headers or None
        self._stale_after = stale_after
        self._ai_fallback_after = ai_fallback_after
        modes = default_modes or {}
        self._lock = threading.Lock()
        # Per-side command state. Default mode "ai" → an unclaimed side is played by its policy
        # until a browser starts sending ``mode == "human"`` for it.
        self._sides: dict[str, dict] = {
            s: {"action": [0.0, 0.0, 0.0, 0.0], "mode": modes.get(s, "ai"), "last_recv": 0.0}
            for s in self._SIDES
        }
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
        """Both sides' current commands: ``{"a": {"action", "mode"}, "b": {"action", "mode"}}``."""
        now = time.monotonic()
        with self._lock:
            snapshot = {s: (list(v["action"]), v["mode"], v["last_recv"])
                        for s, v in self._sides.items()}
        out: dict[str, dict] = {}
        for s, (action, mode, last) in snapshot.items():
            if last > 0.0:
                age = now - last
                if age > self._stale_after:
                    action = [0.0, 0.0, 0.0, 0.0]   # stale → stop driving
                if age > self._ai_fallback_after:
                    mode = "ai"                      # player gone → hand back to the policy
            out[s] = {"action": action, "mode": mode}
        return out

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
            side = msg.get("side", "b")          # legacy messages carry no side → red (B)
            if side not in self._SIDES:
                continue
            mode = msg.get("mode", msg.get("red_mode"))   # accept legacy ``red_mode``
            with self._lock:
                slot = self._sides[side]
                action = msg.get("action")
                if isinstance(action, (list, tuple)) and len(action) >= 4:
                    slot["action"] = [float(x) for x in action[:4]]
                if mode in ("human", "ai"):
                    slot["mode"] = mode
                slot["last_recv"] = time.monotonic()
