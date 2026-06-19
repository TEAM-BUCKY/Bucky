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

    def __init__(self, url: str, max_queue: int = 128) -> None:
        self._url = url
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

        backoff = 0.5
        while not self._stop.is_set():
            try:
                async with websockets.connect(
                    self._url, ping_interval=20, ping_timeout=20
                ) as ws:
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
