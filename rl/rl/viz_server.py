"""WebSocket broadcast server for real-time training visualization.

Usage:
    server = VizServer(port=8765)
    server.start()                   # starts asyncio server in background thread
    server.send_state(state, terms)  # call from env step (thread-safe)
    server.stop()

The SvelteKit frontend connects to ws://localhost:8765 and receives JSON frames.
Frame schema:
    {
      "type": "step",
      "robot_pos": [x, y],
      "robot_heading": float,
      "ball_pos": [x, y],
      "reward_terms": { term: value, ... },
      "episode": int,
      "step": int,
      "total_return": float
    }
"""
from __future__ import annotations
import asyncio
import json
import threading
import queue
import logging

log = logging.getLogger(__name__)


class VizServer:
    """Thread-safe WebSocket broadcast server."""

    def __init__(self, port: int = 8765) -> None:
        self._port = port
        self._queue: queue.Queue[dict] = queue.Queue(maxsize=64)
        self._clients: set = set()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._episode = 0
        self._step = 0
        self._total_return = 0.0

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._loop:
            self._loop.call_soon_threadsafe(self._loop.stop)

    def on_episode_reset(self) -> None:
        self._episode += 1
        self._step = 0
        self._total_return = 0.0

    def send_state(self, state, reward_terms) -> None:
        """Non-blocking; drops frame if queue full."""
        self._step += 1
        self._total_return += reward_terms.total if hasattr(reward_terms, "total") else 0.0
        frame = {
            "type": "step",
            "robot_pos": state.robot_pos.tolist(),
            "robot_heading": float(state.robot_heading),
            "ball_pos": state.ball_pos.tolist(),
            "reward_terms": reward_terms.as_dict() if hasattr(reward_terms, "as_dict") else {},
            "episode": self._episode,
            "step": self._step,
            "total_return": self._total_return,
        }
        try:
            self._queue.put_nowait(frame)
        except queue.Full:
            pass

    def _run_loop(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._serve())

    async def _serve(self) -> None:
        try:
            import websockets
        except ImportError:
            log.error("websockets not installed — pip install websockets")
            return

        async def handler(ws):
            self._clients.add(ws)
            try:
                await ws.wait_closed()
            finally:
                self._clients.discard(ws)

        async def broadcaster():
            while True:
                try:
                    frame = self._queue.get_nowait()
                    if self._clients:
                        msg = json.dumps(frame)
                        await asyncio.gather(
                            *(c.send(msg) for c in list(self._clients)),
                            return_exceptions=True,
                        )
                except queue.Empty:
                    await asyncio.sleep(0.02)

        async with websockets.serve(handler, "localhost", self._port):
            log.info(f"Viz server listening on ws://localhost:{self._port}")
            await broadcaster()
