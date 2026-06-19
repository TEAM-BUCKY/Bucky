"""Fan-out manager for the public read-only browser stream.

Ported from the old hub's broadcast helpers: keeps a set of connected browser
WebSockets and pushes each frame to all of them, dropping any slow/dead client.
"""
from __future__ import annotations

import asyncio
import json
import logging
from typing import Any

from fastapi import WebSocket

log = logging.getLogger(__name__)


class Broadcaster:
    def __init__(self) -> None:
        self._clients: set[WebSocket] = set()

    @property
    def count(self) -> int:
        return len(self._clients)

    async def register(self, ws: WebSocket) -> None:
        await ws.accept()
        self._clients.add(ws)

    def unregister(self, ws: WebSocket) -> None:
        self._clients.discard(ws)

    async def send(self, ws: WebSocket, message: Any) -> None:
        payload = message if isinstance(message, str) else json.dumps(message)
        try:
            await asyncio.wait_for(ws.send_text(payload), timeout=2.0)
        except Exception:  # noqa: BLE001 — slow/dead client; drop it
            self._clients.discard(ws)

    async def broadcast(self, message: Any) -> None:
        if not self._clients:
            return
        payload = message if isinstance(message, str) else json.dumps(message)
        await asyncio.gather(
            *(self.send(c, payload) for c in list(self._clients)),
            return_exceptions=True,
        )
