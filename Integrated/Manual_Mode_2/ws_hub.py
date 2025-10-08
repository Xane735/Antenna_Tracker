# ws_hub.py
import asyncio
from typing import Set
from fastapi import WebSocket, WebSocketDisconnect
from telemetry_payload import build_tracker_payload

class TelemetryHub:
    def __init__(self, state, base_fn, drone_fn, hz: float = 10.0):
        self.state = state
        self.base_fn = base_fn
        self.drone_fn = drone_fn
        self.period = 1.0 / hz
        self.clients: Set[WebSocket] = set()
        self._task: asyncio.Task | None = None
        self._lock = asyncio.Lock()

    async def connect(self, ws: WebSocket):
        await ws.accept()
        async with self._lock:
            self.clients.add(ws)
            if not self._task:
                self._task = asyncio.create_task(self._broadcast_loop())

    async def disconnect(self, ws: WebSocket):
        async with self._lock:
            self.clients.discard(ws)
            if not self.clients and self._task:
                self._task.cancel()
                self._task = None

    async def _broadcast_loop(self):
        try:
            while True:
                payload = build_tracker_payload(self.state, self.base_fn(), self.drone_fn())
                # send concurrently so slow clients don't block
                send_tasks = [c.send_json(payload) for c in list(self.clients)]
                if send_tasks:
                    await asyncio.gather(*send_tasks, return_exceptions=True)
                await asyncio.sleep(self.period)
        except asyncio.CancelledError:
            pass
