# app.py
import asyncio
from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket
from telemetry_payload import TelemetryState
from ws_hub import TelemetryHub
from geo_15 import get_latest_base, get_latest_drone  # adapt names if different

state = TelemetryState()
hub = TelemetryHub(state, get_latest_base, get_latest_drone, hz=10.0)  # 10 Hz

@asynccontextmanager
async def lifespan(app: FastAPI):
    # If you have acquisition threads already, they can keep running.
    # Optionally: start/stop any async tasks here.
    yield

app = FastAPI(lifespan=lifespan)

@app.websocket("/ws/telemetry")
async def ws_telemetry(ws: WebSocket):
    await hub.connect(ws)
    try:
        while True:
            # Keep the connection alive; we broadcast from the hub task
            # Optionally await ws.receive_text() to process client pings/commands
            await asyncio.sleep(60)
    except Exception:
        pass
    finally:
        await hub.disconnect(ws)

# Optional: REST fallback
@app.get("/api/telemetry")
def get_once():
    from telemetry_payload import build_tracker_payload
    return build_tracker_payload(state, get_latest_base(), get_latest_drone())
