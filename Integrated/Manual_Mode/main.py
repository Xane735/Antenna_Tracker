"""
Antenna Tracker Web UI - FastAPI Server
Simple web interface for drone antenna tracker monitoring and control
"""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
import asyncio
import json
from typing import Optional
from datetime import datetime
import threading

app = FastAPI()

# Global state to store latest GPS data
latest_gps_data = {
    "base": {
        "lat": 0.0,
        "lon": 0.0,
        "alt": 0.0,
        "eph": None,
        "epv": None,
        "fix_type": None,
        "sats": None,
        "timestamp": None
    },
    "drone": {
        "lat": 0.0,
        "lon": 0.0,
        "alt": 0.0,
        "eph": None,
        "epv": None,
        "fix_type": None,
        "sats": None,
        "timestamp": None
    },
    "tracker": {
        "mode": "auto",
        "azimuth": 0.0,
        "elevation": 0.0,
        "phys_az": 0.0,
        "phys_el": 0.0
    }
}

active_connections = []


class ConnectionManager:
    def __init__(self):
        self.active_connections = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def broadcast(self, message: dict):
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except:
                pass


manager = ConnectionManager()


# ============ API Functions to integrate with your tracker code ============

def update_base_gps(lat: float, lon: float, alt: float, 
                    eph: Optional[float] = None, epv: Optional[float] = None,
                    fix_type: Optional[int] = None, sats: Optional[int] = None):
    """Call this function from your tracker code to update base GPS data"""
    latest_gps_data["base"].update({
        "lat": lat,
        "lon": lon,
        "alt": alt,
        "eph": eph,
        "epv": epv,
        "fix_type": fix_type,
        "sats": sats,
        "timestamp": datetime.now().isoformat()
    })


def update_drone_gps(lat: float, lon: float, alt: float,
                     eph: Optional[float] = None, epv: Optional[float] = None,
                     fix_type: Optional[int] = None, sats: Optional[int] = None):
    """Call this function from your tracker code to update drone GPS data"""
    latest_gps_data["drone"].update({
        "lat": lat,
        "lon": lon,
        "alt": alt,
        "eph": eph,
        "epv": epv,
        "fix_type": fix_type,
        "sats": sats,
        "timestamp": datetime.now().isoformat()
    })


def update_tracker_state(mode: str, azimuth: float, elevation: float,
                         phys_az: float, phys_el: float):
    """Call this function from your tracker code to update tracker state"""
    latest_gps_data["tracker"].update({
        "mode": mode,
        "azimuth": azimuth,
        "elevation": elevation,
        "phys_az": phys_az,
        "phys_el": phys_el
    })


# ============ WebSocket for real-time updates ============

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            # Send updates every 100ms
            await manager.broadcast(latest_gps_data)
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        manager.disconnect(websocket)


# ============ REST API Endpoints ============

@app.get("/api/gps")
async def get_gps_data():
    """Get current GPS data"""
    return latest_gps_data


@app.post("/api/manual_control")
async def manual_control(command: dict):
    """
    Handle manual control commands from joystick
    Expected format: {"action": "move", "azimuth_delta": 5.0, "elevation_delta": 2.0}
    or {"action": "mode_toggle"}
    
    Integrate this with your ManualController or tracker code
    """
    # TODO: Call your tracker's manual control functions here
    print(f"Manual control command: {command}")
    return {"status": "ok", "command": command}


@app.post("/api/calibrate")
async def calibrate():
    """Trigger calibration mode"""
    # TODO: Call your calibrate_simple() function here
    return {"status": "calibration_started"}


# ============ HTML Interface ============

@app.get("/")
async def get_ui():
    with open("ui.html", "r", encoding="utf-8") as f:
        html_content = f.read()
    return HTMLResponse(content=html_content, status_code=200)


# ============ Integration Guide ============
"""
INTEGRATION WITH YOUR TRACKER CODE (geo_14.py):

1. Import this module at the top of geo_14.py:
   from tracker_ui_server import update_base_gps, update_drone_gps, update_tracker_state

2. In your GPS reader callbacks (set_latest_base/drone), add:
   
   def set_latest_drone(sample: GpsSample):
       global _latest_drone
       with _drone_lock:
           _latest_drone = sample
       # ADD THIS:
       update_drone_gps(sample.lat, sample.lon, sample.alt, 
                       sample.eph, sample.epv, sample.fix_type, sample.sats)

3. In your main loop, after updating servo positions, add:
   
   update_tracker_state(get_mode(), world_az, world_el, curr_phys_az, curr_phys_el)

4. Run the FastAPI server in a separate thread:
   
   import uvicorn
   from threading import Thread
   
   def run_ui_server():
       uvicorn.run(app, host="0.0.0.0", port=8000)
   
   ui_thread = Thread(target=run_ui_server, daemon=True)
   ui_thread.start()

5. Access the UI at: http://localhost:8000
"""


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)