"""
tracker_web_ui.py - Web UI for antenna tracker GPS monitoring
Displays real-time GPS data for base and drone with map integration
"""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
import asyncio
import json
from typing import Optional, Dict, Any
from dataclasses import dataclass, asdict
import time
import threading

# ============= Configuration =============
WEB_PORT = 8000
UPDATE_RATE_HZ = 10  # How often to push updates to clients

# ============= Data Models =============
@dataclass
class GPSData:
    """GPS data structure matching geo_14.py GpsSample"""
    timestamp: float
    lat: float
    lon: float
    alt: float
    eph: Optional[float] = None  # Horizontal accuracy (m)
    epv: Optional[float] = None  # Vertical accuracy (m)
    fix_type: Optional[int] = None
    sats: Optional[int] = None
    heading: Optional[float] = None  # degrees
    speed: Optional[float] = None  # m/s

@dataclass
class TrackerState:
    """Complete tracker state"""
    base: Optional[GPSData] = None
    drone: Optional[GPSData] = None
    world_az: Optional[float] = None
    world_el: Optional[float] = None
    servo_az: Optional[float] = None
    servo_el: Optional[float] = None
    distance: Optional[float] = None  # meters
    is_tracking: bool = False
    base_mode: str = "unknown"
    used_flip: bool = False

# ============= Global State =============
class StateManager:
    """Thread-safe state management"""
    def __init__(self):
        self._state = TrackerState()
        self._lock = threading.Lock()
    
    def update_base(self, gps_data: GPSData):
        with self._lock:
            self._state.base = gps_data
    
    def update_drone(self, gps_data: GPSData):
        with self._lock:
            self._state.drone = gps_data
    
    def update_tracker_info(self, **kwargs):
        with self._lock:
            for key, value in kwargs.items():
                if hasattr(self._state, key):
                    setattr(self._state, key, value)
    
    def get_state(self) -> Dict[str, Any]:
        with self._lock:
            return {
                'base': asdict(self._state.base) if self._state.base else None,
                'drone': asdict(self._state.drone) if self._state.drone else None,
                'world_az': self._state.world_az,
                'world_el': self._state.world_el,
                'servo_az': self._state.servo_az,
                'servo_el': self._state.servo_el,
                'distance': self._state.distance,
                'is_tracking': self._state.is_tracking,
                'base_mode': self._state.base_mode,
                'used_flip': self._state.used_flip,
                'timestamp': time.time()
            }

state_manager = StateManager()

# ============= FastAPI App =============
app = FastAPI(title="Antenna Tracker Monitor")

# WebSocket connection manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

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

# ============= Routes =============
@app.get("/", response_class=HTMLResponse)
async def get_index():
    """Serve the main HTML page"""
    return get_html_content()

@app.get("/api/state")
async def get_state():
    """REST endpoint for current state"""
    return state_manager.get_state()

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket for real-time updates"""
    await manager.connect(websocket)
    try:
        # Send initial state
        await websocket.send_json(state_manager.get_state())
        
        # Keep connection alive and listen for client messages
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)

# ============= Background Tasks =============
async def broadcast_updates():
    """Periodically broadcast state to all connected clients"""
    while True:
        await asyncio.sleep(1.0 / UPDATE_RATE_HZ)
        if manager.active_connections:
            await manager.broadcast(state_manager.get_state())

@app.on_event("startup")
async def startup_event():
    """Start background tasks"""
    asyncio.create_task(broadcast_updates())

# ============= Public API for geo_14.py Integration =============
def update_base_gps(lat: float, lon: float, alt: float, 
                    eph: Optional[float] = None, 
                    epv: Optional[float] = None,
                    fix_type: Optional[int] = None,
                    sats: Optional[int] = None):
    """Call this from geo_14.py to update base GPS"""
    gps_data = GPSData(
        timestamp=time.time(),
        lat=lat, lon=lon, alt=alt,
        eph=eph, epv=epv, 
        fix_type=fix_type, sats=sats
    )
    state_manager.update_base(gps_data)

def update_drone_gps(lat: float, lon: float, alt: float,
                     heading: Optional[float] = None,
                     speed: Optional[float] = None,
                     eph: Optional[float] = None,
                     epv: Optional[float] = None,
                     fix_type: Optional[int] = None,
                     sats: Optional[int] = None):
    """Call this from geo_14.py to update drone GPS"""
    gps_data = GPSData(
        timestamp=time.time(),
        lat=lat, lon=lon, alt=alt,
        heading=heading, speed=speed,
        eph=eph, epv=epv,
        fix_type=fix_type, sats=sats
    )
    state_manager.update_drone(gps_data)

def update_tracker_state(world_az: Optional[float] = None,
                         world_el: Optional[float] = None,
                         servo_az: Optional[float] = None,
                         servo_el: Optional[float] = None,
                         distance: Optional[float] = None,
                         is_tracking: bool = False,
                         base_mode: str = "unknown",
                         used_flip: bool = False):
    """Call this from geo_14.py to update tracker state"""
    state_manager.update_tracker_info(
        world_az=world_az,
        world_el=world_el,
        servo_az=servo_az,
        servo_el=servo_el,
        distance=distance,
        is_tracking=is_tracking,
        base_mode=base_mode,
        used_flip=used_flip
    )

# ============= HTML Content =============
def get_html_content():
    return """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Antenna Tracker Monitor</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: #0f172a;
            color: #e2e8f0;
            padding: 20px;
        }
        
        .container {
            max-width: 1600px;
            margin: 0 auto;
        }
        
        header {
            text-align: center;
            margin-bottom: 30px;
            padding: 20px;
            background: linear-gradient(135deg, #1e40af 0%, #7c3aed 100%);
            border-radius: 12px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.3);
        }
        
        h1 {
            font-size: 2rem;
            margin-bottom: 8px;
        }
        
        .status-badge {
            display: inline-block;
            padding: 6px 16px;
            border-radius: 20px;
            font-size: 0.9rem;
            font-weight: 600;
            margin-top: 10px;
        }
        
        .status-tracking { background: #10b981; color: white; }
        .status-idle { background: #ef4444; color: white; }
        
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(350px, 1fr));
            gap: 20px;
            margin-bottom: 20px;
        }
        
        .card {
            background: #1e293b;
            border-radius: 12px;
            padding: 20px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.3);
            border: 1px solid #334155;
        }
        
        .card h2 {
            font-size: 1.3rem;
            margin-bottom: 16px;
            color: #60a5fa;
            display: flex;
            align-items: center;
            gap: 10px;
        }
        
        .icon {
            font-size: 1.5rem;
        }
        
        .data-row {
            display: flex;
            justify-content: space-between;
            padding: 10px 0;
            border-bottom: 1px solid #334155;
        }
        
        .data-row:last-child {
            border-bottom: none;
        }
        
        .label {
            color: #94a3b8;
            font-weight: 500;
        }
        
        .value {
            color: #e2e8f0;
            font-weight: 600;
            font-family: 'Courier New', monospace;
        }
        
        .map-container {
            background: #1e293b;
            border-radius: 12px;
            padding: 20px;
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.3);
            border: 1px solid #334155;
            min-height: 500px;
        }
        
        #map {
            width: 100%;
            height: 450px;
            background: #0f172a;
            border-radius: 8px;
            display: flex;
            align-items: center;
            justify-content: center;
            color: #64748b;
            font-size: 1.1rem;
        }
        
        .no-data {
            color: #64748b;
            font-style: italic;
        }
        
        .good { color: #10b981; }
        .warning { color: #f59e0b; }
        .bad { color: #ef4444; }
        
        .connection-status {
            position: fixed;
            top: 20px;
            right: 20px;
            padding: 10px 20px;
            border-radius: 8px;
            font-weight: 600;
            z-index: 1000;
        }
        
        .connected {
            background: #10b981;
            color: white;
        }
        
        .disconnected {
            background: #ef4444;
            color: white;
        }
    </style>
</head>
<body>
    <div class="connection-status" id="connectionStatus">Connecting...</div>
    
    <div class="container">
        <header>
            <h1>🛰️ Antenna Tracker Monitor</h1>
            <div class="status-badge" id="trackingStatus">IDLE</div>
        </header>
        
        <div class="grid">
            <!-- Base GPS Card -->
            <div class="card">
                <h2><span class="icon">📍</span>Base Station GPS</h2>
                <div class="data-row">
                    <span class="label">Latitude:</span>
                    <span class="value" id="baseLat">--</span>
                </div>
                <div class="data-row">
                    <span class="label">Longitude:</span>
                    <span class="value" id="baseLon">--</span>
                </div>
                <div class="data-row">
                    <span class="label">Altitude:</span>
                    <span class="value" id="baseAlt">--</span>
                </div>
                <div class="data-row">
                    <span class="label">Accuracy (H/V):</span>
                    <span class="value" id="baseAccuracy">--</span>
                </div>
                <div class="data-row">
                    <span class="label">Fix Type:</span>
                    <span class="value" id="baseFix">--</span>
                </div>
                <div class="data-row">
                    <span class="label">Satellites:</span>
                    <span class="value" id="baseSats">--</span>
                </div>
                <div class="data-row">
                    <span class="label">Mode:</span>
                    <span class="value" id="baseMode">--</span>
                </div>
            </div>
            
            <!-- Drone GPS Card -->
            <div class="card">
                <h2><span class="icon">✈️</span>Drone GPS</h2>
                <div class="data-row">
                    <span class="label">Latitude:</span>
                    <span class="value" id="droneLat">--</span>
                </div>
                <div class="data-row">
                    <span class="label">Longitude:</span>
                    <span class="value" id="droneLon">--</span>
                </div>
                <div class="data-row">
                    <span class="label">Altitude:</span>
                    <span class="value" id="droneAlt">--</span>
                </div>
                <div class="data-row">
                    <span class="label">Accuracy (H/V):</span>
                    <span class="value" id="droneAccuracy">--</span>
                </div>
                <div class="data-row">
                    <span class="label">Heading:</span>
                    <span class="value" id="droneHeading">--</span>
                </div>
                <div class="data-row">
                    <span class="label">Speed:</span>
                    <span class="value" id="droneSpeed">--</span>
                </div>
                <div class="data-row">
                    <span class="label">Satellites:</span>
                    <span class="value" id="droneSats">--</span>
                </div>
            </div>
            
            <!-- Tracker Info Card -->
            <div class="card">
                <h2><span class="icon">🎯</span>Tracker State</h2>
                <div class="data-row">
                    <span class="label">Distance:</span>
                    <span class="value" id="distance">--</span>
                </div>
                <div class="data-row">
                    <span class="label">World Az/El:</span>
                    <span class="value" id="worldAngles">--</span>
                </div>
                <div class="data-row">
                    <span class="label">Servo Az/El:</span>
                    <span class="value" id="servoAngles">--</span>
                </div>
                <div class="data-row">
                    <span class="label">Flip Status:</span>
                    <span class="value" id="flipStatus">--</span>
                </div>
                <div class="data-row">
                    <span class="label">Last Update:</span>
                    <span class="value" id="lastUpdate">--</span>
                </div>
            </div>
        </div>
        
        <!-- Map Container -->
        <div class="map-container">
            <h2 style="margin-bottom: 16px; color: #60a5fa;">
                <span class="icon">🗺️</span> Map View
            </h2>
            <div id="map">
                Map container ready for integration<br>
                (Add Leaflet.js, Google Maps, or Mapbox here)
            </div>
        </div>
    </div>
    
    <script src="/static/tracker-client.js"></script>
</body>
</html>
    """

# ============= Run Server =============
def start_server(port: int = WEB_PORT):
    """Start the FastAPI server"""
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=port)

if __name__ == "__main__":
    print(f"Starting Antenna Tracker Web UI on port {WEB_PORT}")
    print(f"Open http://localhost:{WEB_PORT} in your browser")
    start_server()