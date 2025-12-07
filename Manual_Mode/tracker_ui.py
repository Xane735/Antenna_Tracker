# tracker_ui.py
from __future__ import annotations
from typing import Any, Dict, Optional
from datetime import datetime
import threading

from fastapi import FastAPI, Body
from fastapi.responses import HTMLResponse
import uvicorn

# ---------------- Shared state ----------------
_status: Dict[str, Any] = {}
_status_lock = threading.Lock()

def _deep_update(d: Dict[str, Any], u: Dict[str, Any]) -> Dict[str, Any]:
    for k, v in u.items():
        if isinstance(v, dict) and isinstance(d.get(k), dict):
            _deep_update(d[k], v)
        else:
            d[k] = v
    return d

# ---------------- FastAPI app -----------------
app = FastAPI()

@app.get("/status.json")
def status_json():
    with _status_lock:
        return _status or {"ok": False, "time": datetime.now().isoformat(timespec="seconds")}

@app.post("/update")
def update(payload: Dict[str, Any] = Body(...)):
    """
    Optional remote update endpoint. If you run this UI as a *separate process*,
    your tracker can POST here instead of importing `publish_status`.
    """
    with _status_lock:
        payload = dict(payload)
        payload["time"] = datetime.now().isoformat(timespec="seconds")
        _deep_update(_status, payload)
        return {"ok": True}

@app.get("/", response_class=HTMLResponse)
def index():
    # Simple, CSS-inlined dashboard
    return """
<!doctype html><html><head><meta charset="utf-8">
<title>Tracker Status</title><meta name="viewport" content="width=device-width,initial-scale=1">
<style>
 body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif;margin:16px;background:#0b0f14;color:#e6edf3}
 .grid{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(280px,1fr))}
 .card{background:#111827;border:1px solid #1f2937;border-radius:14px;padding:14px;box-shadow:0 1px 8px rgba(0,0,0,.3)}
 h1{font-size:20px;margin:0 0 8px}
 h2{font-size:16px;margin:0 0 6px;color:#9ca3af}
 .kv{display:grid;grid-template-columns:140px 1fr;gap:6px 10px;font-size:14px}
 .mono{font-family:ui-monospace, SFMono-Regular, Menlo, Consolas, monospace}
 small{color:#9ca3af}
 .row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
</style></head><body>
  <div class="row"><h1>📡 Antenna Tracker — Live Status</h1><small id="ts"></small></div>
  <div class="grid">
    <div class="card">
      <h2>Drone GPS</h2>
      <div class="kv mono">
        <div>Lat / Lon</div><div id="d_latlon">—</div>
        <div>Alt (m)</div><div id="d_alt">—</div>
        <div>Fix / Sats</div><div id="d_fix">—</div>
      </div>
    </div>
    <div class="card">
      <h2>Base GPS</h2>
      <div class="kv mono">
        <div>Mode / Locked</div><div id="b_mode">—</div>
        <div>Lat / Lon</div><div id="b_latlon">—</div>
        <div>Alt (m)</div><div id="b_alt">—</div>
        <div>Fix / Sats</div><div id="b_fix">—</div>
      </div>
    </div>
    <div class="card">
      <h2>Angles</h2>
      <div class="kv mono">
        <div>World (az/el)</div><div id="a_world">—</div>
        <div>Calibrated</div><div id="a_cal">—</div>
        <div>Physical</div><div id="a_phys">—</div>
        <div>Servo (deg)</div><div id="a_servo">—</div>
        <div>Pulses (µs)</div><div id="a_us">—</div>
        <div>Flip</div><div id="a_flip">—</div>
      </div>
    </div>
  </div>
<script>
function fmt(n,p=6){return(n===null||n===undefined||Number.isNaN(n))?'—':Number(n).toFixed(p)}
function s(x){return x===true?'Yes':x===false?'No':'—'}
async function tick(){
  try{
    const r = await fetch('/status.json',{cache:'no-store'});
    const j = await r.json();
    document.getElementById('ts').textContent = j.time || '';
    if(j.drone){
      document.getElementById('d_latlon').textContent = `${fmt(j.drone.lat,6)}, ${fmt(j.drone.lon,6)}`;
      document.getElementById('d_alt').textContent = fmt(j.drone.alt,2);
      document.getElementById('d_fix').textContent = `${j.drone.fix ?? '—'} / ${j.drone.sats ?? '—'}`;
    }
    if(j.base){
      document.getElementById('b_mode').textContent = `${j.base.mode ?? '—'} / locked=${s(j.base.locked)}`;
      document.getElementById('b_latlon').textContent = `${fmt(j.base.lat,6)}, ${fmt(j.base.lon,6)}`;
      document.getElementById('b_alt').textContent = fmt(j.base.alt,2);
      document.getElementById('b_fix').textContent = `${j.base.fix ?? '—'} / ${j.base.sats ?? '—'}`;
    }
    if(j.angles){
      document.getElementById('a_world').textContent = `${fmt(j.angles.world_az,2)}° / ${fmt(j.angles.world_el,2)}°`;
      document.getElementById('a_cal').textContent   = `${fmt(j.angles.cal_az,2)}° / ${fmt(j.angles.cal_el,2)}°`;
      document.getElementById('a_phys').textContent  = `${fmt(j.angles.phys_az,2)}° / ${fmt(j.angles.phys_el,2)}°`;
      document.getElementById('a_servo').textContent = `${fmt(j.angles.servo_az,2)}° / ${fmt(j.angles.servo_el,2)}°`;
      document.getElementById('a_us').textContent    = `${fmt(j.angles.us_az,0)} / ${fmt(j.angles.us_el,0)}`;
      document.getElementById('a_flip').textContent  = j.angles.flip ? 'Yes' : 'No';
    }
  }catch(e){}
}
tick(); setInterval(tick, 500);
</script>
</body></html>
    """

# ------------- Library-style helpers (import & call) -------------

def start_ui(host: str = "0.0.0.0", port: int = 8000):
    """Start FastAPI in a background thread (for use from geo_14.py)."""
    def _run():
        uvicorn.run(app, host=host, port=port, log_level="warning")
    th = threading.Thread(target=_run, daemon=True)
    th.start()
    print(f"[UI] Live at http://{host}:{port}  (/, /status.json)")

def publish_status(*,
    base_mode: str,
    base_locked: bool,
    base_lat: Optional[float], base_lon: Optional[float], base_alt: Optional[float],
    base_fix: Optional[int], base_sats: Optional[int],
    drone_lat: Optional[float], drone_lon: Optional[float], drone_alt: Optional[float],
    world_az: float, world_el: float,
    cal_az: float, cal_el: float,
    phys_az: float, phys_el: float,
    servo_az: float, servo_el: float,
    us_az: float, us_el: float,
    used_flip: bool
):
    """Direct in-process update (no HTTP). Call this each loop iteration."""
    payload = {
        "time": datetime.now().isoformat(timespec="seconds"),
        "base": {
            "mode": base_mode, "locked": bool(base_locked),
            "lat": base_lat, "lon": base_lon, "alt": base_alt,
            "fix": base_fix, "sats": base_sats
        },
        "drone": {
            "lat": drone_lat, "lon": drone_lon, "alt": drone_alt,
            "fix": None, "sats": None
        },
        "angles": {
            "world_az": world_az, "world_el": world_el,
            "cal_az": cal_az, "cal_el": cal_el,
            "phys_az": phys_az, "phys_el": phys_el,
            "servo_az": servo_az, "servo_el": servo_el,
            "us_az": us_az, "us_el": us_el,
            "flip": bool(used_flip),
        }
    }
    with _status_lock:
        _status.clear()
        _status.update(payload)

# ------------- If you want to run it standalone -------------
if __name__ == "__main__":
    # Run as: python3 tracker_ui.py -- then POST updates to /update or open / to view
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
