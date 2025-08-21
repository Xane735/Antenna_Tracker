"""
logger_trk.py — lightweight, thread-safe logging for the antenna tracker

Features
- Non-blocking writes via a background thread (Queue → CSV).
- Two ready-to-use logs: tracker state CSV and raw-GPS CSV.
- Auto-creates timestamped filenames under a log directory.
- Graceful shutdown that drains the queue.
- Works from multiple threads (e.g., MAVLink reader + main loop).

Usage
------
from logger_trk import TrackerLogger

logger = TrackerLogger(log_dir="Tracker_Logs").start()
logger.log_state({...})       # once per loop
logger.log_raw_gps({...})     # in MAVLink reader thread
...
logger.stop()

"""

from __future__ import annotations
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, Optional, Any
import csv
import queue
import threading
import time
import datetime as dt
import os

__all__ = ["TrackerLogger", "AsyncCSVWriter", "LogFileConfig"]


@dataclass
class LogFileConfig:
    path: Path
    fieldnames: Iterable[str]
    flush_every: int = 25
    write_header: bool = True


class AsyncCSVWriter:
    """
    Threaded, non-blocking CSV writer.
    - Accepts dict rows; missing keys are written as empty.
    - Drains queue on stop().
    """
    def __init__(self, cfg: LogFileConfig):
        self.cfg = cfg
        self._fh = None
        self._csv = None
        self._q: "queue.Queue[Optional[Dict[str, Any]]]" = queue.Queue(maxsize=4096)
        self._th: Optional[threading.Thread] = None
        self._rows_since_flush = 0
        self._stopped = threading.Event()
        # Normalize fieldnames to list for deterministic order
        self._fieldnames = list(cfg.fieldnames)

    def start(self) -> "AsyncCSVWriter":
        self.cfg.path.parent.mkdir(parents=True, exist_ok=True)
        # newline='' for csv in py3
        self._fh = open(self.cfg.path, "w", newline="", encoding="utf-8")
        self._csv = csv.DictWriter(self._fh, fieldnames=self._fieldnames, extrasaction="ignore")
        if self.cfg.write_header:
            self._csv.writeheader()
            self._fh.flush()
        self._th = threading.Thread(target=self._run, name=f"CSVWriter:{self.cfg.path.name}", daemon=True)
        self._th.start()
        return self

    def put(self, row: Dict[str, Any]) -> None:
        if self._stopped.is_set():
            return
        # Ensure only known keys; fill missing with ""
        safe_row = {k: row.get(k, "") for k in self._fieldnames}
        self._q.put(safe_row, block=False)

    def _run(self):
        try:
            while not (self._stopped.is_set() and self._q.empty()):
                try:
                    row = self._q.get(timeout=0.2)
                except queue.Empty:
                    continue
                if row is None:
                    # sentinel
                    break
                self._csv.writerow(row)
                self._rows_since_flush += 1
                if self._rows_since_flush >= self.cfg.flush_every:
                    self._fh.flush()
                    self._rows_since_flush = 0
        finally:
            if self._fh:
                self._fh.flush()
                self._fh.close()

    def stop(self):
        self._stopped.set()
        # Drain remaining rows
        try:
            self._q.put(None, block=False)  # sentinel
        except queue.Full:
            pass
        if self._th:
            self._th.join(timeout=2.0)


def _ts_iso_z() -> str:
    # ISO8601 Zulu timestamp
    return dt.datetime.utcnow().replace(tzinfo=dt.timezone.utc).isoformat()


def _session_stamp(prefix: str) -> str:
    # YYYYMMDD-HHMMSS
    return f"{prefix}_{dt.datetime.now().strftime('%Y%m%d-%H%M%S')}"


DEFAULT_STATE_FIELDS = [
    # High-level status
    "t_iso", "t_s",
    "BaseMode", "BaseLocked",
    # World angles (math angles in degrees)
    "WorldAz_deg", "WorldEl_deg",
    # Calibrated / Physical / Servo angles (deg)
    "CalAz_deg", "CalEl_deg",
    "PhysAz_deg", "PhysEl_deg",
    "ServoAz_deg", "ServoEl_deg",
    # PWM microseconds
    "usAz", "usEl",
    # Drone/Base geo
    "DroneLat", "DroneLon", "DroneAlt_m",
    "BaseLat", "BaseLon", "BaseAlt_m",
    # Uncertainty (optional; meters)
    "BaseLatSD_m", "BaseLonSD_m",
    # Distances (computed if available)
    "HorizRange_m", "DeltaAlt_m",
]

DEFAULT_RAW_FIELDS = [
    "t_iso", "t_s", "src", "msg_type",
    "lat", "lon", "alt_m", "rel_alt_m",
    "fix_type", "satellites_visible",
    "hdop", "vdop",
    "vx", "vy", "vz", "ground_speed", "yaw_deg",
]


@dataclass
class TrackerLogger:
    log_dir: Path | str = "Tracker_Logs"
    session_prefix: str = "Tracker"
    raw_prefix: str = "Tracker_RAW"
    state_fields: Iterable[str] = field(default_factory=lambda: DEFAULT_STATE_FIELDS)
    raw_fields: Iterable[str] = field(default_factory=lambda: DEFAULT_RAW_FIELDS)
    flush_every: int = 25

    # Internal
    _dir: Path = field(init=False)
    _state_writer: Optional[AsyncCSVWriter] = field(init=False, default=None)
    _raw_writer: Optional[AsyncCSVWriter] = field(init=False, default=None)
    state_path: Optional[Path] = field(init=False, default=None)
    raw_path: Optional[Path] = field(init=False, default=None)
    _t0: float = field(init=False, default_factory=time.time)

    def start(self) -> "TrackerLogger":
        self._dir = Path(self.log_dir)
        stamp = _session_stamp(self.session_prefix)
        raw_stamp = _session_stamp(self.raw_prefix)
        self.state_path = self._dir / f"{stamp}.csv"
        self.raw_path = self._dir / f"{raw_stamp}.csv"
        # Initialize writers
        self._state_writer = AsyncCSVWriter(LogFileConfig(self.state_path, self.state_fields, self.flush_every)).start()
        self._raw_writer = AsyncCSVWriter(LogFileConfig(self.raw_path, self.raw_fields, self.flush_every)).start()
        # Emit a first line if you want; otherwise header-only
        return self

    def _now_row_base(self) -> Dict[str, Any]:
        return {"t_iso": _ts_iso_z(), "t_s": round(time.time() - self._t0, 3)}

    def log_state(self, row: Dict[str, Any]) -> None:
        """Log a single state row into the tracker CSV."""
        if not self._state_writer:
            return
        base = self._now_row_base()
        base.update(row)
        self._state_writer.put(base)

    def log_raw_gps(self, row: Dict[str, Any]) -> None:
        """Log a single raw GPS row (e.g., from GPS_RAW_INT or GLOBAL_POSITION_INT)."""
        if not self._raw_writer:
            return
        base = self._now_row_base()
        base.update(row)
        self._raw_writer.put(base)

    def stop(self):
        if self._state_writer:
            self._state_writer.stop()
            self._state_writer = None
        if self._raw_writer:
            self._raw_writer.stop()
            self._raw_writer = None
