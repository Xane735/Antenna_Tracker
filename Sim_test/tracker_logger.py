import csv
import pathlib
from datetime import datetime
from typing import Optional, Callable
from dataclasses import dataclass

# Global logging configuration
LOG_TO_CSV = True
LOG_RAW_GPS = True

# Global logging objects
_log_writer = None
_log_file = None
_raw_writer = None
_raw_file = None

@dataclass
class GpsSample:
    """GPS sample data structure for logging"""
    t: float
    lat: float
    lon: float
    alt: float
    eph: Optional[float] = None     # horizontal accuracy (m) if available
    epv: Optional[float] = None     # vertical accuracy (m) if available
    fix_type: Optional[int] = None  # 0..6 (3=3D fix)
    sats: Optional[int] = None

def log_open(prefix="Tracker"):
    """
    Initialize CSV logging files for both main tracking data and raw GPS data.
    
    Args:
        prefix (str): Prefix for the log filenames
    """
    global _log_writer, _log_file, _raw_writer, _raw_file
    
    if not LOG_TO_CSV and not LOG_RAW_GPS:
        return
    
    # Create logs directory
    pathlib.Path("Tracker_Logs").mkdir(exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d-%H%M%S")
    
    # Initialize main tracking CSV log
    if LOG_TO_CSV:
        fn = pathlib.Path(f"Tracker_Logs/{prefix}_{ts}.csv")
        _log_file = fn.open("w", newline="")
        _log_writer = csv.writer(_log_file)
        _log_writer.writerow([
            "Time",
            "BaseMode", "BaseLocked",
            "WorldAz", "WorldEl", "CalAz", "CalEl",
            "PhysAz", "PhysEl", "ServoAz", "ServoEl", "Az(us)", "El(us)",
            "DroneLat", "DroneLon", "DroneAlt",
            "BaseLat", "BaseLon", "BaseAlt",
            "BaseLatSDm", "BaseLonSDm", "BaseFix", "BaseSats"
        ])
        print(f"[INFO] CSV log → {fn}")
    
    # Initialize raw GPS CSV log
    if LOG_RAW_GPS:
        fnr = pathlib.Path(f"Tracker_Logs/{prefix}_RAW_{ts}.csv")
        _raw_file = fnr.open("w", newline="")
        _raw_writer = csv.writer(_raw_file)
        _raw_writer.writerow(["Time", "Stream", "Lat", "Lon", "Alt", "eph(m)", "epv(m)", "fix_type", "sats"])
        print(f"[INFO] RAW GPS log → {fnr}")

def log_row(*row):
    """
    Write a row to the main tracking CSV log.
    
    Args:
        *row: Variable number of arguments representing the data row
    """
    if _log_writer:
        _log_writer.writerow(row)
        _log_file.flush()

def log_raw(stream: str, s: GpsSample):
    """
    Write a raw GPS sample to the raw GPS CSV log.
    
    Args:
        stream (str): Name of the GPS stream (e.g., "drone", "base")
        s (GpsSample): GPS sample data
    """
    if _raw_writer:
        _raw_writer.writerow([
            datetime.now().isoformat(timespec='seconds'),
            stream, 
            f"{s.lat:.7f}", 
            f"{s.lon:.7f}", 
            f"{s.alt:.2f}",
            "" if s.eph is None else f"{s.eph:.2f}",
            "" if s.epv is None else f"{s.epv:.2f}",
            "" if s.fix_type is None else s.fix_type,
            "" if s.sats is None else s.sats
        ])
        _raw_file.flush()

def log_close():
    """Close all open log files."""
    global _log_file, _raw_file
    if _log_file: 
        _log_file.close()
    if _raw_file: 
        _raw_file.close()

def create_raw_logger(enabled: bool = True) -> Optional[Callable[[str, GpsSample], None]]:
    """
    Create a raw logging function that can be passed as callback.
    
    Args:
        enabled (bool): Whether raw logging is enabled
        
    Returns:
        Optional[Callable]: Raw logging function or None if disabled
    """
    return log_raw if (enabled and LOG_RAW_GPS) else None

def set_logging_config(csv_logging: bool = True, raw_gps_logging: bool = True):
    """
    Configure logging settings (must be called before log_open).
    
    Args:
        csv_logging (bool): Enable main CSV logging
        raw_gps_logging (bool): Enable raw GPS logging
    """
    global LOG_TO_CSV, LOG_RAW_GPS
    LOG_TO_CSV = csv_logging
    LOG_RAW_GPS = raw_gps_logging