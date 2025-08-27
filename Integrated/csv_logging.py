import constants
from datetime import datetime
import GPS as gps

_log_writer = None
_log_file   = None
_raw_writer = None
_raw_file   = None

def log_open(prefix="Tracker"):
    global _log_writer, _log_file, _raw_writer, _raw_file
    if not constants.LOG_TO_CSV and not constants.LOG_RAW_GPS:
        return
    import csv, pathlib
    pathlib.Path("Tracker_Logs").mkdir(exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d-%H%M%S")
    if constants.LOG_TO_CSV:
        fn = pathlib.Path(f"Tracker_Logs/{prefix}_{ts}.csv")
        _log_file = fn.open("w", newline="")
        _log_writer = csv.writer(_log_file)
        _log_writer.writerow([
            "Time",
            "BaseMode","BaseLocked",
            "WorldAz","WorldEl","CalAz","CalEl",
            "PhysAz","PhysEl","ServoAz","ServoEl","Az(us)","El(us)",
            "DroneLat","DroneLon","DroneAlt",
            "BaseLat","BaseLon","BaseAlt",
            "BaseLatSDm","BaseLonSDm","BaseFix","BaseSats"
        ])
        print(f"[INFO] CSV log → {fn}")
    if constants.LOG_RAW_GPS:
        fnr = pathlib.Path(f"Tracker_Logs/{prefix}_RAW_{ts}.csv")
        _raw_file = fnr.open("w", newline="")
        _raw_writer = csv.writer(_raw_file)
        _raw_writer.writerow(["Time","Stream","Lat","Lon","Alt","eph(m)","epv(m)","fix_type","sats"])
        print(f"[INFO] RAW GPS log → {fnr}")

def log_row(*row):
    if _log_writer:
        _log_writer.writerow(row)
        _log_file.flush()

def log_raw(stream: str, s: gps.GpsSample):
    if _raw_writer:
        _raw_writer.writerow([datetime.now().isoformat(timespec='seconds'),
                              stream, f"{s.lat:.7f}", f"{s.lon:.7f}", f"{s.alt:.2f}",
                              "" if s.eph is None else f"{s.eph:.2f}",
                              "" if s.epv is None else f"{s.epv:.2f}",
                              "" if s.fix_type is None else s.fix_type,
                              "" if s.sats is None else s.sats])
        _raw_file.flush()

def log_close():
    if _log_file: _log_file.close()
    if _raw_file: _raw_file.close()