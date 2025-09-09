import config
from datetime import datetime

# ===================== CSV logging =====================

_log_writer = None
_log_file   = None

def open(prefix="Tracker"):
    global _log_writer, _log_file, _raw_writer, _raw_file
    if not config.LOG_TO_CSV and not config.LOG_RAW_GPS:
        return
    import csv, pathlib
    pathlib.Path("Tracker_Logs").mkdir(exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d-%H%M%S")
    if config.LOG_TO_CSV:
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
            "BaseLatSDm","BaseLonSDm","BaseFix","BaseSats",
            "UsedFlip"])

        print(f"[INFO] CSV log → {fn}")
    if config.LOG_RAW_GPS:
        fnr = pathlib.Path(f"Tracker_Logs/{prefix}_RAW_{ts}.csv")
        _raw_file = fnr.open("w", newline="")
        _raw_writer = csv.writer(_raw_file)
        _raw_writer.writerow(["Time","Stream","Lat","Lon","Alt","eph(m)","epv(m)","fix_type","sats"])
        print(f"[INFO] RAW GPS log → {fnr}")

def row(*row):
    if _log_writer:
        _log_writer.writerow(row)
        if _log_file:
            _log_file.flush()

def close():
    if _log_file: _log_file.close()
    if _raw_file: _raw_file.close()
