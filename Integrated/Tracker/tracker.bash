#!/bin/bash
set -e

echo "[RUN] Preflight GPS check…"
python3 gps_check.py --drone /dev/ttyUSB1 --base /dev/ttyACM1 --rate-hz 5 --wait-s 4
# or: python3 gps_check.py --skip-base  # if you don’t have a base port

echo "[RUN] Starting tracker…"
python3 geo_13.py --mode ground --base-mode static
