#!/usr/bin/env python3
"""
Testbench for azi_elev_5 angle calculations with geopy.

Usage:
  python testbench_angles.py [--module azi_elev_5] [--base-lat 13.0276844 --base-lon 77.5631084 --base-alt 931.13] [--verbose]

What it does:
- Validates azimuth for cardinal directions (N/E/S/W)
- Validates elevation for overhead, positive, and negative altitude deltas
- Validates adjust_angles_for_servo_limits() clamping/normalization
- Performs a sweep test: targets placed at fixed radius around base at multiple bearings
"""

import importlib
import math
import argparse
from geopy import Point
from geopy.distance import geodesic, distance as geopy_distance

# ---------- CLI ----------
parser = argparse.ArgumentParser()
parser.add_argument("--module", default="azi_elev_5",
                    help="Module name containing the functions (default: azi_elev_5)")
parser.add_argument("--base-lat", type=float, default=13.0276844)
parser.add_argument("--base-lon", type=float, default=77.5631084)
parser.add_argument("--base-alt", type=float, default=931.13)
parser.add_argument("--verbose", action="store_true")
args = parser.parse_args()

mod = importlib.import_module(args.module)

BASE_LAT = args.base_lat
BASE_LON = args.base_lon
BASE_ALT = args.base_alt

# Tolerances
AZ_TOL_DEG = 2.0          # acceptable azimuth error (deg)
EL_TOL_DEG = 0.6          # acceptable elevation error (deg)
DIST_TOL_M = 5.0          # acceptable distance error (m) for internal consistency

# Helpers
def deg_wrap(x):
    return (x + 360.0) % 360.0

def assert_close(label, got, exp, tol):
    ok = (got is not None) and (abs(got - exp) <= tol)
    return ok, f"{label}: got={got:.2f}, exp={exp:.2f}, tol=±{tol}"

def header(title):
    print("\n" + "="*len(title))
    print(title)
    print("="*len(title))

def print_result(name, ok, detail):
    status = "PASS" if ok else "FAIL"
    print(f"[{status}] {name:35s} | {detail}")

# ---------- Fixed Tests ----------
def run_fixed_tests():
    base = Point(latitude=BASE_LAT, longitude=BASE_LON)

    tests = []
    # Cardinal points at ~1 km offset
    # We construct targets by moving 1000 m from the base at given bearings
    radius_m = 1000.0
    for bearing, expected_az in [(0, 0.0), (90, 90.0), (180, 180.0), (270, 270.0)]:
        tgt_point = geopy_distance(meters=radius_m).destination(base, bearing=bearing)
        tests.append({
            "name": f"Azimuth cardinal {bearing}°",
            "target": (tgt_point.latitude, tgt_point.longitude, BASE_ALT),
            "exp_az": expected_az,
            "exp_el": 0.0
        })

    # Overhead: same lat/lon, higher altitude
    tests.append({
        "name": "Overhead (same lat/lon, +100 m alt)",
        "target": (BASE_LAT, BASE_LON, BASE_ALT + 100.0),
        "exp_az": 0.0,      # bearing undefined; module returns 0 in this case
        "exp_el": 90.0
    })

    # Positive elevation (~5.7°): 100 m up at ~1 km east
    tgt_east = geopy_distance(meters=1000.0).destination(base, bearing=90)
    tests.append({
        "name": "Elev + (~100m up @1km)",
        "target": (tgt_east.latitude, tgt_east.longitude, BASE_ALT + 100.0),
        "exp_az": 90.0,
        # compute expected elevation using same geometry: atan2(100, 1000)
        "exp_el": math.degrees(math.atan2(100.0, 1000.0))
    })

    # Negative elevation: 50 m down @1km east ⇒ raw elevation negative, adjusted clamps to 0
    tests.append({
        "name": "Elev - (raw <0; adjusted=0)",
        "target": (tgt_east.latitude, tgt_east.longitude, BASE_ALT - 50.0),
        "exp_az": 90.0,
        "exp_el_raw": math.degrees(math.atan2(-50.0, 1000.0)),  # ~ -2.86°
        "exp_el_adj": 0.0
    })

    # Run tests
    header("Fixed Tests")
    pass_count = 0
    for t in tests:
        name = t["name"]
        lat, lon, alt = t["target"]
        az, el = mod.calculate_azimuth_elevation(BASE_LAT, BASE_LON, BASE_ALT, lat, lon, alt)
        if "exp_el_raw" in t:
            # This case checks both raw and adjusted
            ok1, det1 = assert_close("Azimuth", deg_wrap(az), t["exp_az"], AZ_TOL_DEG)
            ok2, det2 = assert_close("Elevation (raw)", el, t["exp_el_raw"], EL_TOL_DEG)
            adj_az, adj_el = mod.adjust_angles_for_servo_limits(az, el)
            ok3, det3 = assert_close("Elevation (adjusted)", adj_el, t["exp_el_adj"], EL_TOL_DEG)
            ok = ok1 and ok2 and ok3
            detail = f"{det1} | {det2} | {det3}"
        else:
            ok1, det1 = assert_close("Azimuth", deg_wrap(az), t["exp_az"], AZ_TOL_DEG)
            ok2, det2 = assert_close("Elevation", el, t["exp_el"], EL_TOL_DEG)
            ok = ok1 and ok2
            detail = f"{det1} | {det2}"

        print_result(name, ok, detail)
        pass_count += int(ok)

        if args.verbose:
            print(f"  -> got az={az:.2f}, el={el:.2f} for target=({lat:.6f}, {lon:.6f}, {alt:.2f})")

    print(f"\nFixed tests: {pass_count}/{len(tests)} passed.")
    return pass_count, len(tests)

# ---------- Adjust Function Tests ----------
def run_adjust_tests():
    header("Adjust Function Tests")
    tests = [
        # az wrap: -10 -> 350
        {"name": "Az wrap -10° -> 350°", "in": (-10.0, 45.0), "exp": (350.0, 45.0)},
        # el clamp high: 120 -> 90
        {"name": "El clamp 120° -> 90°", "in": (10.0, 120.0), "exp": (10.0, 90.0)},
        # el clamp low: -15 -> 0
        {"name": "El clamp -15° -> 0°", "in": (10.0, -15.0), "exp": (10.0, 0.0)},
    ]
    pass_count = 0
    for t in tests:
        az_in, el_in = t["in"]
        az_out, el_out = mod.adjust_angles_for_servo_limits(az_in, el_in)
        ok1, det1 = assert_close("Azimuth", az_out, t["exp"][0], 0.01)
        ok2, det2 = assert_close("Elevation", el_out, t["exp"][1], 0.01)
        ok = ok1 and ok2
        print_result(t["name"], ok, f"{det1} | {det2}")
        pass_count += int(ok)
        if args.verbose:
            print(f"  -> input (az,el)=({az_in},{el_in}) -> output (az,el)=({az_out},{el_out})")
    print(f"\nAdjust tests: {pass_count}/{len(tests)} passed.")
    return pass_count, len(tests)

# ---------- Sweep Tests (1 km ring around base) ----------
def run_sweep_tests():
    header("Sweep Tests (1 km ring, bearings 0..315 by 45)")
    base = Point(latitude=BASE_LAT, longitude=BASE_LON)

    bearings = list(range(0, 360, 45))  # 0,45,...,315
    radius_m = 1000.0
    alt_deltas = [0.0, 100.0]           # flat and +100 m up

    total = 0
    passed = 0
    for d_alt in alt_deltas:
        for b in bearings:
            tgt_point = geopy_distance(meters=radius_m).destination(base, bearing=b)
            tgt_lat, tgt_lon = tgt_point.latitude, tgt_point.longitude
            tgt_alt = BASE_ALT + d_alt

            az, el = mod.calculate_azimuth_elevation(BASE_LAT, BASE_LON, BASE_ALT,
                                                     tgt_lat, tgt_lon, tgt_alt)
            # Expected az ~ bearing
            ok_az, det_az = assert_close("Azimuth", deg_wrap(az), float(b), AZ_TOL_DEG)

            # Expected elevation based on geometry
            exp_el = math.degrees(math.atan2(d_alt, radius_m)) if radius_m != 0 else (90.0 if d_alt > 0 else 0.0)
            ok_el, det_el = assert_close("Elevation", el, exp_el, EL_TOL_DEG)

            ok = ok_az and ok_el
            total += 1
            passed += int(ok)

            label = f"Bearing {b:3d}°, Δalt {d_alt:.0f} m"
            print_result(label, ok, f"{det_az} | {det_el}")

            if args.verbose:
                print(f"  -> got az={az:.2f}, el={el:.2f} | exp el={exp_el:.2f}")

    print(f"\nSweep tests: {passed}/{total} passed.")
    return passed, total

# ---------- Distance/Bearing Consistency ----------
def run_distance_bearing_consistency():
    header("Distance/Bearing Consistency Checks")
    base = Point(latitude=BASE_LAT, longitude=BASE_LON)
    bearings = [0, 60, 120, 180, 240, 300]
    radii = [250, 1000, 5000]  # meters

    total = 0
    passed = 0
    for r in radii:
        for b in bearings:
            tgt_point = geopy_distance(meters=r).destination(base, bearing=b)
            tgt_lat, tgt_lon = tgt_point.latitude, tgt_point.longitude
            dist_mod, bearing_mod = mod.calculate_distance_and_bearing(BASE_LAT, BASE_LON, tgt_lat, tgt_lon)

            # Expected distance via geopy directly
            dist_true = geodesic(base, tgt_point).meters
            ok_d, det_d = assert_close("Distance", dist_mod, dist_true, DIST_TOL_M)

            # Expected bearing ~ b (for small arcs at these distances)
            ok_b, det_b = assert_close("Bearing", bearing_mod, float(b), AZ_TOL_DEG)

            ok = ok_d and ok_b
            total += 1
            passed += int(ok)

            label = f"r={r:5d} m @ {b:3d}°"
            print_result(label, ok, f"{det_d} | {det_b}")

            if args.verbose:
                print(f"  -> got dist={dist_mod:.2f} m (true {dist_true:.2f}), bearing={bearing_mod:.2f}°")

    print(f"\nDistance/Bearing checks: {passed}/{total} passed.")
    return passed, total

# ---------- Main ----------
if __name__ == "__main__":
    print(f"Base: lat={BASE_LAT}, lon={BASE_LON}, alt={BASE_ALT} m")
    total_pass = 0
    total_cnt = 0

    p, n = run_fixed_tests()
    total_pass += p; total_cnt += n

    p, n = run_adjust_tests()
    total_pass += p; total_cnt += n

    p, n = run_sweep_tests()
    total_pass += p; total_cnt += n

    p, n = run_distance_bearing_consistency()
    total_pass += p; total_cnt += n

    print("\n" + "="*40)
    print(f"OVERALL: {total_pass}/{total_cnt} tests passed")
    print("="*40)
