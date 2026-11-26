#!/usr/bin/env python3
"""Replay KITTI Raw unsynced OXTS through the shared C++ INS example.

Only oxts/data/*.txt and oxts/timestamps.txt are needed. OXTS supplies both
inertial samples and navigation estimates, so this is not an independent
GNSS/IMU accuracy benchmark. No images, LiDAR, or camera calibration are used.
"""

import argparse
import csv
import json
from datetime import datetime
import hashlib
import math
from pathlib import Path
import statistics
import subprocess

from runner import (NAVIGATION_HEADER, PROGRESS_FORMAT,
                    check_output, dot, executable, project_coordinates, replace_output,
                    run_estimator, write_csv, write_metadata)
from plots import plot_replay


OXTS_FIELDS = (
    "lat lon alt roll pitch yaw vn ve vf vl vu ax ay az af al au wx wy wz wf wl wu "
    "pos_accuracy vel_accuracy navstat numsats posmode velmode orimode"
).split()
# Demo observation standard deviations: horizontal position, vertical position,
# and velocity. OXTS does not supply the full independent observation covariance.
# These match the shared C++ replay floors, not a calibrated OXTS noise model.
NAVIGATION_STD = (1.0, 1.5, 0.3)
# Explicit demo gap policy, not reconstructed measurements or calibrated noise.
GAP_THRESHOLD_US = 50000


def timestamp_ns(TEXT):
    """Parse KITTI's local date/time without float epoch rounding or timezone conversion."""
    SECONDS, SEPARATOR, FRACTION = TEXT.strip().partition(".")
    if SEPARATOR != "." or not FRACTION.isascii() or not FRACTION.isdigit() or len(FRACTION) > 9:
        raise ValueError(f"invalid KITTI timestamp: {TEXT!r}")
    DATE = datetime.strptime(SECONDS, "%Y-%m-%d %H:%M:%S")
    WHOLE_SECONDS = DATE.toordinal() * 86400 + DATE.hour * 3600 + DATE.minute * 60 + DATE.second
    return WHOLE_SECONDS * 1000000000 + int(FRACTION.ljust(9, "0"))


def read_oxts(DRIVE):
    from tqdm import tqdm as TQDM

    OXTS = DRIVE / "oxts"
    STAMPS = (OXTS / "timestamps.txt").read_text().splitlines()
    FILES = sorted((OXTS / "data").glob("*.txt"))
    if len(FILES) < 2 or len(FILES) != len(STAMPS):
        raise ValueError("expected matching OXTS timestamps and at least two numbered data files")
    HASH = hashlib.sha256()
    RECORDS = []
    for INDEX, (FILE, STAMP) in enumerate(TQDM(zip(FILES, STAMPS), total=len(FILES),
                                            desc="Read OXTS", dynamic_ncols=True)):
        if FILE.name != f"{INDEX:010d}.txt":
            raise ValueError(f"missing or misnumbered OXTS frame before {FILE.name}")
        TIME = timestamp_ns(STAMP)
        TEXT = FILE.read_text()
        VALUES = [float(VALUE) for VALUE in TEXT.split()]
        if len(VALUES) != len(OXTS_FIELDS) or not all(math.isfinite(VALUE) for VALUE in VALUES):
            raise ValueError(f"{FILE.name}: expected 30 finite OXTS fields")
        PACKET = dict(zip(OXTS_FIELDS, VALUES))
        if abs(PACKET["lat"]) > 90 or abs(PACKET["lon"]) > 180:
            raise ValueError(f"{FILE.name}: latitude/longitude out of range")
        if (abs(PACKET["roll"]) > math.pi or abs(PACKET["pitch"]) > math.pi / 2
                or abs(PACKET["yaw"]) > math.pi):
            raise ValueError(f"{FILE.name}: roll/pitch/yaw out of range")
        MODES = [PACKET[KEY] for KEY in ("posmode", "velmode", "orimode")]
        if any(MODE != int(MODE) or not 0 <= MODE <= 30 for MODE in MODES):
            raise ValueError(f"{FILE.name}: invalid OXTS navigation modes; interpolated (-1) "
                             "outage records are not accepted as measured IMU data")
        if PACKET["navstat"] != int(PACKET["navstat"]) or not 0 <= PACKET["navstat"] <= 23:
            raise ValueError(f"{FILE.name}: invalid navigation status")
        HASH.update(FILE.name.encode() + b"\n" + STAMP.encode() + b"\n" + TEXT.encode() + b"\n")
        RECORDS.append((TIME, PACKET))
    RECORDS, TIMING = order_records(RECORDS)
    return RECORDS, HASH.hexdigest(), TIMING


def order_records(RECORDS):
    """Offline ordering keeps each packet paired with its original timestamp."""
    REVERSALS = sum(B[0] < A[0] for A, B in zip(RECORDS, RECORDS[1:]))
    ORDERED = sorted(RECORDS, key=lambda RECORD: RECORD[0])
    ORIGIN_NS = ORDERED[0][0]
    UNIQUE = []
    DUPLICATES = 0
    for TIME_NS, PACKET in ORDERED:
        TIME = 1000000 + (TIME_NS - ORIGIN_NS) // 1000
        if UNIQUE and TIME == UNIQUE[-1][0]:
            if TIME_NS != PREVIOUS_NS or PACKET != UNIQUE[-1][1]:
                raise ValueError("conflicting OXTS packets or timestamp collision at microsecond resolution")
            DUPLICATES += 1
            continue
        UNIQUE.append((TIME, PACKET))
        PREVIOUS_NS = TIME_NS
    INTERVALS = [B[0] - A[0] for A, B in zip(UNIQUE, UNIQUE[1:])]
    if not INTERVALS or not 5000 <= statistics.median(INTERVALS) <= 20000:
        raise ValueError("use Raw unsynced/extract OXTS (~100 Hz), not synced (~10 Hz)")
    return UNIQUE, {"source_rows": len(RECORDS), "timestamp_reversals": REVERSALS,
                    "identical_duplicates_removed": DUPLICATES}


def navigation_valid(PACKET):
    # navstat=4 is a locked INS solution. GPS modes describe receiver aiding,
    # not validity of the fused attitude; orimode=0 does NOT invalidate q_nb.
    # Do not aid from position/velocity modes None, Search or unknown solutions.
    # Source: OxfordTechnicalSolutions/NCOMdecoder, NavigationStatus/GpsXModeName.
    return PACKET["navstat"] == 4 and 3 <= PACKET["posmode"] <= 9 and 2 <= PACKET["velmode"] <= 9


def initial_quaternion(PACKET):
    """KITTI FLU -> ENU attitude converted to scalar-first FRD -> NED q_nb.

    R_nb = C_enu_to_ned * Rz(yaw) * Ry(pitch) * Rx(roll) * C_frd_to_flu.
    Equivalently, the NED/FRD Euler angles are (roll, -pitch, pi/2 - yaw).
    KITTI's yaw is zero toward East and increases toward North.
    """
    ROLL, PITCH, YAW = PACKET["roll"], -PACKET["pitch"], math.pi / 2 - PACKET["yaw"]
    CR, SR = math.cos(ROLL / 2), math.sin(ROLL / 2)
    CP, SP = math.cos(PITCH / 2), math.sin(PITCH / 2)
    CY, SY = math.cos(YAW / 2), math.sin(YAW / 2)
    return [CR * CP * CY + SR * SP * SY, SR * CP * CY - CR * SP * SY,
            CR * SP * CY + SR * CP * SY, CR * CP * SY - SR * SP * CY]


def inertial_sample(PACKET):
    # Use body xyz, NOT the level forward/left/up (af/al/au, wf/wl/wu) fields.
    # FLU -> FRD is a proper rotation: flip Y and Z for both polar/axial vectors.
    # ax/ay/az are accelerometer specific force (level rest: az ~= +g).
    # Do not subtract gravity here: the core adds gravity in navigation axes.
    return [PACKET["ax"], -PACKET["ay"], -PACKET["az"],
            PACKET["wx"], -PACKET["wy"], -PACKET["wz"]]


def session_settings(NAVIGATION_HZ):
    # Demo-only quality/noise profile. Synthetic DOP cannot participate in a check.
    return dict(model_provider=0, model_heading=0, saved_heading=0, declination_rad=0,
                gnss_gate=5, position_noise=1, velocity_noise=0.3, position_noise_limit=10,
                gnss_checks=29, minimum_satellites=6, maximum_pdop=2.5, maximum_eph=3,
                maximum_epv=5, maximum_sacc=0.5, maximum_hdrift=0.1, maximum_vdrift=0.2,
                gnss_health_time_us=10000000, magnetic_checks=1, field_strength_tolerance=20,
                field_inclination_tolerance=math.radians(20), heading_noise=0.3,
                heading_acceleration_limit=0.5, magnetic_noise_variance=25, magnetic_gate=3,
                bias_force_limit=25, bias_rate_limit=3, bias_decay_time=0.5,
                accelerometer_bias_learning=1, gyroscope_bias_learning=1,
                accelerometer_bias_limit=0.4, gyroscope_bias_limit=0.15,
                gnss_spacing_us=round(1000000 / NAVIGATION_HZ))


def navigation_row(TIME, PACKET):
    VALID = navigation_valid(PACKET)
    return [TIME, 1, PACKET["lat"], PACKET["lon"], PACKET["alt"], PACKET["alt"],
            PACKET["vn"], PACKET["ve"], -PACKET["vu"],
            max(PACKET["pos_accuracy"], NAVIGATION_STD[0]),
            max(PACKET["pos_accuracy"], NAVIGATION_STD[1]),
            max(PACKET["vel_accuracy"], NAVIGATION_STD[2]),
            1, 1, 3 if VALID else 2, int(PACKET["numsats"]), int(VALID), 0]


def prepare(RECORDS, NAVIGATION_HZ, BINARY):
    """One independent session per continuous sequence; no interpolation over outages."""
    GROUPS = [[]]
    GAPS = []
    for TIME, PACKET in RECORDS:
        if GROUPS[-1] and TIME - GROUPS[-1][-1][0] > GAP_THRESHOLD_US:
            GAPS.append([GROUPS[-1][-1][0], TIME])
            GROUPS.append([])
        GROUPS[-1].append((TIME, PACKET))
    SEGMENTS, SKIPPED, ALL_GPS = [], [], []
    PERIOD = round(1000000 / NAVIGATION_HZ)
    for INDEX, GROUP in enumerate(GROUPS, 1):
        PAIRS = list(zip(GROUP, GROUP[1:]))
        FIRST = next((I for I, (_, (_, P)) in enumerate(PAIRS) if navigation_valid(P)), None)
        if FIRST is None:
            SKIPPED.append(INDEX)
            continue
        PAIRS = PAIRS[FIRST:]
        TIME, SEED = PAIRS[0][1]
        GPS_SEED = navigation_row(TIME, SEED)
        PRIOR = [TIME, *initial_quaternion(SEED), math.radians(1)**2, math.radians(1)**2,
                 math.radians(5)**2, SEED["lat"], SEED["lon"], SEED["alt"], SEED["alt"],
                 SEED["vn"], SEED["ve"], -SEED["vu"],
                 GPS_SEED[9]**2, GPS_SEED[9]**2, GPS_SEED[10]**2, *([GPS_SEED[11]**2] * 3)]
        IMU, GPS = [], []
        NEXT_UPDATE = TIME + PERIOD  # The prior packet is not fused again.
        for (PREVIOUS_TIME, PREVIOUS), (TIME, PACKET) in PAIRS:
            DT = TIME - PREVIOUS_TIME
            if not 2000 <= DT <= GAP_THRESHOLD_US:
                raise ValueError(f"OXTS interval {DT} us is outside the replay timing profile")
            IMU.append([TIME, DT, DT, *inertial_sample(PREVIOUS), -1, 1, 2, 0, 0])
            if TIME >= NEXT_UPDATE:
                GPS.append(navigation_row(TIME, PACKET))
                NEXT_UPDATE = TIME + PERIOD
        SEGMENTS.append(dict(segment=INDEX, prior=PRIOR, imu=IMU, gnss=GPS))
        ALL_GPS.extend(GPS)
    if not SEGMENTS:
        raise ValueError("no locked OXTS navigation solution at a real IMU endpoint")
    ORIGIN = SEGMENTS[0]["prior"][8:11]
    COORDINATES = project_coordinates(BINARY, ORIGIN, [[*ROW[2:4], ROW[5]] for ROW in ALL_GPS]) if ALL_GPS else []
    NAVIGATION = [[ROW[0], *P, *[dot(AXIS, ROW[6:9]) for AXIS in R], *ROW[9:12]]
                  for ROW, (P, R) in zip(ALL_GPS, COORDINATES)]
    META = dict(dataset="KITTI Raw unsynced OXTS", imu_rows=sum(len(S["imu"]) for S in SEGMENTS),
                navigation_rows=len(ALL_GPS), navigation_hz=NAVIGATION_HZ,
                segments=[dict(segment=S["segment"], seed_timestamp_us=S["prior"][0],
                               end_timestamp_us=S["imu"][-1][0]) for S in SEGMENTS],
                skipped_segments_without_seed_interval=SKIPPED, imu_gaps=GAPS, synthetic_imu_rows=0,
                origin_lat_deg=ORIGIN[0], origin_lon_deg=ORIGIN[1], origin_alt_m=ORIGIN[2],
                initialization="one coherent OXTS attitude/position/velocity prior per segment; zero local position; "
                    "tilt std 1 deg/yaw std 5 deg are demo priors; startup bias uncertainty unchanged; no fabricated aiding",
                gap_policy="new session after >50 ms; no IMU/output in gap; no bias/P/quality history carried over",
                imu_sampling="left-endpoint hold over actual observed intervals, FLU to FRD; seed interval not integrated twice",
                navigation_source="decimated fused OXTS navigation, not independent raw GNSS; locked aiding maps to fix type 3",
                synthetic_dop="HDOP=VDOP=1 placeholders; PDOP check disabled",
                height="OXTS alt interpreted as ellipsoid and MSL: explicit zero geoid-offset demo assumption",
                source_identity="fixed IDs for one OXTS stream; no hardware/calibration identity evidence",
                clipping="unavailable; not reported as known clear", rest="unavailable; never asserted",
                coordinate_conversion="formal_eskf::geodesy; fixed display frame, session-owned working/navigation frames",
                timing="nominal 10 ms, actual intervals 2..50 ms; 200 ms requested lag determines conservative "
                    "101-sample history; actual lag is reported, typically about 1 s; sensor delay zero",
                uncertainty="positive OXTS pos/vel accuracy with demo std floors; independent axis priors, no upstream cross covariance",
                settings=session_settings(NAVIGATION_HZ), claim="demonstration, not independent accuracy evidence")
    return SEGMENTS, NAVIGATION, META


INITIAL_NAVIGATION_HEADER = (
    "timestamp_us,q0,q1,q2,q3,attitude_variance_n,attitude_variance_e,attitude_variance_d,"
    "latitude_deg,longitude_deg,altitude_ellipsoid_m,altitude_msl_m,vn,ve,vd,"
    "position_variance_n,position_variance_e,position_variance_d,velocity_variance_n,velocity_variance_e,velocity_variance_d"
)


def write_segment(INPUT, SEGMENT, SETTINGS, ORIGIN):
    INPUT.mkdir(parents=True, exist_ok=False)
    write_csv(INPUT / "settings.csv", "name,value", SETTINGS.items())
    write_csv(INPUT / "initial_navigation.csv", INITIAL_NAVIGATION_HEADER, [SEGMENT["prior"]])
    write_csv(INPUT / "imu.csv", "timestamp_us,gyro_dt_us,accel_dt_us,fx,fy,fz,wx,wy,wz,clipping,accel_id,gyro_id,accel_calibration,gyro_calibration", SEGMENT["imu"])
    write_csv(INPUT / "gnss.csv", "timestamp_us,device_id,latitude_deg,longitude_deg,altitude_msl_m,altitude_ellipsoid_m,vn,ve,vd,eph,epv,sacc,hdop,vdop,fix_type,satellites,velocity_valid,spoofed", SEGMENT["gnss"])
    write_csv(INPUT / "magnetometer.csv", "timestamp_us,device_id,calibration_count,mx,my,mz", [])
    write_csv(INPUT / "land.csv", "timestamp_us,landed,at_rest", [])
    write_csv(INPUT / "origin.csv", "latitude_deg,longitude_deg,height_ellipsoid_m", [ORIGIN])


def combine_results(OUTPUT, SEGMENTS):
    """Concatenate labelled outputs; counters remain per session and are also summed."""
    RESULT = OUTPUT / "result"
    RESULT.mkdir(exist_ok=False)
    SUMMARIES = []
    for NAME in ("state.csv", "status.csv", "fusion.csv"):
        with (RESULT / NAME).open("x", newline="") as TARGET:
            WRITER = None
            for SEGMENT in SEGMENTS:
                ID = SEGMENT["segment"]
                with (OUTPUT / "segments" / str(ID) / "result" / NAME).open() as SOURCE:
                    READER = csv.DictReader(SOURCE)
                    FIELDS = list(READER.fieldnames)
                    if "segment" not in FIELDS:
                        FIELDS.insert(1, "segment")
                    if WRITER is None:
                        WRITER = csv.DictWriter(TARGET, fieldnames=FIELDS)
                        WRITER.writeheader()
                    for ROW in READER:
                        ROW["segment"] = ID
                        WRITER.writerow(ROW)
    for SEGMENT in SEGMENTS:
        ID = SEGMENT["segment"]
        SUMMARY = json.loads((OUTPUT / "segments" / str(ID) / "result/summary.json").read_text())
        SUMMARIES.append(dict(segment=ID, **SUMMARY))
    TOTALS = {KEY: sum(S[KEY] for S in SUMMARIES) for KEY in (
        "predictions", "gnss_corrections", "gravity_corrections", "magnetic_corrections",
        "imu_admissions", "imu_rejections", "arrival_rejections", "rate_rejections", "overflows")}
    write_metadata(RESULT / "summary.json", dict(**TOTALS, segments=SUMMARIES))
    return TOTALS


def run_replay(DRIVE, BINARY, NAVIGATION_HZ):
    from tqdm import tqdm as TQDM

    DRIVE, BINARY = DRIVE.resolve(), executable(BINARY)
    if not DRIVE.is_dir() or not (DRIVE / "oxts").is_dir():
        raise ValueError(f"expected a KITTI drive directory containing oxts/: {DRIVE}")
    OUTPUT = Path("output") / ("kitti-" + DRIVE.name)
    check_output(OUTPUT, [DRIVE, DRIVE / "oxts", BINARY])
    RECORDS, HASH, TIMING = read_oxts(DRIVE)
    SEGMENTS, NAVIGATION, META = prepare(RECORDS, NAVIGATION_HZ, BINARY)
    if replace_output(OUTPUT):
        print(f"Removed previous replay output: {OUTPUT}")
    INPUT = OUTPUT / "input"
    INPUT.mkdir(parents=True, exist_ok=False)
    write_csv(INPUT / "navigation.csv", NAVIGATION_HEADER, NAVIGATION)
    META.update(source=str(DRIVE), oxts_sha256=HASH, **TIMING)
    write_metadata(INPUT / "source.json", META)
    ORIGIN = [META["origin_lat_deg"], META["origin_lon_deg"], META["origin_alt_m"]]
    with TQDM(total=len(SEGMENTS), desc="Replay independent segments", dynamic_ncols=True,
              bar_format=PROGRESS_FORMAT) as PROGRESS:
        for SEGMENT in SEGMENTS:
            ROOT = OUTPUT / "segments" / str(SEGMENT["segment"])
            write_segment(ROOT / "input", SEGMENT, META["settings"], ORIGIN)
            run_estimator(BINARY, ROOT / "input", ROOT / "result", KITTI=True)
            PROGRESS.update(1)
    TOTALS = combine_results(OUTPUT, SEGMENTS)
    print(f"{len(SEGMENTS)} independent sessions; {len(META['imu_gaps'])} IMU gaps; no synthetic IMU rows")
    print(f"Delayed predictions: {TOTALS['predictions']}; GNSS corrections: {TOTALS['gnss_corrections']}")
    print(f"Output: {OUTPUT}", flush=True)
    return OUTPUT


def main():
    PARSER = argparse.ArgumentParser(description=__doc__,
                                     epilog="Results replace output/kitti-<drive directory name>/ on each run.")
    PARSER.add_argument("drive", type=Path, help="unsynced/extract drive directory containing oxts/")
    PARSER.add_argument("--binary", type=Path, default=Path("build/demo/replay_ins"),
                        help="prebuilt C++ executable (default: build/demo/replay_ins)")
    PARSER.add_argument("--navigation-hz", type=float, default=10.0,
                        help="Maximum OXTS position/velocity input rate, 0.1 to 100 Hz (default: 10 Hz)")
    PARSER.add_argument("--plot", action="store_true", help="save PNG plots and display them with matplotlib")
    ARGS = PARSER.parse_args()
    if not math.isfinite(ARGS.navigation_hz) or not 0.1 <= ARGS.navigation_hz <= 100:
        PARSER.error("--navigation-hz must be finite and in [0.1, 100]")
    try:
        if ARGS.plot:
            import matplotlib.pyplot
        OUTPUT = run_replay(ARGS.drive, ARGS.binary, ARGS.navigation_hz)
        if ARGS.plot:
            plot_replay(OUTPUT, SOURCE_LABEL="OXTS navigation")
    except subprocess.CalledProcessError as ERROR:
        PARSER.exit(1, f"error: C++ replay failed (exit {ERROR.returncode})\n")
    except ImportError as ERROR:
        PARSER.exit(2, f"error: {ERROR}; install dependencies with: "
                      "python3 -m pip install -r requirements.txt\n")
    except (OSError, KeyError, ValueError) as ERROR:
        PARSER.exit(2, f"error: {ERROR}\n")


if __name__ == "__main__":
    main()
