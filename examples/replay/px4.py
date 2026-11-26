#!/usr/bin/env python3
"""Replay a legacy PX4 ULog through the prebuilt C++ INS example.

Sensor inputs are sensor_combined, vehicle_gps_position and vehicle_magnetometer;
vehicle_land_detected supplies optional at_rest metadata.
PX4 estimator outputs are not inputs. InsSession owns IMU startup, tilt refinement,
heading alignment, delayed fusion, health and output availability. Missing optional
clipping/rest reports remain unavailable; they are not fabricated as healthy.
"""

import argparse
from bisect import bisect_right
import hashlib
import math
from pathlib import Path
import subprocess

from runner import (NAVIGATION_HEADER, PROGRESS_FORMAT,
                    check_output, dot, executable, project_coordinates, replace_output,
                    run_estimator, write_csv, write_metadata)
from plots import plot_replay


def topic(ULOG, NAME, REQUIRED=True):
    MATCHES = [DATA.data for DATA in ULOG.data_list if DATA.name == NAME and DATA.multi_id == 0]
    if not MATCHES and not REQUIRED:
        return {}
    if len(MATCHES) != 1:
        raise ValueError(f"expected one {NAME} instance 0")
    return MATCHES[0]


def timestamps(DATA):
    TIMES = [int(TIME) for TIME in DATA["timestamp"]]
    if not TIMES or TIMES[0] <= 0 or any(B <= A for A, B in zip(TIMES, TIMES[1:])):
        raise ValueError("topic timestamps must be nonzero and strictly increasing")
    return TIMES


def finite(VALUES):
    return all(math.isfinite(float(VALUE)) for VALUE in VALUES)


def xyz(DATA, FIELD, INDEX):
    return [float(DATA[f"{FIELD}[{AXIS}]"][INDEX]) for AXIS in range(3)]


def magnetic_declination(ULOG, MODEL_DECLINATION=None):
    """Select initial heading declination; no learned earth-field state is available."""
    MASK = float(ULOG.initial_parameters.get("EKF2_DECL_TYPE", 3))
    # Older logs also carry bit 2. It does not select an initialization source.
    if not math.isfinite(MASK) or not MASK.is_integer() or not 0 <= MASK <= 7:
        raise ValueError("invalid magnetic declination source mask")
    if int(MASK) & 1 and MODEL_DECLINATION is not None:
        if not math.isfinite(MODEL_DECLINATION):
            raise ValueError("invalid model declination")
        return math.degrees(MODEL_DECLINATION), "geographic_model"
    SAVED = float(ULOG.initial_parameters.get("EKF2_MAG_DECL", 0))
    if int(MASK) & 2 and math.isfinite(SAVED) and SAVED != 0:
        return SAVED, "saved_parameter"
    return 0.0, "magnetic_north"


def magnetic_heading_parameters(ULOG):
    """Logged heading consistency settings, with explicit defaults for older logs."""
    VALUES = [float(ULOG.initial_parameters.get("EKF2_HEAD_NOISE", 0.3)),
              float(ULOG.initial_parameters.get("EKF2_MAG_ACCLIM", 0.5))]
    if not finite(VALUES) or not 0 < VALUES[0] <= math.pi or VALUES[1] < 0:
        raise ValueError("invalid magnetic heading parameters")
    return VALUES


def magnetic_field_parameters(ULOG):
    """PX4 field-check selection; missing legacy settings use explicit v1.16.2 defaults."""
    VALUES = [float(ULOG.initial_parameters.get("EKF2_MAG_CHECK", 1)),
              float(ULOG.initial_parameters.get("EKF2_MAG_CHK_STR", 0.2)),
              float(ULOG.initial_parameters.get("EKF2_MAG_CHK_INC", 20.0))]
    if (not finite(VALUES) or not VALUES[0].is_integer() or not 0 <= VALUES[0] <= 7
            or not 0 <= VALUES[1] <= 1 or not 0 <= VALUES[2] <= 90):
        raise ValueError("invalid magnetic field check settings")
    return [int(VALUES[0]), VALUES[1] * 100.0, math.radians(VALUES[2])]


def gnss_parameters(ULOG, END):
    """Fixed settings for a sole-GNSS aiding replay; no lever-arm compensation."""
    NAMES = ("EKF2_GPS_P_GATE", "EKF2_GPS_V_GATE", "EKF2_GPS_P_NOISE", "EKF2_GPS_V_NOISE", "EKF2_NOAID_NOISE")
    POSITION_NAMES = tuple(f"EKF2_{SENSOR}_POS_{AXIS}" for SENSOR in ("GPS", "IMU") for AXIS in "XYZ")
    VALUES = [float(ULOG.initial_parameters[NAME]) for NAME in NAMES]
    OFFSETS = [float(ULOG.initial_parameters[NAME]) for NAME in POSITION_NAMES]
    if not finite([*VALUES, *OFFSETS]) or min(VALUES[2:]) <= 0:
        raise ValueError("invalid GNSS noise/gate/offset parameters")
    if OFFSETS[:3] != OFFSETS[3:]:
        raise ValueError("GNSS replay requires co-located GNSS/IMU; lever-arm compensation is not implemented")
    for TIME, NAME, _ in ULOG.changed_parameters:
        if TIME <= END and NAME in (*NAMES, *POSITION_NAMES):
            raise ValueError(f"changed {NAME} requires a new GNSS replay configuration")
    VALUES[:2] = [max(VALUE, 1.0) for VALUE in VALUES[:2]]
    return VALUES


def gnss_quality_parameters(ULOG, END):
    """Use recorded checks/limits, with the legacy name and fixed health-time default explicit."""
    PDOP_NAME = "EKF2_REQ_PDOP" if "EKF2_REQ_PDOP" in ULOG.initial_parameters else "EKF2_REQ_GDOP"
    NAMES = ("EKF2_GPS_CHECK", "EKF2_REQ_NSATS", PDOP_NAME, "EKF2_REQ_EPH", "EKF2_REQ_EPV",
             "EKF2_REQ_SACC", "EKF2_REQ_HDRIFT", "EKF2_REQ_VDRIFT")
    VALUES = [float(ULOG.initial_parameters[NAME]) for NAME in NAMES]
    HEALTH_SECONDS = float(ULOG.initial_parameters.get("EKF2_REQ_GPS_H", 10.0))
    HEALTH_US = round(HEALTH_SECONDS * 1e6) if math.isfinite(HEALTH_SECONDS) else 0
    if (not finite(VALUES) or not VALUES[0].is_integer() or not 0 <= VALUES[0] <= 1023
            or not VALUES[1].is_integer() or not 1 <= VALUES[1] <= 255
            or min(VALUES[2:]) <= 0 or not 1 <= HEALTH_US <= 4294967295):
        raise ValueError("invalid GNSS quality parameters")
    if ("EKF2_REQ_PDOP" in ULOG.initial_parameters and "EKF2_REQ_GDOP" in ULOG.initial_parameters
            and ULOG.initial_parameters["EKF2_REQ_PDOP"] != ULOG.initial_parameters["EKF2_REQ_GDOP"]):
        raise ValueError("conflicting GNSS PDOP parameter names")
    for TIME, NAME, _ in ULOG.changed_parameters:
        if TIME <= END and NAME in (*NAMES, "EKF2_REQ_PDOP", "EKF2_REQ_GDOP", "EKF2_REQ_GPS_H"):
            raise ValueError(f"changed {NAME} requires a new GNSS quality session")
    return [*VALUES, HEALTH_US]


def bias_parameters(ULOG, END):
    """Translate current or legacy logged bias controls into one fixed session."""
    NAMES = ("EKF2_ABL_ACCLIM", "EKF2_ABL_GYRLIM", "EKF2_ABL_TAU")
    VALUES = [float(ULOG.initial_parameters[NAME]) for NAME in NAMES]
    if not finite(VALUES) or any(VALUE <= 0 for VALUE in VALUES):
        raise ValueError("invalid bias magnitude/decay parameters")
    SOURCE = "EKF2_IMU_CTRL" if "EKF2_IMU_CTRL" in ULOG.initial_parameters else "EKF2_AID_MASK"
    MASK = float(ULOG.initial_parameters[SOURCE])
    if not math.isfinite(MASK) or not MASK.is_integer() or not 0 <= MASK <= 2**31 - 1:
        raise ValueError(f"invalid {SOURCE} bitmask")
    MASK = int(MASK)
    if SOURCE == "EKF2_IMU_CTRL":
        ACCEL, GYRO = bool(MASK & 2), bool(MASK & 1)
    else:
        # Legacy bit 2 inhibits accelerometer bias; no gyro-bias disable bit.
        ACCEL, GYRO = not bool(MASK & 4), True
    for TIME, NAME, _ in ULOG.changed_parameters:
        if TIME <= END and NAME in (*NAMES, "EKF2_IMU_CTRL", "EKF2_AID_MASK"):
            raise ValueError(f"changed {NAME} requires a new bias-learning session")
    return [*VALUES, int(ACCEL), int(GYRO)]


def bias_limits(ULOG, END):
    """Use logged per-axis bounds, with explicit EKF2 module-default fallbacks."""
    DEFAULTS = {"EKF2_ABL_LIM": 0.4, "EKF2_GYR_B_LIM": 0.15}
    VALUES = [float(ULOG.initial_parameters.get(NAME, DEFAULT)) for NAME, DEFAULT in DEFAULTS.items()]
    if not finite(VALUES) or any(VALUE < 0 for VALUE in VALUES):
        raise ValueError("invalid bias limits")
    for TIME, NAME, _ in ULOG.changed_parameters:
        if TIME <= END and NAME in DEFAULTS:
            raise ValueError(f"changed {NAME} requires a new bias-limit session")
    SOURCES = {NAME: "logged" if NAME in ULOG.initial_parameters else "fallback EKF2 module default"
               for NAME in DEFAULTS}
    return VALUES, SOURCES


def prepare(ULOG, OUTPUT, BINARY, MAGNETIC_MODEL=None):
    """Decode sensor data only; InsSession owns initialization, frames and fusion."""
    IMU = topic(ULOG, "sensor_combined")
    GPS = topic(ULOG, "vehicle_gps_position", REQUIRED=False)
    MAG = topic(ULOG, "vehicle_magnetometer", REQUIRED=False)
    TIMES = timestamps(IMU)
    GPS_TIMES, MAG_TIMES = timestamps(GPS) if GPS else [], timestamps(MAG) if MAG else []
    START, END = TIMES[0], TIMES[-1]
    GNSS = gnss_parameters(ULOG, END)
    QUALITY = gnss_quality_parameters(ULOG, END)
    if GPS and int(QUALITY[0]) & 512 and "spoofing_state" not in GPS:
        raise ValueError("enabled spoofing check requires logged spoofing_state")
    FIELD = magnetic_field_parameters(ULOG)
    HEADING = magnetic_heading_parameters(ULOG)
    BIAS = bias_parameters(ULOG, END)
    LIMITS, LIMIT_SOURCES = bias_limits(ULOG, END)
    if GNSS[0] != GNSS[1]:
        raise ValueError("the session profile requires equal GNSS position/velocity gates")
    DECLINATION, DECLINATION_SOURCE = magnetic_declination(ULOG)
    SETTINGS = dict(zip(("gnss_gate", "position_noise", "velocity_noise", "position_noise_limit"), [GNSS[0], *GNSS[2:]]))
    SETTINGS.update(zip(("gnss_checks", "minimum_satellites", "maximum_pdop", "maximum_eph", "maximum_epv",
                         "maximum_sacc", "maximum_hdrift", "maximum_vdrift", "gnss_health_time_us"), QUALITY))
    SETTINGS.update(zip(("magnetic_checks", "field_strength_tolerance", "field_inclination_tolerance"), FIELD))
    SETTINGS.update(zip(("heading_noise", "heading_acceleration_limit"), HEADING))
    SETTINGS.update(zip(("bias_force_limit", "bias_rate_limit", "bias_decay_time",
                         "accelerometer_bias_learning", "gyroscope_bias_learning"), BIAS))
    SETTINGS.update(zip(("accelerometer_bias_limit", "gyroscope_bias_limit"), LIMITS))
    SETTINGS.update(model_provider=int(MAGNETIC_MODEL is not None),
                    model_heading=int(bool(int(ULOG.initial_parameters.get("EKF2_DECL_TYPE", 3)) & 1)),
                    saved_heading=int(DECLINATION_SOURCE == "saved_parameter"),
                    declination_rad=math.radians(DECLINATION),
                    magnetic_noise_variance=(100 * float(ULOG.initial_parameters["EKF2_MAG_NOISE"]))**2,
                    magnetic_gate=float(ULOG.initial_parameters["EKF2_MAG_GATE"]))
    SELECTION = topic(ULOG, "sensor_selection", REQUIRED=False)
    SELECTION_TIMES = timestamps(SELECTION) if SELECTION else []

    def selected_id(NAME, TIME, FALLBACK):
        # A fixed fallback identifies an observed log stream, not absent hardware.
        if NAME not in SELECTION:
            return FALLBACK
        INDEX = bisect_right(SELECTION_TIMES, TIME) - 1
        return int(SELECTION[NAME][INDEX]) if INDEX >= 0 else 0

    for TIME, NAME, _ in ULOG.changed_parameters:
        if TIME <= END and NAME.startswith(("EKF2_MAG", "EKF2_DECL", "EKF2_HEAD", "SENS_BOARD")):
            raise ValueError(f"changed {NAME} requires a new replay configuration")
        if START <= TIME <= END and ((NAME.startswith("CAL_ACC") and "accel_calibration_count" not in IMU)
                                      or (NAME.startswith("CAL_GYRO") and "gyro_calibration_count" not in IMU)):
            raise ValueError("changed IMU calibration requires logged revision metadata")
        if TIME <= END and NAME.startswith("CAL_MAG") and "calibration_count" not in MAG:
            raise ValueError("changed magnetic calibration requires logged revision metadata")
    IMU_ROWS = []
    for INDEX, TIME in enumerate(TIMES):
        if int(IMU["accelerometer_timestamp_relative"][INDEX]) != 0:
            raise ValueError("replay requires matching accel/gyro endpoints")
        GYRO_DT, ACCEL_DT = int(IMU["gyro_integral_dt"][INDEX]), int(IMU["accelerometer_integral_dt"][INDEX])
        VALUES = xyz(IMU, "accelerometer_m_s2", INDEX) + xyz(IMU, "gyro_rad", INDEX)
        if not finite(VALUES) or not 0 < GYRO_DT <= 50000 or not 0 < ACCEL_DT <= 50000:
            raise ValueError("invalid IMU integration interval or payload")
        CLIPPING = float(IMU["accelerometer_clipping"][INDEX]) if "accelerometer_clipping" in IMU else -1.0
        if not math.isfinite(CLIPPING) or not CLIPPING.is_integer() or not -1 <= CLIPPING <= 7:
            raise ValueError("invalid accelerometer clipping mask")
        IMU_ROWS.append([TIME, GYRO_DT, ACCEL_DT, *VALUES, int(CLIPPING),
                         selected_id("accel_device_id", TIME, 1), selected_id("gyro_device_id", TIME, 2),
                         (int(IMU["accel_calibration_count"][INDEX]) if "accel_calibration_count" in IMU else 0),
                         (int(IMU["gyro_calibration_count"][INDEX]) if "gyro_calibration_count" in IMU else 0)])
    GPS_ROWS = []
    for INDEX, TIME in enumerate(GPS_TIMES):
        if TIME > END:
            continue
        GPS_ROWS.append([TIME, int(GPS["device_id"][INDEX]) if "device_id" in GPS else 1,
                         float(GPS["lat"][INDEX]) * 1e-7, float(GPS["lon"][INDEX]) * 1e-7,
                         float(GPS["alt"][INDEX]) * 1e-3, float(GPS["alt_ellipsoid"][INDEX]) * 1e-3,
                         *[float(GPS[NAME][INDEX]) for NAME in ("vel_n_m_s", "vel_e_m_s", "vel_d_m_s", "eph", "epv", "s_variance_m_s", "hdop", "vdop")],
                         int(GPS["fix_type"][INDEX]), int(GPS["satellites_used"][INDEX]), int(GPS["vel_ned_valid"][INDEX]),
                         int(int(GPS["spoofing_state"][INDEX]) == 3) if "spoofing_state" in GPS else 0])
    MAG_ROWS = [[TIME, int(MAG["device_id"][INDEX]) if "device_id" in MAG else selected_id("mag_device_id", TIME, 3),
                 int(MAG["calibration_count"][INDEX]) if "calibration_count" in MAG else 0,
                 *[VALUE * 100 for VALUE in xyz(MAG, "magnetometer_ga", INDEX)]]
                for INDEX, TIME in enumerate(MAG_TIMES) if TIME <= END]
    LAND = topic(ULOG, "vehicle_land_detected", REQUIRED=False)
    LAND_ROWS = []
    for INDEX, TIME in enumerate(timestamps(LAND) if LAND else []):
        LANDED = LAND["landed"][INDEX]
        AT_REST = LAND["at_rest"][INDEX] if "at_rest" in LAND else -1
        if LANDED not in (0, 1) or AT_REST not in (-1, 0, 1):
            raise ValueError("invalid land/rest flag")
        if TIME <= END:
            LAND_ROWS.append([TIME, int(LANDED), int(AT_REST)])
    VALID = [ROW for ROW in GPS_ROWS if ROW[14] >= 3 and finite(ROW[2:14])
             and abs(ROW[2]) <= 90 and abs(ROW[3]) <= 180 and -12000 <= ROW[5] <= 100000]
    ORIGIN = [*VALID[0][2:4], VALID[0][5]] if VALID else None
    PROJECTED = project_coordinates(BINARY, ORIGIN, [[*ROW[2:4], ROW[5]] for ROW in VALID]) if VALID else []
    NAVIGATION = [[ROW[0], *P, *[dot(AXIS, ROW[6:9]) for AXIS in R], *ROW[9:12]]
                  for ROW, (P, R) in zip(VALID, PROJECTED)]
    OUTPUT.mkdir(parents=True, exist_ok=False)
    write_csv(OUTPUT / "settings.csv", "name,value", SETTINGS.items())
    write_csv(OUTPUT / "imu.csv", "timestamp_us,gyro_dt_us,accel_dt_us,fx,fy,fz,wx,wy,wz,clipping,accel_id,gyro_id,accel_calibration,gyro_calibration", IMU_ROWS)
    write_csv(OUTPUT / "gnss.csv", "timestamp_us,device_id,latitude_deg,longitude_deg,altitude_msl_m,altitude_ellipsoid_m,vn,ve,vd,eph,epv,sacc,hdop,vdop,fix_type,satellites,velocity_valid,spoofed", GPS_ROWS)
    write_csv(OUTPUT / "magnetometer.csv", "timestamp_us,device_id,calibration_count,mx,my,mz", MAG_ROWS)
    write_csv(OUTPUT / "land.csv", "timestamp_us,landed,at_rest", LAND_ROWS)
    if ORIGIN is not None:
        write_csv(OUTPUT / "origin.csv", "latitude_deg,longitude_deg,height_ellipsoid_m", [ORIGIN])
    write_csv(OUTPUT / "navigation.csv", NAVIGATION_HEADER, NAVIGATION)
    return {"start_timestamp_us": START, "end_timestamp_us": END, "imu_rows": len(IMU_ROWS),
            "gnss_rows": len(GPS_ROWS), "magnetometer_rows": len(MAG_ROWS),
            "initialization": "InsSession IMU tilt initialization, gravity refinement, magnetic heading and GNSS qualification",
            "clipping_unavailable_rows": sum(ROW[9] == -1 for ROW in IMU_ROWS),
            "rest_context": "original land message timestamps; missing at_rest never asserts rest",
            "source_identity": "sensor_selection where available; otherwise fixed IDs bind log topic streams, not hardware continuity",
            "calibration_identity": "logged counters when present, zero otherwise; unlogged changes cannot be established",
            "gnss_spoofing": "MULTIPLE(3) maps to spoofed; absent report does not assert detection capability",
            "timing": "logged message timestamps; arrival on following actual IMU tick; 200 ms requested fusion lag; physical sensor delay compensation disabled",
            "display_frame": "fixed WGS84 tangent NED at first valid GNSS point, used only for plots",
            "height": "actual ellipsoid and MSL fields; session owns vertical reference and datum",
            "magnetic_declination_source": DECLINATION_SOURCE,
            "magnetic_model": (None if MAGNETIC_MODEL is None else {
                "executable": str(MAGNETIC_MODEL.resolve()),
                "sha256": hashlib.sha256(MAGNETIC_MODEL.read_bytes()).hexdigest(),
                "query": "runtime-requested location and origin basis; no precomputed estimator reference"}),
            "coordinate_conversion": "formal_eskf::geodesy through replay_ins --geodesy (plot-only preparation)",
            "settings": SETTINGS,
            "bias_limit_sources": LIMIT_SOURCES}


def run_replay(LOG_FILE, BINARY, COMPARE_MAG=False, MAGNETIC_MODEL=None):
    BINARY = executable(BINARY)
    if MAGNETIC_MODEL is not None:
        MAGNETIC_MODEL = executable(MAGNETIC_MODEL)
    if not LOG_FILE.is_file():
        raise ValueError(f"ULog file not found: {LOG_FILE}")
    if LOG_FILE.stem in ("", ".", ".."):
        raise ValueError("ULog filename must have a nonempty name before the extension")
    OUTPUT = Path("output") / LOG_FILE.stem
    LOG_FILE = LOG_FILE.resolve()
    INPUT_PATHS = [LOG_FILE, BINARY] + ([MAGNETIC_MODEL] if MAGNETIC_MODEL is not None else [])
    check_output(OUTPUT, INPUT_PATHS)
    from pyulog import ULog
    from tqdm import tqdm as TQDM
    with TQDM(total=3 + int(COMPARE_MAG), desc="Read ULog", dynamic_ncols=True, bar_format=PROGRESS_FORMAT) as PROGRESS:
        ULOG = ULog(str(LOG_FILE), message_name_filter_list=[
            "sensor_combined", "vehicle_gps_position", "vehicle_magnetometer", "vehicle_land_detected", "sensor_selection"])
        PROGRESS.update(1)
        PROGRESS.set_description_str("Prepare CSV / metadata")
        if replace_output(OUTPUT):
            PROGRESS.write(f"Removed previous replay output: {OUTPUT}")
        META = prepare(ULOG, OUTPUT / "input", BINARY, MAGNETIC_MODEL)
        with LOG_FILE.open("rb") as STREAM:
            HASH = hashlib.sha256()
            for CHUNK in iter(lambda: STREAM.read(1048576), b""):
                HASH.update(CHUNK)
        META.update(source=str(LOG_FILE.resolve()), sha256=HASH.hexdigest(),
                    px4_version=ULOG.msg_info_dict.get("ver_sw"), hardware=ULOG.msg_info_dict.get("ver_hw"),
                    logged_dropout_count=len(ULOG.dropouts))
        write_metadata(OUTPUT / "input/source.json", META)
        PROGRESS.write(f"Prepared {META['imu_rows']} IMU rows, {META['gnss_rows']} GNSS events, "
                       f"{META['magnetometer_rows']} magnetometer events")
        PROGRESS.update(1)
        PROGRESS.set_description_str("Run C++ replay")
        RESULT = run_estimator(BINARY, OUTPUT / "input", OUTPUT / "result", SESSION=True, MAGNETIC_MODEL=MAGNETIC_MODEL)
        PROGRESS.update(1)
        if COMPARE_MAG:
            PROGRESS.set_description_str("Run magnetic loss after alignment")
            BASELINE = run_estimator(BINARY, OUTPUT / "input", OUTPUT / "result_no_mag", MAGNETIC=False, SESSION=True, MAGNETIC_MODEL=MAGNETIC_MODEL)
            PROGRESS.update(1)
        PROGRESS.set_description_str("Replay complete")
    print(RESULT, end="", flush=True)
    if COMPARE_MAG:
        print("Magnetic input stopped after initial heading alignment:", flush=True)
        print(BASELINE, end="", flush=True)
    print(f"Output: {OUTPUT}", flush=True)
    return OUTPUT


def main():
    PARSER = argparse.ArgumentParser(description=__doc__,
                                     epilog="Results replace output/<ulog filename without .ulg>/ on each run.")
    PARSER.add_argument("ulog", type=Path)
    PARSER.add_argument("--binary", type=Path, default=Path("build/demo/replay_ins"),
                        help="prebuilt C++ executable (default: build/demo/replay_ins)")
    PARSER.add_argument("--plot", action="store_true", help="save PNG plots and display them with matplotlib")
    PARSER.add_argument("--compare-mag", action="store_true",
                        help="also stop magnetic input after heading alignment; --plot compares the resulting lifecycle")
    PARSER.add_argument("--magnetic-model", type=Path,
                        help="host px4_magnetic_model executable for field checks; EKF2_DECL_TYPE selects heading use")
    ARGS = PARSER.parse_args()
    try:
        if ARGS.plot:
            # Check the optional dependency before creating the output directory.
            import matplotlib.pyplot
        OUTPUT = run_replay(ARGS.ulog, ARGS.binary, ARGS.compare_mag, ARGS.magnetic_model)
        if ARGS.plot:
            plot_replay(OUTPUT)
    except subprocess.CalledProcessError as ERROR:
        PARSER.exit(1, f"error: C++ replay failed (exit {ERROR.returncode})\n")
    except subprocess.TimeoutExpired:
        PARSER.exit(1, "error: magnetic-model provider timed out\n")
    except ImportError as ERROR:
        PARSER.exit(2, f"error: {ERROR}; install dependencies with: "
                      "python3 -m pip install -r requirements.txt\n")
    except (OSError, KeyError, ValueError) as ERROR:
        PARSER.exit(2, f"error: {ERROR}\n")


if __name__ == "__main__":
    main()
