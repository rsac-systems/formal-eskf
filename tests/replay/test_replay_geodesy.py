#!/usr/bin/env python3
"""Exercise both replay adapters against the real C++ coordinate bridge."""

import argparse
import csv
import math
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

REPO_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_DIR / "examples" / "replay"))
import runner as COMMON
import kitti as KITTI
import px4 as PX4


def ulog_fixture():
    """Distinct ellipsoid/MSL heights, a bad fix, and a nonfinite raw velocity."""
    TIMES = list(range(1000000, 1300001, 10000))
    IMU = {"timestamp": TIMES, "gyro_integral_dt": [10000] * len(TIMES),
           "accelerometer_integral_dt": [10000] * len(TIMES),
           "accelerometer_timestamp_relative": [0] * len(TIMES)}
    MAG = {"timestamp": [1100000, 1260000]}
    for AXIS in range(3):
        IMU[f"accelerometer_m_s2[{AXIS}]"] = [(-9.80665 if AXIS == 2 else 0)] * len(TIMES)
        IMU[f"gyro_rad[{AXIS}]"] = [0] * len(TIMES)
        MAG[f"magnetometer_ga[{AXIS}]"] = [(0.3 if AXIS == 0 else 0)] * 2
    GPS = {"timestamp": [1190000, 1250000, 1270000], "lat": [0, 0, 0], "lon": [0, 900000000, 1800000000],
           "alt_ellipsoid": [100000, 100000, 100000], "alt": [20000, 10000, 5000],
           "vel_n_m_s": [2, 2, math.nan], "vel_e_m_s": [3, 3, 3], "vel_d_m_s": [4, 4, 4],
           "eph": [1, 1, 1], "epv": [1, 1, 1], "s_variance_m_s": [0.3, 0.3, 0.3],
           "fix_type": [3, 3, 2], "vel_ned_valid": [1, 1, 1], "satellites_used": [10, 10, 10],
           "hdop": [1, 1, 1], "vdop": [1, 1, 1]}
    PARAMETERS = {
        "EKF2_MAG_NOISE": 0.05, "EKF2_MAG_GATE": 3,
        "EKF2_ABL_ACCLIM": 25, "EKF2_ABL_GYRLIM": 3, "EKF2_ABL_TAU": 0.5, "EKF2_IMU_CTRL": 3,
        "EKF2_GPS_P_GATE": 5, "EKF2_GPS_V_GATE": 5, "EKF2_GPS_P_NOISE": 0.5,
        "EKF2_GPS_V_NOISE": 0.3, "EKF2_NOAID_NOISE": 10,
        "EKF2_GPS_CHECK": 0, "EKF2_REQ_NSATS": 6, "EKF2_REQ_PDOP": 3,
        "EKF2_REQ_EPH": 5, "EKF2_REQ_EPV": 8, "EKF2_REQ_SACC": 1,
        "EKF2_REQ_HDRIFT": 0.3, "EKF2_REQ_VDRIFT": 0.5,
    }
    PARAMETERS.update({f"EKF2_{SENSOR}_POS_{AXIS}": 0 for SENSOR in ("GPS", "IMU") for AXIS in "XYZ"})
    return SimpleNamespace(initial_parameters=PARAMETERS, changed_parameters=[], data_list=[
        SimpleNamespace(name=NAME, multi_id=0, data=DATA) for NAME, DATA in (
            ("sensor_combined", IMU), ("vehicle_gps_position", GPS), ("vehicle_magnetometer", MAG))])


def read_csv(PATH):
    with PATH.open() as STREAM:
        return [{KEY: float(VALUE) for KEY, VALUE in ROW.items()} for ROW in csv.DictReader(STREAM)]


class ReplayGeodesy(unittest.TestCase):
    def test_axes_and_transport(self):
        # At the equator: N = z, E = y, D = a - x. At longitude 90,
        # sample North stays North; sample East becomes Down; Down becomes West.
        PROJECTED = COMMON.project_coordinates(BINARY, [0, 0, 0], [[0, 0, 0], [0, 90, 100]])
        self.assertEqual(PROJECTED[0][0], [0, 0, 0])
        self.assertEqual(PROJECTED[1][0], [0, 6378237, 6378137])
        VELOCITY = [COMMON.dot(ROW, [2, 3, 4]) for ROW in PROJECTED[1][1]]
        self.assertEqual(VELOCITY, [2, -4, 3])

    def test_explicit_origin_events(self):
        with tempfile.TemporaryDirectory() as DIRECTORY:
            ROOT = Path(DIRECTORY)
            INPUT = ROOT / "input"
            INPUT.mkdir()
            COMMON.write_csv(INPUT / "initial.csv", "timestamp_us,q0,q1,q2,q3,vn,ve,vd,eph,epv,speed_accuracy",
                             [[1000000, 1, 0, 0, 0, 100, 0, 0, 1, 1, 0.3]])
            COMMON.write_csv(INPUT / "origin.csv", "latitude_deg,longitude_deg,height_ellipsoid_m", [[25, 121, 120]])
            COMMON.write_csv(INPUT / "imu.csv", "timestamp_us,integral_dt_us,fx,fy,fz,wx,wy,wz",
                             [[1000000 + I * 10000, 10000, 0, 0, -9.80665, 0, 0, 0] for I in range(31)])
            COMMON.write_csv(INPUT / "navigation.csv", "timestamp_us,pn,pe,pd,vn,ve,vd,eph,epv,speed_accuracy",
                             [[1000000 + I * 10000, I, 0, 0, 100, 0, 0, 1, 1, 0.3] for I in range(5, 31, 5)])
            BASE = ROOT / "baseline"
            RESULT = subprocess.run([str(BINARY), str(INPUT), str(BASE)], text=True, capture_output=True)
            self.assertEqual(RESULT.returncode, 0, RESULT.stderr)
            COMMON.write_csv(INPUT / "origin_requests.csv", "timestamp_us", [[1050000], [1100000], [1200000]])
            MOVED = ROOT / "moved"
            RESULT = subprocess.run([str(BINARY), str(INPUT), str(MOVED)], text=True, capture_output=True)
            self.assertEqual(RESULT.returncode, 0, RESULT.stderr)
            self.assertEqual(RESULT.stdout.count("Origin migration:"), 3)
            BEFORE, AFTER = read_csv(BASE / "state.csv"), read_csv(MOVED / "state.csv")
            self.assertEqual(len(BEFORE), len(AFTER))
            for A, B in zip(BEFORE, AFTER):
                self.assertEqual(A["timestamp_us"], B["timestamp_us"])
                for KEY in ("pn", "pe", "pd", "vn", "ve", "vd"):
                    # Post-event gravity is refreshed; trajectories need not be bitwise equal.
                    self.assertLess(abs(A[KEY] - B[KEY]), 1e-4, (KEY, A, B))
            # The first event itself must preserve the physical estimate more tightly.
            for KEY in ("pn", "pe", "pd", "vn", "ve", "vd"):
                self.assertLess(abs(BEFORE[5][KEY] - AFTER[5][KEY]), 1e-7)
            self.assertEqual((BASE / "navigation_updates.csv").read_text(),
                             (MOVED / "navigation_updates.csv").read_text())
            (INPUT / "origin.csv").unlink()
            RESULT = subprocess.run([str(BINARY), str(INPUT), str(ROOT / "invalid")], text=True, capture_output=True)
            self.assertNotEqual(RESULT.returncode, 0)
            self.assertIn("origin requests require origin.csv", RESULT.stderr)

    def test_invalid_batch(self):
        for TEXT in ("", "0,0\n", "0,0,0,1\n", "nan,0,0\n", "0,0,100001\n",
                     "0,0,0\n91,0,0\n", "0,0,0\n0,181,0\n", "0,0,0\n0,0,-12001\n"):
            with self.subTest(INPUT=TEXT):
                RESULT = subprocess.run([str(BINARY), "--geodesy"], input=TEXT, text=True, capture_output=True, check=False)
                self.assertNotEqual(RESULT.returncode, 0)
                self.assertIn("error:", RESULT.stderr)
        # Even if an earlier row was emitted, the Python bridge must reject the batch.
        with self.assertRaisesRegex(ValueError, "coordinate conversion failed"):
            COMMON.project_coordinates(BINARY, [0, 0, 0], [[0, 1, 0], [0, 0, 100001]])

    def test_kitti_preparation(self):
        SEED = dict.fromkeys(KITTI.OXTS_FIELDS, 0.0)
        SEED.update(lat=0, lon=0, alt=100, yaw=math.pi / 2, vn=2, ve=3, vu=-4,
                    az=9.80665, navstat=4, posmode=3, velmode=2)
        MOVED = {**SEED, "lon": 90}
        UNAVAILABLE = {**SEED, "lon": 120, "posmode": 0}
        SEGMENTS, NAVIGATION, META = KITTI.prepare(
            [(1000000, SEED), (1010000, SEED), (1020000, MOVED), (1030000, UNAVAILABLE)], 100, BINARY)
        self.assertEqual(SEGMENTS[0]["prior"][12:15], [2, 3, 4])
        self.assertEqual(len(NAVIGATION), 2)  # Bad quality remains a runtime decision.
        self.assertEqual(NAVIGATION[0][:7], [1020000, 0, 6378237, 6378237, 2, -4, 3])
        self.assertEqual(SEGMENTS[0]["gnss"][1][14], 2)
        self.assertIn("formal_eskf::geodesy", META["coordinate_conversion"])

    def test_px4_preparation_preserves_quality_domain(self):
        with tempfile.TemporaryDirectory() as DIRECTORY:
            OUTPUT = Path(DIRECTORY) / "input"
            META = PX4.prepare(ulog_fixture(), OUTPUT, BINARY)
            NAVIGATION = read_csv(OUTPUT / "navigation.csv")
            GNSS = read_csv(OUTPUT / "gnss.csv")
            IMU = read_csv(OUTPUT / "imu.csv")
            self.assertFalse((OUTPUT / "initial.csv").exists())
            self.assertEqual(len(IMU), 31)  # No Python alignment window is discarded.
            self.assertTrue(all(ROW["clipping"] == -1 for ROW in IMU))
            self.assertEqual([ROW["timestamp_us"] for ROW in GNSS], [1190000, 1250000, 1270000])
            self.assertEqual([ROW["timestamp_us"] for ROW in NAVIGATION], [1190000, 1250000])
            self.assertEqual([GNSS[1][KEY] for KEY in ("vn", "ve", "vd")], [2, 3, 4])
            self.assertEqual(GNSS[1]["altitude_ellipsoid_m"], 100)
            self.assertEqual(GNSS[1]["altitude_msl_m"], 10)
            self.assertEqual(GNSS[2]["fix_type"], 2)
            self.assertTrue(math.isnan(GNSS[2]["vn"]))
            self.assertEqual([NAVIGATION[1][KEY] for KEY in ("pn", "pe", "pd", "vn", "ve", "vd")],
                             [0, 6378237, 6378237, 2, -4, 3])
            self.assertEqual(META["gnss_rows"], 3)

    def test_px4_invalid_height_reaches_runtime(self):
        ULOG = ulog_fixture()
        PX4.topic(ULOG, "vehicle_gps_position")["alt_ellipsoid"][1] = 100001000
        with tempfile.TemporaryDirectory() as DIRECTORY:
            OUTPUT = Path(DIRECTORY) / "input"
            PX4.prepare(ULOG, OUTPUT, BINARY)
            # The plot is not a receiver-quality gate. Preserve the original input
            # for InsSession's admission/quality policy; omit it from plot conversion.
            self.assertEqual(read_csv(OUTPUT / "gnss.csv")[1]["altitude_ellipsoid_m"], 100001)
            self.assertEqual(len(read_csv(OUTPUT / "navigation.csv")), 1)

    def test_magnetic_vector_uses_same_origin(self):
        # Controlled external model values test the adapter, not WMM accuracy.
        DECLINATION = math.degrees(math.atan2(3, 2))
        INCLINATION = math.degrees(math.atan2(4, math.hypot(2, 3)))
        STRENGTH = math.sqrt(29)
        with tempfile.TemporaryDirectory() as DIRECTORY:
            MODEL = Path(DIRECTORY) / "model"
            MODEL.write_text(f"#!/bin/sh\nwhile read -r LAT LON; do printf '%s\\n' '{DECLINATION} {INCLINATION} {STRENGTH}'; done\n")
            MODEL.chmod(0o755)
            FIELD = COMMON.magnetic_model(BINARY, MODEL, [0, 90, 0, 0])
            OUTPUT = Path(DIRECTORY) / "input"
            META = PX4.prepare(ulog_fixture(), OUTPUT, BINARY, MODEL)
            self.assertFalse((OUTPUT / "magnetic_model.csv").exists())
            self.assertEqual(META["settings"]["model_provider"], 1)
        self.assertAlmostEqual(FIELD[0], math.atan2(-4, 2), delta=1e-14)
        self.assertAlmostEqual(FIELD[1], math.atan2(3, math.hypot(2, 4)), delta=1e-14)
        self.assertAlmostEqual(FIELD[2], STRENGTH, delta=1e-14)


class ReplayExecution(unittest.TestCase):
    def test_source_overlap_preserves_existing_output(self):
        with tempfile.TemporaryDirectory() as DIRECTORY:
            OUTPUT = Path(DIRECTORY) / "output"
            OUTPUT.mkdir()
            SOURCE = OUTPUT / "source.ulg"
            SOURCE.write_bytes(b"source")
            with self.assertRaises(ValueError):
                COMMON.check_output(OUTPUT, [SOURCE, BINARY])
            self.assertEqual(SOURCE.read_bytes(), b"source")

    def test_output_inside_dataset_is_rejected(self):
        with tempfile.TemporaryDirectory() as DIRECTORY:
            SOURCE = Path(DIRECTORY)
            with self.assertRaises(ValueError):
                COMMON.check_output(SOURCE / "output", [SOURCE, BINARY])
            self.assertFalse((SOURCE / "output").exists())

    def test_symlink_output_is_rejected(self):
        with tempfile.TemporaryDirectory() as DIRECTORY:
            ROOT = Path(DIRECTORY)
            TARGET = ROOT / "target"
            TARGET.mkdir()
            OUTPUT = ROOT / "output"
            OUTPUT.symlink_to(TARGET, target_is_directory=True)
            with self.assertRaises(ValueError):
                COMMON.check_output(OUTPUT, [BINARY])
            self.assertTrue(TARGET.is_dir())

    def test_replacement_is_limited_to_run_directory(self):
        with tempfile.TemporaryDirectory() as DIRECTORY:
            ROOT = Path(DIRECTORY)
            OUTPUT = ROOT / "output"
            (ROOT / "keep").write_text("keep")
            self.assertFalse(COMMON.replace_output(OUTPUT))
            (OUTPUT / "old.csv").write_text("old")
            self.assertTrue(COMMON.replace_output(OUTPUT))
            self.assertEqual(list(OUTPUT.iterdir()), [])
            self.assertEqual((ROOT / "keep").read_text(), "keep")

    def test_estimator_failure_is_not_reported_as_success(self):
        with tempfile.TemporaryDirectory() as DIRECTORY:
            ROOT = Path(DIRECTORY)
            INPUT = ROOT / "input"
            INPUT.mkdir()
            with self.assertRaises(subprocess.CalledProcessError) as FAILURE:
                COMMON.run_estimator(BINARY, INPUT, ROOT / "result")
            self.assertEqual(FAILURE.exception.returncode, 1)
            self.assertFalse((ROOT / "result").exists())


if __name__ == "__main__":
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument("--binary", type=Path, required=True)
    ARGS, REMAINING = PARSER.parse_known_args()
    BINARY = ARGS.binary.resolve()
    unittest.main(argv=[sys.argv[0], *REMAINING])
