#!/usr/bin/env python3
"""Standard-session host binding: real entrypoints and independent stationary inputs."""

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

from test_replay_geodesy import COMMON, PX4, read_csv, ulog_fixture


def stationary_log(SECONDS=34, CLIPPING=None, GPS=True, MAGNETIC=True):
    ULOG = ulog_fixture()
    ULOG.initial_parameters.update(EKF2_DECL_TYPE=2, EKF2_MAG_DECL=0.01, EKF2_GPS_CHECK=29)
    IMU_TIMES = list(range(1000000, 1000000 + SECONDS * 1000000 + 1, 4000))
    IMU = {"timestamp": IMU_TIMES, "gyro_integral_dt": [4000] * len(IMU_TIMES),
           "accelerometer_integral_dt": [4000] * len(IMU_TIMES),
           "accelerometer_timestamp_relative": [0] * len(IMU_TIMES)}
    if CLIPPING is not None:
        IMU["accelerometer_clipping"] = [CLIPPING] * len(IMU_TIMES)
    for AXIS in range(3):
        IMU[f"accelerometer_m_s2[{AXIS}]"] = [-9.80665 if AXIS == 2 else 0] * len(IMU_TIMES)
        IMU[f"gyro_rad[{AXIS}]"] = [0] * len(IMU_TIMES)
    TOPICS = [("sensor_combined", IMU)]
    if GPS:
        TIMES = IMU_TIMES[::50]  # 5 Hz GNSS on a stationary, known location.
        VALUES = dict(lat=250000000, lon=1210000000, alt_ellipsoid=120000, alt=80000,
                      vel_n_m_s=0, vel_e_m_s=0, vel_d_m_s=0, eph=0.7, epv=1,
                      s_variance_m_s=0.2, hdop=0.7, vdop=0.9, fix_type=3,
                      satellites_used=12, vel_ned_valid=1)
        TOPICS.append(("vehicle_gps_position", {"timestamp": TIMES, **{
            KEY: [VALUE] * len(TIMES) for KEY, VALUE in VALUES.items()}}))
    if MAGNETIC:
        TIMES = IMU_TIMES[::5]
        TOPICS.append(("vehicle_magnetometer", {"timestamp": TIMES, **{
            f"magnetometer_ga[{AXIS}]": [VALUE] * len(TIMES)
            for AXIS, VALUE in enumerate((0.2, 0, 0.4))}}))
    # No rest flag: the adapter must not infer stationary status from landed.
    TIMES = IMU_TIMES[::250]
    TOPICS.append(("vehicle_land_detected", {"timestamp": TIMES, "landed": [1] * len(TIMES)}))
    ULOG.data_list = [SimpleNamespace(name=NAME, multi_id=0, data=DATA) for NAME, DATA in TOPICS]
    return ULOG


class ReplaySession(unittest.TestCase):
    def setUp(self):
        self.TEMP = tempfile.TemporaryDirectory()
        self.addCleanup(self.TEMP.cleanup)
        self.ROOT = Path(self.TEMP.name)

    def run_session(self, LOG, WITHDRAW=False, PROVIDER=None):
        INPUT = self.ROOT / "input"
        if not INPUT.exists():
            PX4.prepare(LOG, INPUT, BINARY, PROVIDER)
        OUTPUT = self.ROOT / ("withdrawn" if WITHDRAW else "result")
        COMMON.run_estimator(BINARY, INPUT, OUTPUT, MAGNETIC=not WITHDRAW,
                             SESSION=True, MAGNETIC_MODEL=PROVIDER)
        return json.loads((OUTPUT / "summary.json").read_text()), read_csv(OUTPUT / "status.csv"), read_csv(OUTPUT / "state.csv")

    def test_standard_startup_and_magnetic_loss(self):
        LOG = stationary_log()
        SUMMARY, STATUS, STATE = self.run_session(LOG)
        self.assertEqual(SUMMARY["initialized_at_us"], 1000000)
        self.assertGreater(SUMMARY["tilt_aligned_at_us"], SUMMARY["initialized_at_us"])
        self.assertGreaterEqual(SUMMARY["heading_available_at_us"], SUMMARY["tilt_aligned_at_us"])
        self.assertGreaterEqual(SUMMARY["navigation_available_at_us"], 11000000)  # Real 10 s qualification.
        self.assertTrue(SUMMARY["final_navigation_available"])
        self.assertEqual(SUMMARY["overflows"], 0)
        self.assertEqual(SUMMARY["imu_rejections"], 0)
        for KEY in ("gravity_corrections", "magnetic_corrections", "gnss_corrections"):
            self.assertGreater(SUMMARY[KEY], 0)
        self.assertEqual(len(STATE), len(PX4.topic(LOG, "sensor_combined")["timestamp"]))
        self.assertTrue(math.isnan(STATE[0]["q0"]))
        self.assertTrue(math.isnan(STATE[0]["pn"]))
        self.assertTrue(all(ROW["fault"] == 0 and ROW["clipping_available"] == 0 for ROW in STATUS))
        self.assertTrue(all(ROW["gnss_raw_time_us"] >= ROW["gnss_selection_time_us"] for ROW in STATUS))
        self.assertTrue(all(ROW["covariance_time_us"] <= ROW["timestamp_us"] for ROW in STATUS))
        for KEY in ("pn", "pe", "pd", "vn", "ve", "vd"):
            self.assertLess(abs(STATE[-1][KEY]), 1e-5, KEY)
        self.assertGreater(STATUS[-1]["lag_us"], 0)
        LOSS, LOSS_STATUS, LOSS_STATE = self.run_session(LOG, WITHDRAW=True)
        self.assertEqual(LOSS["heading_available_at_us"], SUMMARY["heading_available_at_us"])
        self.assertEqual(LOSS["magnetic_withdrawal_at_us"], SUMMARY["heading_available_at_us"])
        self.assertFalse(LOSS["final_heading_available"])
        self.assertGreater(LOSS_STATUS[-1]["heading_reasons"], 0)
        self.assertLess(LOSS["magnetic_corrections"], SUMMARY["magnetic_corrections"])
        # Identical physical initialization and calculations until input withdrawal.
        for A, B in zip(STATE, LOSS_STATE):
            if A["timestamp_us"] > LOSS["magnetic_withdrawal_at_us"]:
                break
            for KEY in A:
                self.assertTrue(A[KEY] == B[KEY] or (math.isnan(A[KEY]) and math.isnan(B[KEY])), KEY)

    def test_clipped_input_does_not_initialize(self):
        SUMMARY, STATUS, STATE = self.run_session(stationary_log(SECONDS=2, CLIPPING=7))
        self.assertEqual(SUMMARY["initialized_at_us"], 0)
        self.assertEqual(SUMMARY["predictions"], 0)
        self.assertFalse(SUMMARY["final_attitude_available"])
        self.assertTrue(all(ROW["fault"] == 0 for ROW in STATUS))
        self.assertTrue(all(math.isnan(ROW["q0"]) for ROW in STATE))

    def test_attitude_only_without_navigation_sensors(self):
        SUMMARY, STATUS, _ = self.run_session(stationary_log(GPS=False, MAGNETIC=False))
        self.assertTrue(SUMMARY["final_attitude_available"])
        self.assertFalse(SUMMARY["final_heading_available"])
        self.assertFalse(SUMMARY["final_navigation_available"])
        self.assertEqual(SUMMARY["gnss_corrections"], 0)
        self.assertEqual(SUMMARY["magnetic_corrections"], 0)
        self.assertTrue(all(ROW["global_position"] == 0 for ROW in STATUS))
        self.assertFalse((self.ROOT / "input/origin.csv").exists())

    def test_bad_gnss_quality_does_not_enable_navigation(self):
        LOG = stationary_log()
        GPS = PX4.topic(LOG, "vehicle_gps_position")
        GPS["fix_type"] = [2] * len(GPS["timestamp"])
        SUMMARY, STATUS, _ = self.run_session(LOG)
        self.assertTrue(SUMMARY["final_heading_available"])
        self.assertFalse(SUMMARY["final_navigation_available"])
        self.assertTrue(all(ROW["gnss_qualified"] == 0 for ROW in STATUS))
        self.assertEqual(SUMMARY["gnss_corrections"], 0)

    def test_original_context_epochs_and_calibration_identity(self):
        LOG = stationary_log(SECONDS=2)
        IMU = PX4.topic(LOG, "sensor_combined")
        IMU["accel_calibration_count"] = [0] * 250 + [1] * (len(IMU["timestamp"]) - 250)
        PX4.prepare(LOG, self.ROOT / "input", BINARY)
        LAND = read_csv(self.ROOT / "input/land.csv")
        self.assertEqual([ROW["timestamp_us"] for ROW in LAND], [1000000, 2000000, 3000000])
        self.assertTrue(all(ROW["at_rest"] == -1 for ROW in LAND))
        ROWS = read_csv(self.ROOT / "input/imu.csv")
        self.assertEqual(ROWS[249]["accel_calibration"], 0)
        self.assertEqual(ROWS[250]["accel_calibration"], 1)

    def test_missing_requested_spoofing_check_is_rejected(self):
        LOG = stationary_log(SECONDS=2)
        LOG.initial_parameters["EKF2_GPS_CHECK"] |= 512
        with self.assertRaisesRegex(ValueError, "spoofing_state"):
            PX4.prepare(LOG, self.ROOT / "input", BINARY)
        self.assertFalse((self.ROOT / "input").exists())

    def test_unordered_input_fails_before_output(self):
        PX4.prepare(stationary_log(SECONDS=2), self.ROOT / "input", BINARY)
        PATH = self.ROOT / "input/gnss.csv"
        LINES = PATH.read_text().splitlines()
        LINES[1], LINES[2] = LINES[2], LINES[1]
        PATH.write_text("\n".join(LINES) + "\n")
        RESULT = subprocess.run([str(BINARY), str(PATH.parent), str(self.ROOT / "result"), "--session"],
                                capture_output=True, text=True)
        self.assertNotEqual(RESULT.returncode, 0)
        self.assertIn("unordered GNSS input", RESULT.stderr)
        self.assertFalse((self.ROOT / "result").exists())

    def test_imu_history_fault_is_not_success(self):
        LOG = stationary_log(SECONDS=2)
        # A one-second hole is not replaced by held/fabricated IMU measurements.
        IMU = PX4.topic(LOG, "sensor_combined")
        IMU["timestamp"][-1] += 1000000
        PX4.prepare(LOG, self.ROOT / "input", BINARY)
        with self.assertRaises(subprocess.CalledProcessError):
            COMMON.run_estimator(BINARY, self.ROOT / "input", self.ROOT / "result", SESSION=True)
        STATUS = read_csv(self.ROOT / "result/status.csv")
        self.assertEqual(STATUS[-1]["fault"], 1)
        self.assertEqual(STATUS[-1]["attitude"], 0)
        self.assertEqual((self.ROOT / "result/summary.json").read_text(), "")

    def test_model_requests_use_real_session_locations(self):
        LOG = stationary_log()
        LOG.initial_parameters["EKF2_DECL_TYPE"] = 1
        PROVIDER = self.ROOT / "model"
        REQUESTS = self.ROOT / "requests"
        PROVIDER.write_text(f"#!/bin/sh\nread -r LAT LON\nprintf '%s %s\\n' \"$LAT\" \"$LON\" >> '{REQUESTS}'\nprintf '0 63.43494882292201 44.721359549995796\\n'\n")
        PROVIDER.chmod(0o755)
        SUMMARY, _, _ = self.run_session(LOG, PROVIDER=PROVIDER)
        self.assertTrue(SUMMARY["final_navigation_available"])
        QUERIES = [[float(VALUE) for VALUE in LINE.split()] for LINE in REQUESTS.read_text().splitlines()]
        self.assertGreater(len(QUERIES), 0)
        for LAT, LON in QUERIES:
            self.assertAlmostEqual(LAT, 25, places=5)
            self.assertAlmostEqual(LON, 121, places=5)


if __name__ == "__main__":
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument("--binary", type=Path, required=True)
    ARGS, REMAINING = PARSER.parse_known_args()
    BINARY = ARGS.binary.resolve()
    unittest.main(argv=[sys.argv[0], *REMAINING])
