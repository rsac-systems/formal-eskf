#!/usr/bin/env python3
"""Standard-session host binding: real entrypoints and independent stationary inputs."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

from test_replay_geodesy import COMMON, PX4, KITTI, read_csv, ulog_fixture


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
                             MAGNETIC_MODEL=PROVIDER)
        return json.loads((OUTPUT / "summary.json").read_text()), read_csv(OUTPUT / "status.csv"), read_csv(OUTPUT / "state.csv")

    def test_standard_startup_and_magnetic_loss(self):
        LOG = stationary_log()
        SUMMARY, STATUS, STATE = self.run_session(LOG)
        self.assertGreater(SUMMARY["configuration_id"], 0)
        self.assertGreater(SUMMARY["configuration_revision"], 0)
        self.assertEqual(SUMMARY["settings_sha256"],
                         hashlib.sha256((self.ROOT / "input/settings.csv").read_bytes()).hexdigest())
        self.assertEqual(SUMMARY["executable_sha256"], hashlib.sha256(Path(BINARY).read_bytes()).hexdigest())
        self.assertEqual(SUMMARY["position_velocity_frame"], "fixed_plot_ned")
        self.assertEqual(SUMMARY["quaternion_frame"], "geographic_ned_or_unattached_local")
        self.assertEqual(SUMMARY["output_point"], "configured_body_origin")
        self.assertEqual(SUMMARY["covariance_source"], "delayed_imu_point")
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
            COMMON.run_estimator(BINARY, self.ROOT / "input", self.ROOT / "result")
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


    def test_gnss_outage_and_recovery(self):
        LOG = stationary_log(SECONDS=90)
        GPS = PX4.topic(LOG, "vehicle_gps_position")
        KEEP = [I for I, TIME in enumerate(GPS["timestamp"]) if TIME < 50000000 or TIME >= 65000000]
        for KEY in GPS:
            GPS[KEY] = [GPS[KEY][I] for I in KEEP]
        SUMMARY, STATUS, _ = self.run_session(LOG)
        self.assertTrue(any(ROW["global_position"] == 1 and ROW["timestamp_us"] < 50000000 for ROW in STATUS))
        self.assertTrue(all(ROW["global_position"] == 0 for ROW in STATUS if 60000000 <= ROW["timestamp_us"] <= 65000000))
        # Existing session retains historical initial qualification; recovery
        # must still wait for current quality/health, unlike a new KITTI session.
        self.assertTrue(all(ROW["global_position"] == 0 for ROW in STATUS if 65000000 <= ROW["timestamp_us"] <= 66000000))
        self.assertTrue(any(ROW["gnss_qualified"] == 1 and ROW["global_position"] == 0 for ROW in STATUS))
        self.assertTrue(SUMMARY["final_navigation_available"])
        self.assertEqual(SUMMARY["overflows"], 0)

    def test_magnetic_calibration_change_reaches_session(self):
        LOG = stationary_log(SECONDS=65)
        MAG = PX4.topic(LOG, "vehicle_magnetometer")
        MAG["calibration_count"] = [int(T >= 50000000) for T in MAG["timestamp"]]
        SUMMARY, STATUS, _ = self.run_session(LOG)
        with (self.ROOT / "result/fusion.csv").open() as STREAM:
            ROWS = [ROW for ROW in csv.DictReader(STREAM) if ROW["group"] == "magnetic"]
        GENERATIONS = {int(ROW["source_generation"]) for ROW in ROWS}
        self.assertGreater(len(GENERATIONS), 1)
        self.assertTrue(all(ROW["fault"] == 0 for ROW in STATUS))
        self.assertTrue(SUMMARY["final_heading_available"])
        LAST = max(GENERATIONS)
        self.assertTrue(all(int(ROW["raw_time_us"]) >= 50000000 for ROW in ROWS if int(ROW["source_generation"]) == LAST))


class KittiSession(unittest.TestCase):
    def setUp(self):
        self.TEMP = tempfile.TemporaryDirectory()
        self.addCleanup(self.TEMP.cleanup)
        self.ROOT = Path(self.TEMP.name)

    @staticmethod
    def records(START=1000000, SECONDS=15):
        PACKET = dict.fromkeys(KITTI.OXTS_FIELDS, 0.)
        PACKET.update(lat=25, lon=121, alt=80, yaw=math.pi / 2, az=9.80665,
                      navstat=4, numsats=12, posmode=3, velmode=2,
                      pos_accuracy=1, vel_accuracy=0.3)
        return [(TIME, PACKET.copy()) for TIME in range(START, START + SECONDS * 1000000 + 1, 10000)]

    def test_prior_qualification_and_gap_restart(self):
        RECORDS = self.records(SECONDS=15) + self.records(START=20000000, SECONDS=15)
        SEGMENTS, _, META = KITTI.prepare(RECORDS, 10, BINARY)
        self.assertEqual(len(SEGMENTS), 2)
        self.assertEqual(META["synthetic_imu_rows"], 0)
        self.assertEqual(META["imu_gaps"], [[16000000, 20000000]])
        self.assertEqual(META["settings"]["gnss_health_time_us"], 10000000)
        self.assertEqual(META["settings"]["gnss_checks"] & 2, 0)  # Synthetic DOP never checked.
        for SEGMENT in SEGMENTS:
            START = SEGMENT["prior"][0]
            self.assertEqual(SEGMENT["imu"][0][0], START)
            self.assertTrue(all(ROW[0] > START for ROW in SEGMENT["gnss"]))
            self.assertTrue(all(ROW[9] == -1 for ROW in SEGMENT["imu"]))  # Clipping unavailable.
            ROOT = self.ROOT / "segments" / str(SEGMENT["segment"])
            KITTI.write_segment(ROOT / "input", SEGMENT, META["settings"], [25, 121, 80])
            COMMON.run_estimator(BINARY, ROOT / "input", ROOT / "result", MODE="kitti")
            SUMMARY = json.loads((ROOT / "result/summary.json").read_text())
            STATUS = read_csv(ROOT / "result/status.csv")
            self.assertEqual(SUMMARY["initialized_at_us"], START)
            self.assertGreaterEqual(SUMMARY["navigation_available_at_us"], START + 10000000)
            self.assertTrue(SUMMARY["final_navigation_available"])
            self.assertEqual(SUMMARY["magnetic_corrections"], 0)
            self.assertEqual(SUMMARY["imu_rejections"], 0)
            self.assertEqual(SUMMARY["overflows"], 0)
            self.assertTrue(any(ROW["heading"] == 0 and ROW["gnss_qualified"] == 0 for ROW in STATUS))
            self.assertEqual(STATUS[0]["gnss_qualified"], 0)
            self.assertEqual(STATUS[0]["lag_us"], 0)
        TOTALS = KITTI.combine_results(self.ROOT, SEGMENTS)
        STATUS = read_csv(self.ROOT / "result/status.csv")
        self.assertEqual({ROW["segment"] for ROW in STATUS}, {1, 2})
        self.assertFalse(any(16000000 < ROW["timestamp_us"] < 20010000 for ROW in STATUS))
        self.assertGreater(TOTALS["gnss_corrections"], 0)
        self.assertEqual(TOTALS["imu_admissions"], sum(len(S["imu"]) for S in SEGMENTS))
        # Two identical independent segments have the same lifetime counts.
        SUMMARY = json.loads((self.ROOT / "result/summary.json").read_text())
        for KEY in TOTALS:
            self.assertEqual(SUMMARY["segments"][0][KEY], SUMMARY["segments"][1][KEY])

    def test_short_segment_and_invalid_prior(self):
        SEGMENTS, _, META = KITTI.prepare(self.records(SECONDS=1), 10, BINARY)
        INPUT = self.ROOT / "input"
        KITTI.write_segment(INPUT, SEGMENTS[0], META["settings"], [25, 121, 80])
        COMMON.run_estimator(BINARY, INPUT, self.ROOT / "short", MODE="kitti")
        SUMMARY = json.loads((self.ROOT / "short/summary.json").read_text())
        self.assertFalse(SUMMARY["final_navigation_available"])
        self.assertEqual(SUMMARY["gnss_corrections"], 0)
        PATH = INPUT / "initial_navigation.csv"
        ROWS = PATH.read_text().splitlines()
        VALUES = ROWS[1].split(',')
        VALUES[0] = str(int(VALUES[0]) + 1)
        PATH.write_text(ROWS[0] + '\n' + ','.join(VALUES) + '\n')
        with self.assertRaises(subprocess.CalledProcessError):
            COMMON.run_estimator(BINARY, INPUT, self.ROOT / "bad", MODE="kitti")
        self.assertFalse((self.ROOT / "bad").exists())

    def test_standard_mode_does_not_use_kitti_prior(self):
        SEGMENTS, _, _ = KITTI.prepare(self.records(SECONDS=1), 10, BINARY)
        INPUT = self.ROOT / "input"
        PX4.prepare(stationary_log(SECONDS=1), INPUT, BINARY)
        COMMON.write_csv(INPUT / "initial_navigation.csv", KITTI.INITIAL_NAVIGATION_HEADER, [SEGMENTS[0]["prior"]])
        with self.assertRaises(subprocess.CalledProcessError):
            COMMON.run_estimator(BINARY, INPUT, self.ROOT / "result")
        self.assertFalse((self.ROOT / "result").exists())

    def test_no_gap_filling_and_prior_frame_conversion(self):
        RECORDS = self.records(SECONDS=1)
        # Frame sanity: body FLU -> FRD; identity orientation is east-facing KITTI yaw pi/2.
        RECORDS[1][1].update(ax=1, ay=2, az=3, wx=4, wy=5, wz=6, vn=2, ve=3, vu=4)
        SEGMENTS, _, _ = KITTI.prepare(RECORDS, 10, BINARY)
        self.assertEqual(SEGMENTS[0]["prior"][1:5], [1, 0, 0, 0])
        self.assertEqual(SEGMENTS[0]["prior"][12:15], [2, 3, -4])
        self.assertEqual(SEGMENTS[0]["imu"][1][3:9], [1, -2, -3, 4, -5, -6])
        # One isolated packet cannot supply a real initialization interval.
        SEGMENTS, _, META = KITTI.prepare([(1, RECORDS[0][1]), *RECORDS], 10, BINARY)
        self.assertEqual(META["skipped_segments_without_seed_interval"], [1])
        self.assertEqual(len(SEGMENTS), 1)
        # Bad navigation quality is passed onward; it is not silently discarded.
        RECORDS[-1][1]["posmode"] = 0
        SEGMENTS, _, _ = KITTI.prepare(RECORDS, 100, BINARY)
        self.assertEqual(SEGMENTS[0]["gnss"][-1][14], 2)
        self.assertEqual(SEGMENTS[0]["gnss"][-1][16], 0)


    def test_session_rebase_keeps_display_frame(self):
        RECORDS = self.records(SECONDS=70)
        SPEED, RADIUS = 20., 6378137. + 80.
        for TIME, PACKET in RECORDS:
            ELAPSED = (TIME - RECORDS[0][0]) * 1e-6
            PACKET.update(lat=0, lon=math.degrees(SPEED * ELAPSED / RADIUS),
                          yaw=0, vn=0, ve=SPEED, vu=0)
        SEGMENTS, _, META = KITTI.prepare(RECORDS, 10, BINARY)
        INPUT = self.ROOT / "input"
        ORIGIN = SEGMENTS[0]["prior"][8:11]
        KITTI.write_segment(INPUT, SEGMENTS[0], META["settings"], ORIGIN)
        COMMON.run_estimator(BINARY, INPUT, self.ROOT / "result", MODE="kitti")
        STATE = read_csv(self.ROOT / "result/state.csv")
        with (self.ROOT / "result/fusion.csv").open() as STREAM:
            EPOCHS = {int(ROW["frame_epoch"]) for ROW in csv.DictReader(STREAM) if ROW["group"] == "gnss_position"}
        self.assertGreater(len(EPOCHS), 1)  # Explicit prior starts at frame zero; migration advances it.
        FINITE = [ROW for ROW in STATE if math.isfinite(ROW["pe"])]
        self.assertGreater(FINITE[-1]["pe"], 1300)  # No reset to zero in the fixed display frame.
        # 20 m/s x 10 ms = 0.2 m; a 1 m per-tick bound leaves numeric/fusion
        # margin while detecting a kilometre-scale origin jump, not accuracy.
        self.assertLess(max(abs(B["pe"] - A["pe"]) for A, B in zip(FINITE, FINITE[1:])), 1.)


if __name__ == "__main__":
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument("--binary", type=Path, required=True)
    ARGS, REMAINING = PARSER.parse_known_args()
    BINARY = ARGS.binary.resolve()
    unittest.main(argv=[sys.argv[0], *REMAINING])
