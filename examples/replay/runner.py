"""Input preparation and process/output management for host replay entrypoints."""

import csv
import math
import json
import os
from pathlib import Path
import shutil
import subprocess


IMU_HEADER = "timestamp_us,integral_dt_us,fx,fy,fz,wx,wy,wz"
NAVIGATION_HEADER = "timestamp_us,pn,pe,pd,vn,ve,vd,eph,epv,speed_accuracy"
INITIAL_HEADER = "timestamp_us,q0,q1,q2,q3,vn,ve,vd,eph,epv,speed_accuracy"
PROGRESS_FORMAT = "{desc}: {bar} {n_fmt}/{total_fmt} stages [{elapsed}]"


def project_coordinates(BINARY, ORIGIN, POSITIONS):
    """Batch C++ geodesy: degrees/ellipsoidal metres -> position and sample-to-origin NED basis."""
    INPUT = "".join(",".join(format(VALUE, ".17g") for VALUE in ROW) + "\n" for ROW in [ORIGIN, *POSITIONS])
    RESULT = subprocess.run([str(BINARY), "--geodesy"], input=INPUT, text=True, capture_output=True, check=False)
    if RESULT.returncode:
        raise ValueError(f"C++ coordinate conversion failed: {RESULT.stderr.strip()}")
    ROWS = RESULT.stdout.splitlines()
    if len(ROWS) != len(POSITIONS):
        raise ValueError("coordinate conversion must return one row per sample")
    PROJECTED = []
    for ROW in ROWS:
        VALUES = [float(VALUE) for VALUE in ROW.split(",")]
        if len(VALUES) != 12 or not all(math.isfinite(VALUE) for VALUE in VALUES):
            raise ValueError("invalid C++ coordinate conversion result")
        PROJECTED.append((VALUES[:3], [VALUES[3:6], VALUES[6:9], VALUES[9:12]]))
    return PROJECTED


def dot(A, B):
    return sum(X * Y for X, Y in zip(A, B))


def write_csv(PATH, HEADER, ROWS):
    with PATH.open("x", newline="") as STREAM:
        WRITER = csv.writer(STREAM)
        WRITER.writerow(HEADER.split(","))
        WRITER.writerows(ROWS)


def executable(PATH):
    """Resolve and validate a host executable before preparing any output."""
    PATH = Path(PATH).resolve()
    if not PATH.is_file() or not os.access(PATH, os.X_OK):
        raise ValueError(f"replay executable not found or not executable: {PATH}; build it with CMake first")
    return PATH


def check_output(OUTPUT, INPUTS):
    """A replay may replace only its own output, never its source or executable."""
    if OUTPUT.is_symlink() or OUTPUT.parent.is_symlink():
        raise ValueError(f"replay output path must not be a symlink: {OUTPUT}")
    DESTINATION = OUTPUT.resolve()
    for INPUT in INPUTS:
        SOURCE = INPUT.resolve()
        if SOURCE.is_relative_to(DESTINATION) or (SOURCE.is_dir() and DESTINATION.is_relative_to(SOURCE)):
            raise ValueError("dataset, executable and replay output paths must not overlap")


def replace_output(OUTPUT):
    """Create an empty run directory; callers decide when input validation is complete."""
    REMOVED = OUTPUT.exists()
    if REMOVED:
        if not OUTPUT.is_dir():
            raise ValueError(f"replay output already exists and is not a directory: {OUTPUT}")
        shutil.rmtree(OUTPUT)
    OUTPUT.mkdir(parents=True, exist_ok=False)
    return REMOVED


def write_metadata(PATH, METADATA):
    with PATH.open("x") as STREAM:
        json.dump(METADATA, STREAM, indent=2, allow_nan=False)
        STREAM.write("\n")


def run_estimator(BINARY, INPUT, OUTPUT, MAGNETIC=True):
    """Execute the shared C++ runner, propagating its nonzero exit and diagnostics."""
    COMMAND = [str(BINARY), str(INPUT), str(OUTPUT)]
    if not MAGNETIC:
        COMMAND.append("--no-mag")
    return subprocess.run(COMMAND, check=True, stdout=subprocess.PIPE, text=True).stdout
