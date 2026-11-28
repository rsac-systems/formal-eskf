"""Input preparation and process/output management for host replay entrypoints."""

import csv
import hashlib
import math
import json
import os
from pathlib import Path
import shutil
import subprocess


NAVIGATION_HEADER = "timestamp_us,pn,pe,pd,vn,ve,vd,eph,epv,speed_accuracy"
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


def magnetic_model(BINARY, PROVIDER, REQUEST):
    """Query at the session's requested point, then express the field in its requested NED basis."""
    LATITUDE, LONGITUDE, ORIGIN_LATITUDE, ORIGIN_LONGITUDE = REQUEST
    RESULT = subprocess.run([str(PROVIDER)], input=f"{LATITUDE:.17g} {LONGITUDE:.17g}\n",
                            text=True, capture_output=True, check=True, timeout=30)
    VALUES = [float(VALUE) for VALUE in RESULT.stdout.split()]
    if (len(VALUES) != 3 or not all(math.isfinite(VALUE) for VALUE in VALUES)
            or not -180 <= VALUES[0] <= 180 or not -90 <= VALUES[1] <= 90 or VALUES[2] <= 0):
        raise ValueError("invalid magnetic provider result")
    DECLINATION, INCLINATION, STRENGTH = math.radians(VALUES[0]), math.radians(VALUES[1]), VALUES[2]
    FIELD = [STRENGTH * math.cos(INCLINATION) * math.cos(DECLINATION),
             STRENGTH * math.cos(INCLINATION) * math.sin(DECLINATION), STRENGTH * math.sin(INCLINATION)]
    _, ROTATION = project_coordinates(BINARY, [ORIGIN_LATITUDE, ORIGIN_LONGITUDE, 0], [[LATITUDE, LONGITUDE, 0]])[0]
    FIELD = [dot(ROW, FIELD) for ROW in ROTATION]
    return [math.atan2(FIELD[1], FIELD[0]), math.atan2(FIELD[2], math.hypot(*FIELD[:2])), STRENGTH]


def run_estimator(BINARY, INPUT, OUTPUT, MAGNETIC=True, MAGNETIC_MODEL=None, MODE="px4"):
    """Run the host estimator; optional model requests use a line protocol, never shell commands."""
    if MODE not in ("px4", "kitti"):
        raise ValueError("replay mode must be px4 or kitti")
    COMMAND = [str(BINARY), str(INPUT), str(OUTPUT),
               "--kitti-session" if MODE == "kitti" else "--session"]
    if not MAGNETIC:
        COMMAND.append("--no-mag")
    LINES = []
    with subprocess.Popen(COMMAND, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True) as PROCESS:
        try:
            for LINE in PROCESS.stdout:
                if LINE.startswith("@MODEL "):
                    REQUEST = [float(VALUE) for VALUE in LINE.split()[1:]]
                    if len(REQUEST) != 4 or not all(math.isfinite(VALUE) for VALUE in REQUEST):
                        raise ValueError("invalid session magnetic-model request")
                    VALUES = magnetic_model(BINARY, MAGNETIC_MODEL, REQUEST) if MAGNETIC_MODEL is not None else None
                    PROCESS.stdin.write((",".join(format(VALUE, ".17g") for VALUE in VALUES)
                                         if VALUES is not None else "unavailable") + "\n")
                    PROCESS.stdin.flush()
                else:
                    LINES.append(LINE)
            PROCESS.stdin.close()
            STATUS = PROCESS.wait()
            if STATUS:
                raise subprocess.CalledProcessError(STATUS, COMMAND, output="".join(LINES))
        except BaseException:
            PROCESS.kill()
            PROCESS.wait()
            raise
    SUMMARY_FILE = OUTPUT / "summary.json"
    SUMMARY = json.loads(SUMMARY_FILE.read_text())
    SUMMARY["settings_sha256"] = hashlib.sha256((INPUT / "settings.csv").read_bytes()).hexdigest()
    SUMMARY["executable_sha256"] = hashlib.sha256(Path(BINARY).read_bytes()).hexdigest()
    SUMMARY_FILE.write_text(json.dumps(SUMMARY, indent=2) + "\n")
    return "".join(LINES)
