"""Offline checks of extractor rejection; never refresh production proof pins."""

import hashlib
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def check_source_binding(ROOT, RELATIVE, MODULE, ERROR_TEXT):
    """Return True for current pins; otherwise verify fail-closed CLI behavior.

    Stale bindings retain a proof gap. The caller still tests extraction helpers,
    and runs its full correspondence assertions when the production pins match.
    """
    STALE = [NAME for NAME, HASH in MODULE.PINS.items()
             if not (ROOT / NAME).is_file()
             or hashlib.sha256((ROOT / NAME).read_bytes()).hexdigest() != HASH]
    if not STALE:
        return True

    with tempfile.TemporaryDirectory(prefix="formal-eskf-stale-binding-") as TEMP:
        FIXTURE = Path(TEMP) / "repo"
        ORIGINALS = {}
        for NAME in [*MODULE.PINS, str(RELATIVE)]:
            if not (ROOT / NAME).is_file():
                continue
            TARGET = FIXTURE / NAME
            TARGET.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT / NAME, TARGET)
            ORIGINALS[NAME] = TARGET.read_bytes()
        COMMAND = [sys.executable, str(FIXTURE / RELATIVE)]
        OUTPUT = FIXTURE / "output"
        EXPECTED_ERROR = ERROR_TEXT if (ROOT / STALE[0]).is_file() else "No such file"
        for EXISTING in (False, True):
            if EXISTING:
                OUTPUT.mkdir()
                (OUTPUT / "sentinel").write_bytes(b"previous evidence")
            RESULT = subprocess.run([*COMMAND, str(OUTPUT)], capture_output=True, text=True)
            assert RESULT.returncode == 1, RESULT.stdout + RESULT.stderr
            assert EXPECTED_ERROR in RESULT.stderr and STALE[0] in RESULT.stderr, RESULT.stderr
            if EXISTING:
                assert {P.name: P.read_bytes() for P in OUTPUT.iterdir()} == {"sentinel": b"previous evidence"}
            else:
                assert not OUTPUT.exists(), "failed extraction created output"
        assert all((FIXTURE / NAME).read_bytes() == DATA for NAME, DATA in ORIGINALS.items())
    print(f"{RELATIVE.name}: stale binding rejection verified; source-correspondence proof remains pending")
    return False
