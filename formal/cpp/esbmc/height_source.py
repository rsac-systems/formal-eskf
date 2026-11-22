#!/usr/bin/env python3
"""Prepare the pinned HeightAiding source for compositional verification.

One conditional lvalue needs explicit selection on the qualified frontend.
No numerical or reporting producer is replaced. No production files are modified.
Reviewed correspondence is not an automated refinement proof.
"""

import argparse
import hashlib
import json
from pathlib import Path
import sys

REPO_DIR = Path(__file__).resolve().parents[3]
PINS = {
    "include/formal_eskf/runtime/sensors/height/height_aiding.hpp": "3f3042740a2f2689c083e66ba9a8ace713ba4d9b8fb23419280935a4cd325c93",
}


def sha256(DATA):
    return hashlib.sha256(DATA).hexdigest()


def replace_once(TEXT, BEFORE, AFTER):
    if TEXT.count(BEFORE) != 1:
        raise ValueError(f"unexpected source fragment: {BEFORE}")
    return TEXT.replace(BEFORE, AFTER)


def generate(OUTPUT_DIR):
    for DIRECTORY in ("include", "integrations", "formal", "tests", "scripts", "configs"):
        if OUTPUT_DIR.resolve().is_relative_to((REPO_DIR / DIRECTORY).resolve()):
            raise ValueError("verification output must not replace source files")
    SOURCES = {}
    for NAME, EXPECTED in PINS.items():
        DATA = (REPO_DIR / NAME).read_bytes()
        if sha256(DATA) != EXPECTED:
            raise ValueError(f"source changed; review the verification boundary: {NAME}")
        SOURCES[Path(NAME).name] = DATA.decode()

    HEIGHT = SOURCES["height_aiding.hpp"]
    HEIGHT = replace_once(HEIGHT, "source == Source::barometer ? barometer : gnss",
                          "selected_sample(source, barometer, gnss)")
    HELPER = """    static Sample const & selected_sample(Source source, Sample const & barometer, Sample const & gnss)
    {
        if (source == Source::barometer)
        {
            return barometer;
        }
        return gnss;
    }
"""
    ANCHOR = "    static constexpr std::size_t index(Source source)"
    HEIGHT = replace_once(HEIGHT, ANCHOR, HELPER + ANCHOR)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    OUTPUTS = {"HeightAiding.hpp": HEIGHT}
    for NAME, TEXT in OUTPUTS.items():
        (OUTPUT_DIR / NAME).write_text(TEXT)
    MANIFEST = {
        "claim": "Actual source caller with independently checked producer contracts and reviewed correspondence.",
        "source_sha256": PINS,
        "generator_sha256": sha256(Path(__file__).read_bytes()),
        "generated_sha256": {NAME: sha256(TEXT.encode()) for NAME, TEXT in OUTPUTS.items()},
        "transformation": "Select the same borrowed sample through explicit returns. Keep all fields, settings, other bodies and evaluation order.",
    }
    (OUTPUT_DIR / "manifest.json").write_text(json.dumps(MANIFEST, indent=2) + "\n")


def main():
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument("output_dir", type=Path)
    ARGS = PARSER.parse_args()
    try:
        generate(ARGS.output_dir)
    except (OSError, ValueError, IndexError) as ERROR:
        print(f"error: {ERROR}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
