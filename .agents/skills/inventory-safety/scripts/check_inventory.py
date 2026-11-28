#!/usr/bin/env python3
"""Check inventory fingerprints and accounting; do not evaluate safety claims."""

import argparse
import hashlib
import re
import sys
from pathlib import Path


CASE_ID = re.compile(r"\b(?:FHA|FMEA|FTA)-C[0-9]{3,}\b")
LINK = re.compile(r"\[([^\]]+)\]\(([^)]+)\)")
REQUIREMENT_ID = re.compile(r"^\*\*ID:\*\*\s*`([^`]+)`", re.MULTILINE)


def tables(PATH):
    """Read the deliberately small Markdown-table format used by this skill."""
    HEADER = None
    for NUMBER, LINE in enumerate(PATH.read_text().splitlines(), 1):
        if not LINE.startswith("|"):
            HEADER = None
            continue
        CELLS = [CELL.strip() for CELL in LINE.strip().strip("|").split("|")]
        if all(re.fullmatch(r":?-+:?", CELL) for CELL in CELLS):
            continue
        if HEADER is None:
            if CASE_ID.search(CELLS[0]):
                raise ValueError(f"{PATH}:{NUMBER}: candidate row without table header")
            HEADER = CELLS
            continue
        if len(CELLS) != len(HEADER):
            raise ValueError(f"{PATH}:{NUMBER}: inconsistent table columns")
        yield dict(zip(HEADER, CELLS))


def linked_ids(CELL):
    return [LABEL for LABEL, _ in LINK.findall(CELL)]


def check_links(PATH, ERRORS):
    for LABEL, TARGET in LINK.findall(PATH.read_text()):
        if re.match(r"[a-z]+://", TARGET):
            continue
        FILE, _, ANCHOR = TARGET.partition("#")
        DESTINATION = (PATH.parent / FILE).resolve() if FILE else PATH
        if not DESTINATION.is_file():
            ERRORS.append(f"{PATH.name}: missing link {TARGET}")
        elif CASE_ID.fullmatch(LABEL) and ANCHOR:
            if f'id="{ANCHOR}"' not in DESTINATION.read_text():
                ERRORS.append(f"{PATH.name}: missing scenario anchor {TARGET}")


def check_fingerprint(ROW, FIELD, BASE, ERRORS):
    REFERENCES = LINK.findall(ROW[FIELD])
    if len(REFERENCES) != 1:
        raise ValueError(f"{FIELD}: expected one linked file")
    ID, TARGET = REFERENCES[0]
    PATH = (BASE / TARGET).resolve()
    HASH = ROW.get("SHA-256", "")
    if not re.fullmatch(r"[0-9a-f]{64}", HASH):
        ERRORS.append(f"{ID}: invalid SHA-256")
    elif PATH.is_file() and hashlib.sha256(PATH.read_bytes()).hexdigest() != HASH:
        ERRORS.append(f"{ID}: input changed; review before refreshing its fingerprint")
    return ID, PATH


def check_inventory(REPO, INVENTORY):
    ERRORS = []
    REQUIREMENTS = {}
    for PATH in sorted((REPO / "docs/spec/requirements").rglob("*.md")):
        IDS = REQUIREMENT_ID.findall(PATH.read_text())
        if len(IDS) != 1:
            ERRORS.append(f"{PATH}: expected one canonical requirement ID")
            continue
        ID = IDS[0]
        if ID in REQUIREMENTS:
            ERRORS.append(f"duplicate requirement ID: {ID}")
        REQUIREMENTS[ID] = PATH.resolve()
    if not REQUIREMENTS:
        ERRORS.append("no canonical requirements found")

    CASES = {}
    ROWS = {}
    DOCUMENTS = [INVENTORY / "README.md"]
    for METHOD in ("FHA", "FMEA", "FTA"):
        PATH = INVENTORY / f"{METHOD.lower()}.md"
        DOCUMENTS.append(PATH)
        for ROW in tables(PATH):
            if "ID" not in ROW:
                continue
            MATCH = CASE_ID.search(ROW["ID"])
            if not MATCH or not MATCH[0].startswith(METHOD + "-"):
                ERRORS.append(f"{PATH.name}: invalid candidate ID {ROW['ID']}")
                continue
            ID = MATCH[0]
            if ID in CASES:
                ERRORS.append(f"duplicate candidate ID: {ID}")
            ROWS[ID] = ROW
            REFERENCES = linked_ids(ROW.get("Requirements", "")) if METHOD == "FMEA" else []
            if METHOD != "FMEA":
                CASES[ID] = set()
                continue
            if not REFERENCES or len(REFERENCES) != len(set(REFERENCES)):
                ERRORS.append(f"{ID}: empty or duplicate requirement references")
            for REFERENCE, TARGET in LINK.findall(ROW.get("Requirements", "")):
                if REFERENCE not in REQUIREMENTS:
                    ERRORS.append(f"{ID}: unknown requirement {REFERENCE}")
                elif (PATH.parent / TARGET).resolve() != REQUIREMENTS[REFERENCE]:
                    ERRORS.append(f"{ID}: incorrect requirement link {REFERENCE}")
            CASES[ID] = set(REFERENCES)

    for ID, ROW in ROWS.items():
        if ID.startswith("FMEA-"):
            continue
        CONTRIBUTORS = linked_ids(ROW.get("Contributors", ""))
        if not CONTRIBUTORS or len(CONTRIBUTORS) != len(set(CONTRIBUTORS)):
            ERRORS.append(f"{ID}: empty or duplicate contributors")
        EXPECTED = {FM for FM, OTHER in ROWS.items() if FM.startswith("FMEA-")
                    and ID in linked_ids(OTHER.get("Related candidates", ""))}
        if set(CONTRIBUTORS) != EXPECTED:
            ERRORS.append(f"{ID}: nonreciprocal contributor links")
        for FM in CONTRIBUTORS:
            if not FM.startswith("FMEA-") or FM not in CASES:
                ERRORS.append(f"{ID}: unknown FMEA contributor {FM}")
            else:
                CASES[ID].update(CASES[FM])

    LEDGER = {}
    SUPPLEMENTAL = set()
    for ROW in tables(INVENTORY / "README.md"):
        if "Input" in ROW:
            _, PATH = check_fingerprint(ROW, "Input", INVENTORY, ERRORS)
            if PATH in SUPPLEMENTAL:
                ERRORS.append(f"duplicate supplemental input: {PATH}")
            SUPPLEMENTAL.add(PATH)
        if "Requirement" not in ROW:
            continue
        ID, PATH = check_fingerprint(ROW, "Requirement", INVENTORY, ERRORS)
        if ID in LEDGER:
            ERRORS.append(f"duplicate ledger requirement: {ID}")
        if ID in REQUIREMENTS and PATH != REQUIREMENTS[ID]:
            ERRORS.append(f"{ID}: ledger points to the wrong requirement file")
        LEDGER[ID] = ROW

    for ID in sorted(REQUIREMENTS.keys() - LEDGER.keys()):
        ERRORS.append(f"unaccounted requirement: {ID}")
    for ID in sorted(LEDGER.keys() - REQUIREMENTS.keys()):
        ERRORS.append(f"removed or unknown requirement in ledger: {ID}")
    for ID, ROW in LEDGER.items():
        for METHOD in ("FHA", "FMEA", "FTA"):
            CELL = ROW.get(METHOD, "")
            EXPECTED = {CASE for CASE, REFS in CASES.items()
                        if CASE.startswith(METHOD + "-") and ID in REFS}
            if CELL.startswith(("N/A:", "Pending:")):
                if len(CELL.partition(":")[2].strip()) < 12:
                    ERRORS.append(f"{ID}/{METHOD}: disposition needs a specific reason")
                ACTUAL = set()
            else:
                IDS = linked_ids(CELL)
                ACTUAL = set(IDS)
                if not IDS or len(IDS) != len(ACTUAL):
                    ERRORS.append(f"{ID}/{METHOD}: empty or duplicate accounting")
                if any(not CASE.startswith(METHOD + "-") for CASE in IDS):
                    ERRORS.append(f"{ID}/{METHOD}: wrong method in accounting")
            if ACTUAL != EXPECTED:
                ERRORS.append(f"{ID}/{METHOD}: nonreciprocal accounting; "
                              f"listed={sorted(ACTUAL)}, candidates={sorted(EXPECTED)}")

    for PATH in DOCUMENTS:
        check_links(PATH, ERRORS)
        for ID in set(CASE_ID.findall(PATH.read_text())) - CASES.keys():
            ERRORS.append(f"{PATH.name}: unknown candidate reference {ID}")
    if ERRORS:
        for ERROR in ERRORS:
            print(f"error: {ERROR}", file=sys.stderr)
        return 1

    COUNTS = ", ".join(f"{METHOD} {sum(ID.startswith(METHOD + '-') for ID in CASES)}"
                       for METHOD in ("FHA", "FMEA", "FTA"))
    print(f"Inventory accounting: pass ({len(REQUIREMENTS)} requirements; {COUNTS})")
    print("This checks recorded inputs and references, not scenario completeness or safety.")
    return 0


def main():
    PARSER = argparse.ArgumentParser(description=__doc__)
    PARSER.add_argument("--repo", type=Path, default=Path.cwd(), help="repository root")
    PARSER.add_argument("--inventory", type=Path, help="alternate inventory directory")
    ARGS = PARSER.parse_args()
    REPO = ARGS.repo.resolve()
    INVENTORY = (ARGS.inventory or REPO / "docs/analysis/scenarios").resolve()
    try:
        return check_inventory(REPO, INVENTORY)
    except (OSError, ValueError) as ERROR:
        print(f"error: {ERROR}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
