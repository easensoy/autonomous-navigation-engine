#!/usr/bin/env python3
"""Reruns evidence_export on the committed scenarios and compares the result with docs/data.

Every field must match exactly, including every route, expansion and push event, except the two
fields that name the compiler and the scenario path. A difference means the committed traces no
longer describe what the committed code does.

    python tools/evidence/compare_traces.py build/tools/evidence/evidence_export

Standard library only.
"""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SCENARIOS = REPO / "tools" / "evidence" / "scenarios"
DATA = REPO / "docs" / "data"
JOBS = [
    ("hero", "hero_replan.txt", "hero.json"),
    ("search", "hero_replan.txt", "search.json"),
    ("failure", "failure_goal_blocked.txt", "failure_goal_blocked.json"),
    ("failure", "failure_disconnected.txt", "failure_disconnected.json"),
]
IGNORED = {"compiler", "scenario_file"}


def first_difference(a, b, path="$"):
    """Path of the first difference between two parsed JSON values, or None."""
    if isinstance(a, dict) and isinstance(b, dict):
        for key in sorted(set(a) | set(b)):
            if key in IGNORED and path == "$":
                continue
            if key not in a or key not in b:
                return f"{path}.{key} (present in only one file)"
            found = first_difference(a[key], b[key], f"{path}.{key}")
            if found:
                return found
        return None
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return f"{path} (length {len(a)} vs {len(b)})"
        for i, (x, y) in enumerate(zip(a, b)):
            found = first_difference(x, y, f"{path}[{i}]")
            if found:
                return found
        return None
    return None if a == b else f"{path} ({a!r} vs {b!r})"


def main(argv) -> int:
    if len(argv) != 2:
        print(__doc__)
        return 2
    tool = Path(argv[1])
    failures = 0
    with tempfile.TemporaryDirectory() as tmp:
        for command, scenario, name in JOBS:
            out = Path(tmp) / name
            subprocess.run([str(tool), command, str(SCENARIOS / scenario), str(out)], check=True,
                           stdout=subprocess.DEVNULL)
            difference = first_difference(json.loads(out.read_text(encoding="utf-8")),
                                          json.loads((DATA / name).read_text(encoding="utf-8")))
            if difference:
                failures += 1
                print(f"FAIL {name}: first difference at {difference}")
            else:
                print(f"ok   {name}: identical to the committed trace")
    print(f"compare_traces: {len(JOBS) - failures}/{len(JOBS)} traces reproduced")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
