#!/usr/bin/env python3
"""Mutation test for check_outputs.py.

Copies the committed evidence (README.md and docs/) into a temporary directory, requires the copy to
pass, then applies one corruption at a time and requires check_outputs to report a failure:

  - a route step that is not an edge of the exported graph;
  - a README number that disagrees with docs/data/summary.json;
  - a README link to a file that does not exist;
  - an A* benchmark result costlier than Dijkstra's;
  - a trace whose expansion count disagrees with its recorded events;
  - a benchmark run that is missing from docs/data/history.

    python tools/evidence/test_check_outputs.py      # exit status 1 if any corruption goes unnoticed

Standard library only, like check_outputs.py.
"""

from __future__ import annotations

import contextlib
import io
import json
import shutil
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(HERE))

import check_outputs  # noqa: E402


def load(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def dump(path: Path, value) -> None:
    path.write_text(json.dumps(value, indent=1), encoding="utf-8")


def break_route(root: Path) -> None:
    path = root / "docs" / "data" / "hero.json"
    hero = load(path)
    nodes = hero["routes"][0]["route"]["nodes"]
    nodes[2], nodes[-2] = nodes[-2], nodes[2]  # two far-apart nodes: the steps are no longer edges
    dump(path, hero)


def break_readme_number(root: Path) -> None:
    path = root / "README.md"
    expansions = load(root / "docs" / "data" / "summary.json")["hero"]["search"]["astar"]["expansions"]
    text = path.read_text(encoding="utf-8")
    path.write_text(text.replace(str(expansions), str(expansions + 1)), encoding="utf-8")


def break_readme_link(root: Path) -> None:
    path = root / "README.md"
    path.write_text(path.read_text(encoding="utf-8") + "\n[missing figure](docs/assets/missing.png)\n", encoding="utf-8")


def break_bench_result(root: Path) -> None:
    path = root / "docs" / "data" / "bench" / "warehouse_s.query_results.csv"
    lines = path.read_text(encoding="utf-8").splitlines()
    header = lines[0].split(",")
    variant, cost = header.index("variant"), header.index("cost")
    for i, line in enumerate(lines[1:], start=1):
        fields = line.split(",")
        if fields[variant] == "astar" and fields[header.index("found")] == "1":
            fields[cost] = repr(float(fields[cost]) + 1.0)
            lines[i] = ",".join(fields)
            break
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def break_expansion_count(root: Path) -> None:
    path = root / "docs" / "data" / "search.json"
    search = load(path)
    search["variants"][0]["expansions"] += 1
    dump(path, search)


def break_history(root: Path) -> None:
    path = root / "docs" / "data" / "history" / "index.json"
    index = load(path)
    index["runs"] = [run for run in index["runs"] if not run.get("folder")] + \
                    [run for run in index["runs"] if run.get("folder")][:-1]  # drop the published run
    dump(path, index)


MUTATIONS = [
    ("a route step that is not an edge", break_route),
    ("a README number that disagrees with the data", break_readme_number),
    ("a README link to a missing file", break_readme_link),
    ("an A* result costlier than Dijkstra's", break_bench_result),
    ("an expansion count that disagrees with the events", break_expansion_count),
    ("a published run missing from the history", break_history),
]


def run_checker(root: Path) -> bool:
    """True if check_outputs passes. A crash counts against the checker: it must report, not raise."""
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            return check_outputs.main(root)
    except Exception as error:  # noqa: BLE001
        raise SystemExit(f"FAIL check_outputs raised {type(error).__name__}: {error}") from error


def fresh_copy(parent: Path) -> Path:
    root = Path(tempfile.mkdtemp(dir=parent))
    shutil.copy2(REPO / "README.md", root / "README.md")
    shutil.copytree(REPO / "docs", root / "docs")
    return root


def main() -> int:
    failures = 0
    with tempfile.TemporaryDirectory() as tmp:
        parent = Path(tmp)
        if not run_checker(fresh_copy(parent)):
            print("FAIL the unmodified copy does not pass check_outputs")
            return 1
        print("ok   the unmodified copy passes")
        for name, mutate in MUTATIONS:
            root = fresh_copy(parent)
            mutate(root)
            if run_checker(root):
                print(f"FAIL not detected: {name}")
                failures += 1
            else:
                print(f"ok   detected: {name}")
    print(f"test_check_outputs: {len(MUTATIONS) - failures}/{len(MUTATIONS)} corruptions detected")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
