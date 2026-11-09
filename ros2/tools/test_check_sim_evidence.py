#!/usr/bin/env python3
"""Mutation test for check_sim_evidence.py: corrupts a copy of the simulation evidence in several ways
and requires the checker to fail on each, after passing on the unmodified copy.

  python ros2/tools/test_check_sim_evidence.py [--route-check <ane_route_check>]

Standard library only.
"""

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]


def run_checker(root, route_check):
    command = [sys.executable, str(root / "ros2" / "tools" / "check_sim_evidence.py")]
    if route_check:
        command += ["--route-check", route_check]
    result = subprocess.run(command, capture_output=True, text=True)
    return result.returncode, (result.stdout + result.stderr).strip().splitlines()[-1:]


def first_trial(root, scenario):
    summary = json.loads((root / "ros2" / "evidence" / "summary.json").read_text(encoding="utf-8"))
    return root / "ros2" / "evidence" / "trials" / summary["campaign"] / scenario / "trial_0"


def edit_json(path, change):
    data = json.loads(path.read_text(encoding="utf-8"))
    change(data)
    path.write_text(json.dumps(data, indent=1) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--route-check")
    args = parser.parse_args()

    def metric_edit(r):
        r["metrics"]["final_position_error_m"] = 0.5

    def route_edit(t):
        route = next(e for e in t["event_log"] if e["type"] == "ROUTE_REPLACED")
        route["nodes"][2], route["nodes"][3] = route["nodes"][3], route["nodes"][2]

    def readme_edit(root):
        # Corrupt the first number the checker requires in the README's simulation section.
        sys.path.insert(0, str(root / "ros2" / "tools"))
        import check_sim_evidence
        summary = json.loads((root / "ros2" / "evidence" / "summary.json").read_text(encoding="utf-8"))
        claim = check_sim_evidence.readme_claims(summary)[1][0]
        readme = (root / "README.md").read_text(encoding="utf-8")
        wrong = ("8" if claim[0] == "9" else "9") + claim[1:]
        (root / "README.md").write_text(readme.replace(claim, wrong), encoding="utf-8")
        sys.path.pop(0)
        sys.modules.pop("check_sim_evidence", None)

    mutations = [
        ("a trial metric changed after the fact", lambda root: edit_json(first_trial(root, "A_nominal") / "metrics.json", metric_edit)),
        ("an acceptance criterion loosened", lambda root: (root / "ros2/ane_sim/scenarios/A_nominal.txt").write_text(
            (root / "ros2/ane_sim/scenarios/A_nominal.txt").read_text(encoding="utf-8").replace("<= 0.10", "<= 0.20"), encoding="utf-8")),
        ("a trial manifest deleted", lambda root: (first_trial(root, "C_disconnection") / "manifest.json").unlink()),
        ("integration code changed", lambda root: (root / "ros2/ane_sim/launch/sim.launch.py").write_text(
            (root / "ros2/ane_sim/launch/sim.launch.py").read_text(encoding="utf-8") + "\n# edited\n", encoding="utf-8")),
        ("a README number changed", readme_edit),
    ]
    if args.route_check:
        mutations.append(("a replacement route edited", lambda root: edit_json(first_trial(root, "B_aisle_blockage") / "trajectory.json", route_edit)))

    failures = 0
    with tempfile.TemporaryDirectory() as tmp:
        base = Path(tmp) / "base"
        base.mkdir()
        # The checker never reads the recordings' media, so the copies leave them out.
        shutil.copytree(REPO / "ros2", base / "ros2", ignore=shutil.ignore_patterns("__pycache__", "*.mp4", "*.gif"))
        shutil.copy2(REPO / "README.md", base / "README.md")
        code, last = run_checker(base, args.route_check)
        ok = code == 0
        failures += not ok
        print(f"{'ok  ' if ok else 'FAIL'} unmodified evidence passes: {last}")
        for i, (what, mutate) in enumerate(mutations):
            root = Path(tmp) / f"m{i}"
            shutil.copytree(base, root)
            mutate(root)
            code, last = run_checker(root, args.route_check)
            ok = code != 0
            failures += not ok
            print(f"{'ok  ' if ok else 'FAIL'} detected: {what}: {last}")
    print(f"test_check_sim_evidence: {len(mutations) + 1 - failures}/{len(mutations) + 1} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
