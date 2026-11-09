#!/usr/bin/env python3
"""Checks the published simulation evidence without running the simulator.

  python ros2/tools/check_sim_evidence.py [--route-check <ane_route_check>]

For the campaign named in ros2/evidence/summary.json it checks that
  - every attempt in the index has its manifest, and every attempt with data its runner result, metrics
    and trajectory;
  - every manifest's scenario SHA-256 matches the committed scenario file (criteria unchanged since);
  - the campaign's integration-tree hash matches the committed ros2/ code and configuration;
  - every acceptance verdict, re-evaluated from the stored metrics with the committed criteria, matches;
  - the summary's counts follow from the trial files;
  - the nodes each trial reached form a walk on the committed roadmap;
  - with --route-check: every route the navigator received from the engine during the trial (planned,
    replaced, or none when BLOCKED) is planned again by this checkout's engine from the same node
    around the same blocked nodes, and comes out identical;
  - the numbers the README quotes for the simulation match the summary.
Exits with status 1 on any failure. Standard library only (plus the route-check binary, if given).
"""

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "ros2" / "ane_sim" / "scripts"))
from scenario_lib import compare, integration_tree_sha256, load_roadmap, load_scenario  # noqa: E402

RESULTS = []


def check(ok, what):
    RESULTS.append((bool(ok), what))
    if not ok:
        print(f"FAIL {what}")
    return bool(ok)


def route_check(tool, roadmap_file, start, goal, blocked):
    out = subprocess.run([tool, str(roadmap_file), str(start), str(goal), *map(str, blocked)],
                         capture_output=True, text=True, check=True).stdout.split("\n")
    return None if out[0].strip() == "no-route" else [int(v) for v in out[0].split()[1:]]


def readme_claims(summary):
    """Strings the README's simulation section must contain, derived from the summary."""
    s = summary["scenarios"]
    m = {name: e["metrics"] for name, e in s.items()}
    claims = [
        (f"{summary['attempts']} trials", "number of attempted trials"),
        (f"{summary['accepted']} of {summary['attempts']}", "trials that met every criterion"),
    ]
    det = m["B_aisle_blockage"]["detection_latency_s"]
    if det:
        claims.append((f"{det['min']:.2f}–{det['max']:.2f} s", "B detection latency range"))
    err = [m[n]["final_position_error_m"] for n in ("A_nominal", "B_aisle_blockage", "D_feedback_dropout") if m[n]["final_position_error_m"]]
    if err:
        lo, hi = min(e["min"] for e in err), max(e["max"] for e in err)
        claims.append((f"{lo:.3f}–{hi:.3f} m", "final position error range of the completed scenarios"))
    clearance = min(e["metrics"]["min_clearance_m"]["min"] for e in s.values() if e["metrics"]["min_clearance_m"])
    claims.append((f"{clearance:.2f} m", "smallest clearance of any trial"))
    hold = m["D_feedback_dropout"]["hold_detect_s"]
    if hold:
        claims.append((f"{hold['max']:.2f} s", "D slowest feedback-loss detection"))
    blind = [m[n]["blind_travel_m"] for n in ("D_feedback_dropout", "E_feedback_loss") if m[n]["blind_travel_m"]]
    if blind:
        claims.append((f"{max(b['max'] for b in blind):.2f} m", "longest travel without feedback"))
    return claims


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--route-check", help="path to ane_route_check built from this checkout")
    args = parser.parse_args()

    evidence = REPO / "ros2" / "evidence"
    summary = json.loads((evidence / "summary.json").read_text(encoding="utf-8"))
    root = evidence / "trials" / summary["campaign"]
    campaign = json.loads((root / "campaign.json").read_text(encoding="utf-8"))
    index = json.loads((root / "index.json").read_text(encoding="utf-8"))
    roadmap_file = REPO / "ros2" / "ane_sim" / "maps" / "warehouse_roadmap.txt"
    roadmap = load_roadmap(roadmap_file)

    check(campaign["integration_tree_sha256"] == integration_tree_sha256(REPO / "ros2"),
          "campaign integration-tree hash matches the committed ros2/ code and configuration")
    # `git status --porcelain` entries for src/, include/ and CMakeLists.txt at campaign time; untracked
    # files ("??") are not part of the build, so only tracked changes count as modified engine sources.
    tracked = [c for c in campaign.get("engine_source_changes", []) if not c.startswith("??")]
    check(not tracked, f"the engine sources were unmodified when the campaign ran {tracked or ''}")

    accepted_by_scenario, outcomes_by_scenario, attempts_by_scenario = {}, {}, {}
    replayed = 0
    for attempt in index["attempts"]:
        name, k = attempt["scenario"], attempt["trial"]
        folder = root / name / f"trial_{k}"
        attempts_by_scenario[name] = attempts_by_scenario.get(name, 0) + 1
        if not check((folder / "manifest.json").exists(), f"{name} trial {k}: manifest present"):
            continue
        manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8"))
        scenario_file = REPO / manifest["scenario_file"]
        check(manifest["scenario_sha256"] == hashlib.sha256(scenario_file.read_bytes()).hexdigest()
              or manifest["scenario_sha256"] == hashlib.sha256(scenario_file.read_bytes().replace(b"\r\n", b"\n")).hexdigest(),
              f"{name} trial {k}: scenario file unchanged since the trial")
        if attempt["status"] != "completed":
            continue
        files = {f: folder / f for f in ("runner_result.json", "metrics.json", "trajectory.json")}
        if not check(all(p.exists() for p in files.values()), f"{name} trial {k}: runner result, metrics and trajectory present"):
            continue
        result = json.loads(files["metrics.json"].read_text(encoding="utf-8"))
        trajectory = json.loads(files["trajectory.json"].read_text(encoding="utf-8"))
        scenario = load_scenario(scenario_file)
        verdicts = [compare(result["metrics"].get(metric), op, target) for metric, op, target in scenario.accept]
        check(verdicts == [v["pass"] for v in result["acceptance"]] and len(verdicts) == len(result["acceptance"]),
              f"{name} trial {k}: acceptance verdicts follow from the stored metrics and the committed criteria")
        check(result["accepted"] == all(verdicts), f"{name} trial {k}: accepted flag")
        accepted_by_scenario[name] = accepted_by_scenario.get(name, 0) + bool(result["accepted"])
        outcome = result["metrics"]["outcome"]
        outcomes_by_scenario.setdefault(name, {})[outcome] = outcomes_by_scenario.setdefault(name, {}).get(outcome, 0) + 1

        log = trajectory["event_log"]
        reached = [e["nodes"][0] for e in log if e["type"] == "NODE_REACHED"]
        start = next((e["nodes"][0] for e in log if e["type"] == "ROUTE_PLANNED"), None)
        walk = ([start] if start is not None else []) + reached
        check(all((a, b) in roadmap["weights"] for a, b in zip(walk, walk[1:])),
              f"{name} trial {k}: reached nodes form a walk on the roadmap")
        if args.route_check:
            goal = roadmap["stations"][scenario.goal]
            blocked = []
            for e in log:
                if e["type"] == "BLOCKAGE_REPORTED":
                    blocked += [n for n in e["nodes"] if n not in blocked]
                elif e["type"] in ("ROUTE_PLANNED", "ROUTE_REPLACED"):
                    replayed += 1
                    check(route_check(args.route_check, roadmap_file, e["nodes"][0], goal, blocked) == e["nodes"],
                          f"{name} trial {k}: {e['type']} at t={e['t']:.2f} s reproduced by the engine")
                elif e["type"] == "ROUTE_BLOCKED":
                    replayed += 1
                    check(route_check(args.route_check, roadmap_file, e["nodes"][0], goal, blocked) is None,
                          f"{name} trial {k}: BLOCKED at t={e['t']:.2f} s has no route in the engine either")

    for name, entry in summary["scenarios"].items():
        check(entry["attempts"] == attempts_by_scenario.get(name, 0), f"summary: {name} attempts")
        check(entry["accepted"] == accepted_by_scenario.get(name, 0), f"summary: {name} accepted")
        check(entry["outcomes"] == outcomes_by_scenario.get(name, {}), f"summary: {name} outcomes")
    check(summary["attempts"] == len(index["attempts"]), "summary: total attempts")

    readme = (REPO / "README.md").read_text(encoding="utf-8")
    section = readme.split("## Robotics simulation validation", 1)
    if check(len(section) == 2, "README has a 'Robotics simulation validation' section"):
        text = section[1].split("\n## ", 1)[0]
        for claim, what in readme_claims(summary):
            check(claim in text, f"README simulation section states {what} as '{claim}'")

    passed = sum(ok for ok, _ in RESULTS)
    print(f"check_sim_evidence: {passed}/{len(RESULTS)} checks passed"
          + (f"; {replayed} engine routes replayed" if args.route_check else ""))
    return 0 if passed == len(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
