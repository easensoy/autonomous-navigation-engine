#!/usr/bin/env python3
"""Aggregates a campaign's trial metrics into ros2/evidence/summary.json and ros2/evidence/SUMMARY.md.

  python ros2/tools/summarize_trials.py --campaign <name>

Every attempt in the campaign index is counted, including attempts that produced no data. Numeric
metrics are summarised per scenario as n / min / median / max over the trials that produced data
(five trials are too few for means and standard deviations to say more). Standard library only.
"""

import argparse
import json
import statistics
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
NUMERIC = ["final_position_error_m", "min_clearance_m", "max_cross_track_m", "rms_cross_track_m", "replans",
           "sensor_blockage_reports", "operator_blockage_reports", "detection_latency_s", "detection_latency_from_trigger_s",
           "replan_reaction_s", "stop_time_after_terminal_s", "drift_after_stop_m", "hold_detect_s", "blind_travel_m",
           "failure_detect_s", "path_length_m", "executed_graph_cost", "planned_route_cost", "mission_time_s",
           "mission_wall_s", "real_time_factor", "collisions", "interventions", "blocked_nodes_entered", "waypoint_progress"]


def stats(values):
    values = [v for v in values if isinstance(v, (int, float)) and not isinstance(v, bool)]
    if not values:
        return None
    return {"n": len(values), "min": min(values), "median": statistics.median(values), "max": max(values)}


def fmt(s, digits=2, unit=""):
    if not s:
        return "–"
    if s["n"] == 1 or s["min"] == s["max"]:
        return f"{s['median']:.{digits}f}{unit}"
    return f"{s['median']:.{digits}f}{unit} ({s['min']:.{digits}f}–{s['max']:.{digits}f})"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--campaign", required=True)
    parser.add_argument("--out", type=Path, default=REPO / "ros2" / "evidence",
                        help="folder for summary.json and SUMMARY.md (default: ros2/evidence, the published campaign)")
    args = parser.parse_args()
    root = REPO / "ros2" / "evidence" / "trials" / args.campaign
    campaign = json.loads((root / "campaign.json").read_text())
    index = json.loads((root / "index.json").read_text())

    scenarios = {}
    for attempt in index["attempts"]:
        entry = scenarios.setdefault(attempt["scenario"], {"attempts": 0, "with_data": 0, "accepted": 0, "consistent": 0,
                                                             "outcomes": {}, "trials": [], "criteria": {}})
        entry["attempts"] += 1
        metrics_file = root / attempt["scenario"] / f"trial_{attempt['trial']}" / "metrics.json"
        if not metrics_file.exists():
            entry["trials"].append({"trial": attempt["trial"], "seed": attempt["seed"], "status": attempt["status"]})
            continue
        result = json.loads(metrics_file.read_text())
        m = result["metrics"]
        entry["with_data"] += 1
        entry["accepted"] += bool(result["accepted"])
        entry["consistent"] += bool(result["consistent"])
        entry["outcomes"][m["outcome"]] = entry["outcomes"].get(m["outcome"], 0) + 1
        entry["trials"].append({"trial": attempt["trial"], "seed": attempt["seed"], "status": attempt["status"],
                                "outcome": m["outcome"], "accepted": result["accepted"], "consistent": result["consistent"],
                                "failed_criteria": [f"{v['metric']} {v['op']} {v['target']} (got {v['value']})"
                                                    for v in result["acceptance"] if not v["pass"]],
                                "failed_checks": [k for k, ok in result["consistency"].items() if not ok],
                                "metrics": {k: m.get(k) for k in NUMERIC}, "engine_call_ms": m["engine_call_ms"]})
        for v in result["acceptance"]:
            key = f"{v['metric']} {v['op']} {v['target']}"
            entry["criteria"][key] = entry["criteria"].get(key, 0) + bool(v["pass"])

    # Wall-clock values were measured on the container's monotonic clock; the campaign's clock checks give
    # its rate against the host (container seconds per host second). Host-time values divide by it.
    rates = [c["container_monotonic_rate"] for c in (campaign.get("clock_check_before"), campaign.get("clock_check_after"))
             if c and "container_monotonic_rate" in c]
    rate = statistics.mean(rates) if rates else None
    for name, entry in scenarios.items():
        with_data = [t for t in entry["trials"] if "metrics" in t]
        entry["metrics"] = {k: stats([t["metrics"][k] for t in with_data]) for k in NUMERIC}
        calls = {}
        for t in with_data:
            for kind, values in t["engine_call_ms"].items():
                calls.setdefault(kind, []).extend(values)
        entry["engine_call_ms"] = {k: stats(v) for k, v in calls.items() if v}
        if rate:
            entry["metrics"]["real_time_factor_host"] = stats([t["metrics"]["real_time_factor"] * rate for t in with_data
                                                                if t["metrics"]["real_time_factor"]])
            entry["metrics"]["mission_wall_host_s"] = stats([t["metrics"]["mission_wall_s"] / rate for t in with_data
                                                              if t["metrics"]["mission_wall_s"]])
            entry["engine_call_ms_host"] = {k: stats([x / rate for x in v]) for k, v in calls.items() if v}
        scenario_file = REPO / "ros2" / "ane_sim" / "scenarios" / f"{name}.txt"
        entry["title"] = next((line[6:].strip() for line in scenario_file.read_text().splitlines() if line.startswith("title ")), name)

    attempts = len(index["attempts"])
    with_data = sum(e["with_data"] for e in scenarios.values())
    summary = {
        "schema": "ane-sim/summary/1",
        "campaign": args.campaign,
        "engine_commit": campaign["engine_commit"],
        "engine_sources_modified": campaign["engine_sources_modified"],
        "integration_tree_sha256": campaign["integration_tree_sha256"],
        "image": campaign["image"],
        "host": campaign["host"],
        "timing": campaign["timing"],
        "clock_check_before": campaign.get("clock_check_before"),
        "clock_check_after": campaign.get("clock_check_after"),
        "container_monotonic_rate": rate,
        "attempts": attempts,
        "attempts_with_data": with_data,
        "accepted": sum(e["accepted"] for e in scenarios.values()),
        "scenarios": scenarios,
    }
    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    (out / "summary.json").write_text(json.dumps(summary, indent=1) + "\n", encoding="utf-8")

    lines = [f"# Simulation campaign {args.campaign}", "",
             f"Generated by `ros2/tools/summarize_trials.py` from `ros2/evidence/trials/{args.campaign}/`. "
             f"Gazebo Harmonic simulation; not a physical experiment. Engine commit `{campaign['engine_commit'][:7]}`.", "",
             f"{attempts} trials attempted, {with_data} produced data, {summary['accepted']} met every acceptance criterion "
             "fixed in their scenario file.", "",
             "Values are median (min–max) over the trials of a scenario. Times are simulation seconds unless marked "
             "*host*: wall-clock values measured on the container's monotonic clock and divided by its measured rate "
             f"against the host ({rate:.4f} container seconds per host second)." if rate else
             "Values are median (min–max) over the trials of a scenario.", "",
             "| Scenario | Trials (accepted / attempted) | Outcomes | Final position error (completed missions) | Min. clearance | Max. deviation from roadmap | Mission time | Real-time factor (host) |",
             "|---|---|---|---|---|---|---|---|"]
    for name, e in scenarios.items():
        m = e["metrics"]
        outcomes = ", ".join(f"{k} ×{v}" for k, v in e["outcomes"].items())
        error = fmt(m["final_position_error_m"], 3, " m") if set(e["outcomes"]) == {"COMPLETED"} else "– (halted)"
        rtf = m.get("real_time_factor_host") or m["real_time_factor"]
        lines.append(f"| {name} | {e['accepted']} / {e['attempts']} | {outcomes} | {error} | "
                     f"{fmt(m['min_clearance_m'], 2, ' m')} | {fmt(m['max_cross_track_m'], 3, ' m')} | "
                     f"{fmt(m['mission_time_s'], 1, ' s')} | {fmt(rtf, 3)} |")
    lines += ["", "| Scenario | Detection latency | Replans | Engine call (host ms) | Hold detected after | Travel without feedback | Failure detected after | Drift after stop |",
              "|---|---|---|---|---|---|---|---|"]
    for name, e in scenarios.items():
        m = e["metrics"]
        call_stats = e.get("engine_call_ms_host") or e["engine_call_ms"]
        calls = "; ".join(f"{k.split('_', 1)[1].lower()} {fmt(v, 3)}" for k, v in call_stats.items())
        lines.append(f"| {name} | {fmt(m['detection_latency_s'], 3, ' s')} | {fmt(m['replans'], 0)} | {calls} | "
                     f"{fmt(m['hold_detect_s'], 3, ' s')} | {fmt(m['blind_travel_m'], 3, ' m')} | {fmt(m['failure_detect_s'], 2, ' s')} | "
                     f"{fmt(m['drift_after_stop_m'], 3, ' m')} |")
    lines += ["", "## Acceptance criteria", ""]
    for name, e in scenarios.items():
        lines.append(f"**{name}**: " + "; ".join(f"`{k}` {v}/{e['with_data']}" for k, v in e["criteria"].items()))
        lines.append("")
    failed = [(name, t) for name, e in scenarios.items() for t in e["trials"]
              if t.get("status") != "completed" or not t.get("accepted", False) or t.get("failed_checks")]
    lines += ["## Attempts that failed a criterion or produced no data", ""]
    if not failed:
        lines.append("None.")
    for name, t in failed:
        lines.append(f"- {name} trial {t['trial']} (seed {t['seed']}): status {t['status']}; "
                     f"failed criteria: {', '.join(t.get('failed_criteria', [])) or 'none'}; "
                     f"failed consistency checks: {', '.join(t.get('failed_checks', [])) or 'none'}")
    (out / "SUMMARY.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"{attempts} attempts, {with_data} with data, {summary['accepted']} accepted")


if __name__ == "__main__":
    main()
