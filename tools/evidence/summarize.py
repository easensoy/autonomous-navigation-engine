"""Headline numbers for the README, computed from the raw exported and benchmark files.

Percentile convention: nearest-rank on the sorted sample (p-th percentile = value at rank
ceil(p/100 * n)). Latency statistic: each query's (or replan event's) median over its timed repeats;
percentiles are then taken across queries. p99 is reported only when n >= 1000.
"""

from __future__ import annotations

import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from statistics import median

WORKLOADS = ("warehouse_s", "warehouse_m", "warehouse_l")
VARIANTS = ("astar", "astar_zero", "dijkstra", "bfs")
PHASES = ("end_to_end", "copy", "remove", "search_first", "search_cached")


def percentile(values, p):
    xs = sorted(values)
    if not xs:
        return None
    rank = max(1, math.ceil(p / 100.0 * len(xs)))
    return xs[rank - 1]


def dist_summary(values, *, scale=1.0):
    n = len(values)
    out = {"n": n, "min": min(values) * scale, "p50": percentile(values, 50) * scale,
           "p95": percentile(values, 95) * scale, "max": max(values) * scale, "mean": sum(values) / n * scale}
    if n >= 1000:
        out["p99"] = percentile(values, 99) * scale
    return out


def read_csv(path: Path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def close(a, b):
    return abs(a - b) <= 1e-9 * max(1.0, abs(a), abs(b))


def summarize_queries(bench: Path, name: str) -> dict:
    samples = defaultdict(list)  # (query, variant) -> ns
    for row in read_csv(bench / f"{name}.query_times.csv"):
        samples[(int(row["query"]), row["variant"])].append(int(row["ns"]))
    per_query = defaultdict(dict)  # variant -> query -> median ns
    spread = defaultdict(list)
    for (q, v), xs in samples.items():
        m = median(xs)
        per_query[v][q] = m
        if len(xs) > 1 and m > 0:
            spread[v].append((max(xs) - min(xs)) / m)
    results = defaultdict(dict)
    for row in read_csv(bench / f"{name}.query_results.csv"):
        results[row["variant"]][int(row["query"])] = row
    run = json.loads((bench / f"{name}.query_run.json").read_text())

    out = {"nodes": run["graph_nodes"], "edges": run["graph_edges"], "fingerprint": run["graph_fingerprint"],
           "queries": run["queries"], "repeats": run["repeats"], "warmup": run["warmup"],
           "graph_build_ms": run["graph_build_ns"] / 1e6,
           "astar_first_query_ms": median(run["astar_first_query_ns"]) / 1e6,
           "astar_cached_query_ms": median(run["astar_cached_query_ns"]) / 1e6,
           "background_cpu_load": run.get("background_cpu_load"), "variants": {}}
    dijkstra = results["dijkstra"]
    for v in VARIANTS:
        lat = list(per_query[v].values())
        res = results[v]
        found = sum(r["found"] == "1" for r in res.values())
        valid = sum(r["valid"] == "1" for r in res.values())
        reachable = sum(dijkstra[q]["found"] == "1" for q in res)
        ratios = [float(r["cost_ratio_to_dijkstra"]) for r in res.values() if r["cost_ratio_to_dijkstra"]]
        optimal = sum(1 for q, r in res.items()
                      if r["found"] == "1" and dijkstra[q]["found"] == "1"
                      and close(float(r["cost"]), float(dijkstra[q]["cost"])))
        entry = {"latency_ms": dist_summary(lat, scale=1e-6), "found": found, "valid": valid,
                 "reachable": reachable, "optimal": optimal,
                 "cost_ratio": dist_summary(ratios) if ratios else None,
                 # Measurement variation: (max - min) / median of one query's timed repeats, over queries.
                 "repeat_spread_median": median(spread[v]) if spread[v] else None,
                 "repeat_spread_p95": percentile(spread[v], 95) if spread[v] else None,
                 # Workload variation: how much queries differ, from the per-query medians.
                 "workload_p95_over_p50": (percentile(lat, 95) / percentile(lat, 50)) if lat else None}
        if v in ("astar", "astar_zero"):
            exps = [int(r["astar_expansions"]) for r in res.values() if r["astar_expansions"]]
            entry["expansions"] = dist_summary(exps)
            misses = [(float(r["cost"]) - float(dijkstra[q]["cost"])) for q, r in res.items()
                      if r["found"] == "1" and not close(float(r["cost"]), float(dijkstra[q]["cost"]))]
            entry["miss_excess_cost"] = dist_summary(misses) if misses else None
            entry["miss_ratio"] = dist_summary([float(res[q]["cost_ratio_to_dijkstra"]) for q, r in res.items()
                                                if r["found"] == "1" and r["cost_ratio_to_dijkstra"]
                                                and not close(float(r["cost"]), float(dijkstra[q]["cost"]))]) \
                if misses else None
        if v == "bfs":
            entry["costlier_than_optimal"] = sum(1 for x in ratios if x > 1 + 1e-9)
        out["variants"][v] = entry
    a, d = per_query["astar"], per_query["dijkstra"]
    paired = [d[q] / a[q] for q in a if a[q] > 0]
    out["dijkstra_over_astar_paired"] = dist_summary(paired)
    z = per_query["astar_zero"]
    out["astar_zero_over_astar_paired"] = dist_summary([z[q] / a[q] for q in a if a[q] > 0])
    return out


def summarize_replans(bench: Path, name: str) -> dict:
    samples = defaultdict(list)
    for row in read_csv(bench / f"{name}.replan_times.csv"):
        samples[(int(row["event"]), row["phase"])].append(int(row["ns"]))
    per_event = defaultdict(dict)
    repeat_spread = []
    for (e, phase), xs in samples.items():
        per_event[phase][e] = median(xs)
        if phase == "end_to_end" and len(xs) > 1 and median(xs) > 0:
            repeat_spread.append((max(xs) - min(xs)) / median(xs))
    rows = read_csv(bench / f"{name}.replan_results.csv")
    run = json.loads((bench / f"{name}.replan_run.json").read_text())
    out = {"events": len(rows), "repeats": run["repeats"],
           "background_cpu_load": run.get("background_cpu_load"),
           "end_to_end_repeat_spread_median": median(repeat_spread) if repeat_spread else None,
           "on_route": sum(r["on_route"] == "1" for r in rows),
           "replanned": sum(r["replanned"] == "1" for r in rows),
           "found": sum(r["found"] == "1" for r in rows),
           "valid": sum(r["valid"] == "1" for r in rows),
           "avoids_blocked": sum(r["avoids_blocked"] == "1" for r in rows if r["found"] == "1"),
           "optimal": sum(1 for r in rows if r["found"] == "1" and r["optimum"] not in ("", "null")
                          and close(float(r["new_cost"]), float(r["optimum"]))),
           "blocked_nodes": dist_summary([int(r["blocked_nodes"]) for r in rows]),
           "phases_ms": {}}
    for phase in PHASES:
        out["phases_ms"][phase] = dist_summary(list(per_event[phase].values()), scale=1e-6)
    # Per-event shares of the end-to-end time, then their median.
    events = per_event["end_to_end"].keys()
    shares = defaultdict(list)
    for e in events:
        total = per_event["end_to_end"][e]
        if total <= 0:
            continue
        scale = per_event["search_first"][e] - per_event["search_cached"][e]
        shares["search"].append(per_event["search_cached"][e] / total)
        shares["scale_pass"].append(max(scale, 0) / total)
        shares["copy"].append(per_event["copy"][e] / total)
        shares["remove"].append(per_event["remove"][e] / total)
        parts = per_event["copy"][e] + per_event["remove"][e] + per_event["search_first"][e]
        shares["parts_over_end_to_end"].append(parts / total)
    out["share_of_end_to_end_median"] = {k: median(v) for k, v in shares.items()}
    return out


def summarize_hero(data: Path, bench: Path) -> dict:
    hero = json.loads((data / "hero.json").read_text())
    search = json.loads((data / "search.json").read_text())
    times = defaultdict(list)
    by_rep = defaultdict(dict)
    for row in read_csv(bench / "hero.times.csv"):
        times[row["phase"]].append(int(row["ns"]))
        by_rep[int(row["rep"])][row["phase"]] = int(row["ns"])
    shares = defaultdict(list)
    for rep in by_rep.values():
        total = rep["replan"]
        shares["search"].append(rep["search_cached"] / total)
        shares["scale_pass"].append(max(rep["search_first"] - rep["search_cached"], 0) / total)
        shares["copy"].append(rep["copy"] / total)
        shares["remove"].append(rep["remove"] / total)
        shares["parts_over_end_to_end"].append((rep["copy"] + rep["remove"] + rep["search_first"]) / total)
    run = json.loads((bench / "hero.run.json").read_text())
    replan = hero["replan"]
    out = {
        "graph": {"nodes": hero["graph"]["node_count"], "edges": hero["graph"]["edge_count"],
                  "fingerprint": hero["graph"]["fingerprint"]},
        "blocked_nodes": len(hero["blocked_nodes"]),
        "block_step": hero["block_step"],
        "initial": hero["routes"][0]["route"] | {"nodes": None},
        "abandoned_remainder": replan["abandoned_remainder"] | {"nodes": None},
        "new_remainder": replan["new_remainder"] | {"nodes": None},
        "traversed": hero["traversed"] | {"nodes": None},
        "steps": len(hero["timeline"]) - 1,
        "replan_position": replan["position"],
        "nodes_ahead_to_block": None,
        "timing_runs": run["repeats"],
        "background_cpu_load": run.get("background_cpu_load"),
        "timing_ms": {phase: dist_summary(xs, scale=1e-6) for phase, xs in times.items()},
        "replan_share_median": {k: median(v) for k, v in shares.items()},
        "checks_passed": all(c["pass"] for c in hero["checks"]),
    }
    route = hero["routes"][0]["route"]["nodes"]
    idx = replan["route_index"]
    blocked = set(hero["blocked_nodes"])
    ahead = next((k for k in range(idx + 1, len(route)) if route[k] in blocked), None)
    out["nodes_ahead_to_block"] = (ahead - idx) if ahead is not None else None
    out["search"] = {}
    for v in search["variants"]:
        out["search"][v["key"]] = {"label": v["label"], "cost": v["route"]["cost"], "length": v["route"]["length"],
                                   "hops": v["route"]["hops"], "expansions": v["expansions"],
                                   "pushes": v["pushes"],
                                   "never_expanded_below_optimum": len(v["never_expanded_below_optimum"])}
    ref = search["reference_shortest_length"]
    out["search"]["shortest_length_reference"] = {"cost": ref["cost"], "length": ref["length"], "hops": ref["hops"]}
    out["search"]["optimum"] = search["dijkstra_optimum"]
    out["search"]["checks_passed"] = all(c["pass"] for c in search["checks"])
    return out


def summarize_failures(data: Path) -> dict:
    out = {}
    for name in ("failure_goal_blocked", "failure_disconnected"):
        d = json.loads((data / f"{name}.json").read_text())
        out[name] = {k: d[k] for k in (
            "goal_blocked", "status_before", "status_after", "state_after", "console_message", "execute_step_after",
            "retry_after", "state_after_retry", "remaining_after_is_infinite", "invalidated_route_blocked_nodes",
            "reachable", "astar_expansions", "astar_pushed_nodes", "reachable_component_nodes", "reduced_graph_nodes")}
        out[name]["current_path_after_nodes"] = len(d["current_path_after"])
        out[name]["invalidated_route_nodes"] = len(d["invalidated_route"])
        out[name]["replanner_route_empty"] = not d["replanner_route"]
        out[name]["astar_route_empty"] = not d["astar_route"]
        out[name]["astar_pushed_never_expanded"] = len(d["astar_pushed_never_expanded"])
        out[name]["blocked_nodes"] = len(d["blocked_nodes"])
        out[name]["checks_passed"] = all(c["pass"] for c in d["checks"])
    return out


def summarize_astar_fix(data: Path, queries: dict, replans: dict):
    """Before and after the A* open-list fix: the pre-fix misses frozen in astar_fix_before.json, and
    the same workloads now. check_outputs.py checks each frozen miss against the current results."""
    path = data / "astar_fix_before.json"
    if not path.exists():
        return None
    before = json.loads(path.read_text(encoding="utf-8"))
    found = [e for e in before["queries"] if e["astar_cost"] is not None]
    return {
        "before": {
            "source": before["source"],
            "query_counts": before["query_counts"],
            "replan_counts": before["replan_counts"],
            "queries_without_route": len(before["queries"]) - len(found),
            "excess_cost": dist_summary([e["astar_cost"] - e["dijkstra_cost"] for e in found]) if found else None,
            "cost_ratio": dist_summary([e["astar_cost"] / e["dijkstra_cost"] for e in found
                                        if e["dijkstra_cost"] > 0]) if found else None,
            "search_replan_query_astar": before["search_replan_query_astar"],
            "failure_disconnected_astar": before["failure_disconnected_astar"],
        },
        "after": {
            "queries": sum(queries[n]["queries"] for n in WORKLOADS),
            "queries_optimal": sum(queries[n]["variants"]["astar"]["optimal"] for n in WORKLOADS),
            "replans": sum(replans[n]["events"] for n in WORKLOADS),
            "replans_optimal": sum(replans[n]["optimal"] for n in WORKLOADS),
        },
    }


def write_summary(data: Path, out_path: Path):
    bench = data / "bench"
    queries = {name: summarize_queries(bench, name) for name in WORKLOADS}
    replans = {name: summarize_replans(bench, name) for name in WORKLOADS}
    summary = {
        "schema": "ane-evidence/summary/1",
        "percentiles": "nearest rank; latency = per-query median over repeats, percentiles across queries; "
                       "p99 only when n >= 1000",
        "hero": summarize_hero(data, bench),
        "failures": summarize_failures(data),
        "queries": queries,
        "replans": replans,
        "astar_fix": summarize_astar_fix(data, queries, replans),
        "environment": (json.loads((bench / "environment.json").read_text(encoding="utf-8"))
                        if (bench / "environment.json").exists() else None),
    }
    out_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {out_path}")
    return summary


if __name__ == "__main__":
    repo = Path(__file__).resolve().parents[2]
    write_summary(repo / "docs" / "data", repo / "docs" / "data" / "summary.json")
