"""Independent checks on the generated data, figures and README.

Re-validates the exported routes against the exported graph with plain arithmetic (no planner),
confirms the benchmark measured the documented workloads, and confirms that every number the
README quotes equals the value computed from the raw files.

    python tools/evidence/check_outputs.py      # exit status 1 if anything fails
"""

from __future__ import annotations

import csv
import json
import math
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
DATA = REPO / "docs" / "data"


class Report:
    def __init__(self):
        self.failures = []
        self.count = 0

    def check(self, name, ok, detail=""):
        self.count += 1
        if not ok:
            self.failures.append(f"{name}{': ' + detail if detail else ''}")
        return ok


def load(name):
    return json.loads((DATA / name).read_text(encoding="utf-8"))


class Graph:
    def __init__(self, record):
        self.xy = {n[0]: (n[1], n[2]) for n in record["graph"]["nodes"]}
        self.w = {}
        for a, b, w in record["graph"]["edges"]:
            self.w[(a, b)] = w
            self.w[(b, a)] = w

    def follows(self, route):
        return bool(route) and route[0] in self.xy and all((a, b) in self.w for a, b in zip(route, route[1:]))

    def cost(self, route):
        return sum(self.w[(a, b)] for a, b in zip(route, route[1:]))

    def length(self, route):
        return sum(math.dist(self.xy[a], self.xy[b]) for a, b in zip(route, route[1:]))


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol * max(1.0, abs(a), abs(b))


def check_route_record(r, g, record, label):
    nodes = record["nodes"]
    if not r.check(f"{label}: follows exported edges", g.follows(nodes) or not nodes):
        return  # cost and length are undefined off the graph
    if nodes:
        r.check(f"{label}: cost recomputes", close(g.cost(nodes), record["cost"]), f"{g.cost(nodes)} vs {record['cost']}")
        r.check(f"{label}: length recomputes", close(g.length(nodes), record["length"]))
        r.check(f"{label}: hops", len(nodes) - 1 == record["hops"])


def unexpanded_below(variant, optimum):
    """Nodes whose cheapest push had priority below the optimum but that were never expanded, recomputed
    from the exported events. A best-first search with a consistent heuristic leaves none."""
    best = {}
    for node, _parent, cost, priority in variant["push_events"]:
        if node not in best or cost < best[node][0]:
            best[node] = (cost, priority)
    expanded = {e[0] for e in variant["expand_events"]}
    limit = optimum - 1e-9 * max(1.0, optimum)
    return sorted(n for n, (_c, p) in best.items() if p < limit and n not in expanded)


def check_exports(r):
    hero = load("hero.json")
    g = Graph(hero)
    r.check("hero: exporter checks all passed", all(c["pass"] for c in hero["checks"]))
    for route in hero["routes"]:
        check_route_record(r, g, route["route"], f"hero route v{route['version']}")
    blocked = set(hero["blocked_nodes"])
    k = hero["replan"]["route_index"]
    trav = hero["traversed"]["nodes"]
    r.check("hero: starts at start, ends at goal", trav[0] == hero["start"] and trav[-1] == hero["goal"])
    r.check("hero: no blocked node after the replan", not (set(trav[k + 1:]) & blocked))
    r.check("hero: timeline positions are the traversed route", [s[1] for s in hero["timeline"]] == trav)
    check_route_record(r, g, hero["replan"]["new_remainder"], "hero new remainder")
    r.check("hero: replanned remainder cost equals the exported Dijkstra optimum",
            close(hero["replan"]["new_remainder"]["cost"], hero["replan"]["dijkstra_optimum_on_reduced_graph"]))

    search = load("search.json")
    gs = Graph(search)
    r.check("search: exporter checks all passed", all(c["pass"] for c in search["checks"]))
    blocked = set(search["blocked_nodes"])
    for v in search["variants"]:
        check_route_record(r, gs, v["route"], f"search {v['key']}")
        r.check(f"search {v['key']}: avoids blocked nodes", not (set(v["route"]["nodes"]) & blocked))
        r.check(f"search {v['key']}: expansion count = events", v["expansions"] == len(v["expand_events"]))
        order = [e[2] for e in v["expand_events"]]
        r.check(f"search {v['key']}: expansion priorities nondecreasing",
                all(b >= a - 1e-9 * max(1, abs(a)) for a, b in zip(order, order[1:])))
    opt = search["dijkstra_optimum"]
    for key in ("dijkstra", "astar", "astar_zero"):
        v = next(v for v in search["variants"] if v["key"] == key)
        r.check(f"search {key}: optimal cost", close(v["route"]["cost"], opt))
        dropped = unexpanded_below(v, opt)
        r.check(f"search {key}: every node pushed below the optimum was expanded (from the events)", not dropped,
                f"{len(dropped)} never expanded: {dropped[:5]}")
    for name in ("failure_goal_blocked", "failure_disconnected"):
        d = load(f"{name}.json")
        r.check(f"{name}: exporter checks all passed", all(c["pass"] for c in d["checks"]))
        r.check(f"{name}: replanner returned no route", d["replanner_route"] == [])
        route, step = d["route_before"]["nodes"], d["block_step"]
        r.check(f"{name}: navigator is BLOCKED and does not step",
                d["state_after"] == "BLOCKED" and d["status_after"].startswith("BLOCKED") and not d["execute_step_after"])
        r.check(f"{name}: getCurrentPath is the travelled part only", d["current_path_after"] == route[: step + 1])
        r.check(f"{name}: getInvalidatedRoute is the abandoned route ahead", d["invalidated_route"] == route[step:])
        r.check(f"{name}: remaining distance infinite; retry fails while blocked",
                d["remaining_after_is_infinite"] and not d["retry_after"] and d["state_after_retry"] == "BLOCKED")
        if not d["goal_blocked"]:
            r.check(f"{name}: A* expanded the whole reachable component before giving up",
                    d["astar_expansions"] == d["reachable_component_nodes"] and not d["astar_pushed_never_expanded"],
                    f"{d['astar_expansions']} of {d['reachable_component_nodes']}")


def query_results(name):
    """{query index: {variant: results row}} from a benchmark's raw results file."""
    out = {}
    with open(DATA / "bench" / f"{name}.query_results.csv", newline="") as f:
        for row in csv.DictReader(f):
            out.setdefault(int(row["query"]), {})[row["variant"]] = row
    return out


def replan_results(name):
    with open(DATA / "bench" / f"{name}.replan_results.csv", newline="") as f:
        return {int(row["event"]): row for row in csv.DictReader(f)}


def optimal_replan(row):
    optimum = float(row["optimum"])
    if math.isinf(optimum):
        return row["found"] != "1"
    return row["found"] == "1" and close(float(row["new_cost"]), optimum)


def check_bench(r):
    workloads = {w["name"]: w for w in json.loads((DATA / "workloads" / "workloads.json").read_text())["workloads"]}
    for name, w in workloads.items():
        for kind in ("query", "replan"):
            run = json.loads((DATA / "bench" / f"{name}.{kind}_run.json").read_text())
            r.check(f"{name} {kind}: measured graph matches the saved workload",
                    run["graph_fingerprint"] == w["fingerprint"], f"{run['graph_fingerprint']} vs {w['fingerprint']}")
            r.check(f"{name} {kind}: no invalid routes", run["invalid_routes"] == 0)
            r.check(f"{name} {kind}: no observer attached", run["observers"].startswith("none"))
        with open(DATA / "bench" / f"{name}.query_times.csv", newline="") as f:
            rows = sum(1 for _ in csv.DictReader(f))
        run = json.loads((DATA / "bench" / f"{name}.query_run.json").read_text())
        r.check(f"{name}: one sample per query, variant and repeat",
                rows == run["queries"] * 4 * run["repeats"], f"{rows} rows")
        # A* must return Dijkstra's cost on every query and replan: by the benchmark's own count, and
        # recomputed here from the raw results files.
        r.check(f"{name}: benchmark counts no non-optimal A* route",
                run["astar_not_optimal"] == 0 and run["astar_zero_not_optimal"] == 0,
                f"{run['astar_not_optimal']} (h = 0: {run['astar_zero_not_optimal']})")
        replan_run = json.loads((DATA / "bench" / f"{name}.replan_run.json").read_text())
        r.check(f"{name}: benchmark counts no non-optimal replan", replan_run["replans_not_optimal"] == 0,
                str(replan_run["replans_not_optimal"]))
        worse = [q for q, v in query_results(name).items() for key in ("astar", "astar_zero")
                 if v["dijkstra"]["found"] == "1"
                 and (v[key]["found"] != "1" or not close(float(v[key]["cost"]), float(v["dijkstra"]["cost"])))]
        r.check(f"{name}: A* and A* (h = 0) cost equals Dijkstra's on every query (raw results)", not worse,
                f"{len(worse)} differ, first {worse[:5]}")
        worse = [e for e, row in replan_results(name).items() if row["on_route"] == "1" and not optimal_replan(row)]
        r.check(f"{name}: every replan costs Dijkstra's optimum (raw results)", not worse,
                f"{len(worse)} differ, first {worse[:5]}")
    hero_run = json.loads((DATA / "bench" / "hero.run.json").read_text())
    r.check("hero bench: replans valid", hero_run["invalid_routes"] == 0)


def check_fix_record(r):
    """Each A* miss frozen before the open-list fix must now be the same query (same start and goal,
    same Dijkstra optimum) answered at Dijkstra's cost."""
    path = DATA / "astar_fix_before.json"
    if not r.check("A* fix record exists", path.exists()):
        return
    before = json.loads(path.read_text(encoding="utf-8"))
    queries = {name: query_results(name) for name in before["query_counts"]}
    bad = []
    for e in before["queries"]:
        row = queries[e["workload"]].get(e["query"], {})
        a, d = row.get("astar"), row.get("dijkstra")
        same = (a is not None and d is not None and int(a["start"]) == e["start"] and int(a["goal"]) == e["goal"]
                and close(float(d["cost"]), e["dijkstra_cost"]))
        if not (same and a["found"] == "1" and close(float(a["cost"]), float(d["cost"]))):
            bad.append((e["workload"], e["query"]))
    r.check(f"each of the {len(before['queries'])} queries A* missed before the fix now costs Dijkstra's optimum",
            not bad, f"{len(bad)} do not: {bad[:5]}")
    replans = {name: replan_results(name) for name in before["replan_counts"]}
    bad = []
    for e in before["replans"]:
        row = replans[e["workload"]].get(e["event"])
        same = (row is not None and int(row["position"]) == e["position"] and int(row["goal"]) == e["goal"]
                and close(float(row["optimum"]), e["optimum"]))
        if not (same and optimal_replan(row)):
            bad.append((e["workload"], e["event"]))
    r.check(f"each of the {len(before['replans'])} replans that missed before the fix now costs the optimum",
            not bad, f"{len(bad)} do not: {bad[:5]}")
    r.check("A* fix record: counts match its entries",
            sum(c["costlier"] for c in before["query_counts"].values()) == len(before["queries"])
            and sum(c["costlier"] for c in before["replan_counts"].values()) == len(before["replans"]))


def readme_claims(s):
    """(description, text the README must contain), each computed from summary.json."""
    h, q, rp = s["hero"], s["queries"], s["replans"]
    se = h["search"]
    claims = [
        ("initial plan cost", f"{h['initial']['cost']:.1f}"),
        ("blocked node count", f"{h['blocked_nodes']} nodes"),
        ("new remainder cost", f"{h['new_remainder']['cost']:.1f}"),
        ("abandoned remainder cost", f"{h['abandoned_remainder']['cost']:.1f}"),
        ("shortest-by-distance detour length", f"{se['shortest_length_reference']['length']:.1f} u"),
        ("shortest-by-distance detour cost", f"{se['shortest_length_reference']['cost']:.1f}"),
        ("A* expansions on the replan query", f"{se['astar']['expansions']}"),
        ("A* h=0 expansions", f"{se['astar_zero']['expansions']}"),
        ("Dijkstra expansions", f"{se['dijkstra']['expansions']}"),
        ("BFS route cost", f"{se['bfs']['cost']:.1f}"),
        ("hero replan p50", f"{h['timing_ms']['replan']['p50']:.2f} ms"),
        ("hero plan p50", f"{h['timing_ms']['plan']['p50']:.2f} ms"),
    ]
    names = ("warehouse_s", "warehouse_m", "warehouse_l")
    for name in names:
        claims.append((f"{name} node count", f"{q[name]['nodes']:,}"))
    fix = s["astar_fix"]
    counts = [f"{fix['before']['query_counts'][n]['costlier']}/{fix['before']['query_counts'][n]['queries']:,}"
              for n in names]
    claims.append(("A* costlier queries before the fix", f"{counts[0]}, {counts[1]} and {counts[2]}"))
    ex = fix["before"]["excess_cost"]
    claims.append(("A* excess cost range before the fix", f"{ex['min']:.2f}–{ex['max']:.2f}"))
    claims.append(("A* worst cost ratio before the fix", f"{fix['before']['cost_ratio']['max'] - 1:.1%}"))
    after = fix["after"]
    if after["queries_optimal"] == after["queries"] and after["replans_optimal"] == after["replans"]:
        after_text = f"all {after['queries']:,} queries and {after['replans']:,} replans"
    else:  # never in the README, so the check fails until the text is updated
        after_text = (f"{after['queries_optimal']:,} of {after['queries']:,} queries and "
                      f"{after['replans_optimal']:,} of {after['replans']:,} replans")
    claims.append(("A* optimal on the benchmark after the fix", after_text))
    fd = s["failures"]["failure_disconnected"]
    claims.append(("A* expansions when the goal is cut off",
                   f"all {fd['reachable_component_nodes']} reachable nodes"
                   if fd["astar_expansions"] == fd["reachable_component_nodes"]
                   else f"{fd['astar_expansions']} of {fd['reachable_component_nodes']} reachable nodes"))
    shares = [rp[n]["share_of_end_to_end_median"]["search"] for n in names]
    shares.append(h["replan_share_median"]["search"])
    claims.append(("search share range", f"{min(shares):.0%}–{max(shares):.0%}".replace("%–", "–")))
    large = rp["warehouse_l"]["phases_ms"]
    claims.append(("large replan p50", f"{large['end_to_end']['p50']:.1f} ms"))
    claims.append(("large search-alone p50", f"{large['search_cached']['p50']:.2f} ms"))
    paired = [q[n]["dijkstra_over_astar_paired"]["p50"] for n in names]
    claims.append(("Dijkstra/A* paired p50 range", f"{min(paired):.1f}–{max(paired):.1f}×"))
    zero = [q[n]["astar_zero_over_astar_paired"]["p50"] for n in names]
    claims.append(("A* h=0 / A* paired p50 range", f"{min(zero):.1f}–{max(zero):.1f}×"))
    bfs = [q[n]["variants"]["bfs"]["cost_ratio"]["p50"] for n in names]
    claims.append(("BFS median cost ratio range", f"×{min(bfs):.2f}–×{max(bfs):.2f}"))
    # Measurement environment of the published run.
    gate = (s.get("environment") or {}).get("load_gate")
    if gate:
        claims.append(("load gate threshold", f"{gate['threshold']:.0%}"))
    loads = [x for x in [q[n].get("background_cpu_load") for n in names] + [rp[n].get("background_cpu_load") for n in names]
             + [h.get("background_cpu_load")] if x is not None and x >= 0]
    if loads:
        low, high = f"{min(loads):.0%}", f"{max(loads):.0%}"
        claims.append(("background CPU load range", low if low == high else f"{low[:-1]}–{high}"))
    history = DATA / "history" / "index.json"
    if history.exists():
        runs = json.loads(history.read_text(encoding="utf-8"))["runs"]
        claims.append(("benchmark runs in the history", f"{len(runs)} runs"))
    return claims


def check_readme(r):
    readme = (REPO / "README.md").read_text(encoding="utf-8")
    summary = load("summary.json")
    for description, text in readme_claims(summary):
        r.check(f"README states {description} as '{text}'", text in readme)
    # Every relative image or link target exists.
    for target in re.findall(r'(?:src|srcset|href)="([^"#:]+)"', readme) + re.findall(r"\]\(([^)#:]+)\)", readme):
        target = target.strip()
        if target and not target.startswith(("http", "mailto")):
            r.check(f"README link target exists: {target}", (REPO / target).exists())


def check_history(r):
    """The published run must be recorded in docs/data/history and be its newest complete run (the
    declared selection rule); every recorded run keeps its summary and manifest."""
    import history

    index_path = DATA / "history" / "index.json"
    if not r.check("history index exists", index_path.exists()):
        return
    index = json.loads(index_path.read_text(encoding="utf-8"))
    runs = index.get("runs", [])
    r.check("history states its selection rule", bool(index.get("selection_rule")))
    stamps = [run["generated_utc"] for run in runs]
    r.check("history runs are unique and in time order", len(set(stamps)) == len(stamps) and stamps == sorted(stamps))
    complete = [run for run in runs if run.get("folder")]
    for run in complete:
        folder = DATA / "history" / run["folder"]
        r.check(f"history {run['folder']}: summary and manifest kept",
                (folder / "summary.json").exists() and (folder / "manifest.json").exists())
    digest = history.records_digest(DATA / "bench")
    r.check("the published run is the newest complete run in the history",
            bool(complete) and complete[-1].get("bench_records_sha256") == digest)


def main(repo: Path = REPO) -> bool:
    global REPO, DATA
    REPO = Path(repo)
    DATA = REPO / "docs" / "data"
    r = Report()
    check_exports(r)
    check_bench(r)
    check_fix_record(r)
    check_history(r)
    check_readme(r)
    for failure in r.failures:
        print(f"  FAIL {failure}")
    print(f"check_outputs: {r.count - len(r.failures)}/{r.count} checks passed")
    return not r.failures


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
