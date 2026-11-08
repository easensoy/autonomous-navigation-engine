#!/usr/bin/env python3
"""Paired comparison of two `evidence_bench queries` runs on the same workload.

    python tools/evidence/compare_bench.py <dir-A> <dir-B> <workload> [<dir-A2> <dir-B2> ...]

For each planner, each query's latency is the median of its timed repeats. The ratio B ÷ A is taken
per query (paired), and its median and 5th–95th percentile range are reported. With several (A, B)
directory pairs, the queries of all pairs are pooled. Used to measure what the SearchObserver hooks
cost with no observer attached (A = library at HEAD, B = working tree); see docs/EVIDENCE.md.
"""

from __future__ import annotations

import csv
import json
import math
import sys
from collections import defaultdict
from pathlib import Path
from statistics import median


def per_query(directory: Path, workload: str):
    samples = defaultdict(list)
    with open(directory / f"{workload}.query_times.csv", newline="") as f:
        for row in csv.DictReader(f):
            samples[(row["variant"], int(row["query"]))].append(int(row["ns"]))
    run = json.loads((directory / f"{workload}.query_run.json").read_text())
    return {key: median(xs) for key, xs in samples.items()}, run


def percentile(xs, p):
    xs = sorted(xs)
    return xs[max(0, math.ceil(p / 100 * len(xs)) - 1)]


def main(argv):
    if len(argv) < 4 or (len(argv) - 1) % 2 != 1:
        print(__doc__)
        return 2
    workload = argv[3]
    pairs = [(Path(argv[1]), Path(argv[2]))] + [(Path(argv[i]), Path(argv[i + 1])) for i in range(4, len(argv), 2)]
    ratios = defaultdict(list)
    for a_dir, b_dir in pairs:
        a, run_a = per_query(a_dir, workload)
        b, run_b = per_query(b_dir, workload)
        if run_a["graph_fingerprint"] != run_b["graph_fingerprint"]:
            print(f"different graphs in {a_dir} and {b_dir}")
            return 1
        for key, value in a.items():
            if key in b and value > 0:
                ratios[key[0]].append(b[key] / value)
    print(f"{workload}: B/A latency per query (paired), {len(pairs)} pair(s) of runs")
    for variant in ("astar", "astar_zero", "dijkstra", "bfs"):
        xs = ratios.get(variant, [])
        if xs:
            print(f"  {variant:11s} median {median(xs):.3f}   p5 {percentile(xs, 5):.3f}   p95 {percentile(xs, 95):.3f}"
                  f"   n={len(xs)}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
