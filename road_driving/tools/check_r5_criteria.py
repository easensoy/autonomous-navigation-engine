"""Evaluate the predeclared R5 criteria (benchmark/PROTOCOL_R5.md, V1-V6) from a closed_loop.jsonl.

    python check_r5_criteria.py road_driving/benchmark/heldout-r5/closed_loop.jsonl

A pair is the same scenario, condition, seed and filter, driven without and with the re-join. A fallback
stop is an episode whose stack ends in the fallback (still braking, or held after it).
"""
import collections
import json
import sys

FALLBACK = {"FALLBACK", "HOLD_AFTER_FALLBACK"}


def main():
    rows = [json.loads(line) for line in open(sys.argv[1])]
    est = [r for r in rows if r["mode"] != "ground_truth"]
    pairs = collections.defaultdict(dict)
    for r in est:
        # R5 recorded the setting as true/false; later runs write "off" or "on" for the same two stacks.
        rj = r["rejoin"] if isinstance(r["rejoin"], bool) else r["rejoin"] != "off"
        pairs[(r["scenario"], r["condition"], r["seed"], r["estimator"])][rj] = r
    incomplete = [k for k, v in pairs.items() if set(v) != {False, True}]
    if incomplete:
        sys.exit("%d episodes have no partner, e.g. %s" % (len(incomplete), incomplete[0]))
    conds = sorted({k[1] for k in pairs})
    filters = sorted({k[3] for k in pairs})
    nominal = [c for c in conds if c.startswith("nominal")]

    def cell(cond, filt, rejoin):
        return [v[rejoin] for k, v in pairs.items() if k[1] == cond and k[3] == filt]

    over = lambda rs: sum(1 for r in rs if r["clearance_violations"] > 0)
    deepest = lambda rs: max([r["max_crossing"] for r in rs] + [0.0])
    falls = lambda rs: sum(1 for r in rs if r["stack_mode"] in FALLBACK)
    same = lambda rs: sum(1 for r in rs if r["same_as_reference"])
    results = []

    # V1
    nom = [v for k, v in pairs.items() if k[1] in nominal]
    ident = sum(1 for v in nom if v[False]["trace_hash"] == v[True]["trace_hash"])
    results.append(("V1", ident == len(nom) and len(nom) > 0, "identical nominal pairs: %d of %d" % (ident, len(nom))))
    # V2
    k2 = [v[True] for k, v in pairs.items() if k[3] == "ekf_kerb2"]
    o2 = [r for r in k2 if r["clearance_violations"] > 0]
    results.append(("V2", not o2, "ekf_kerb2 with the re-join over a kerb in %d of %d episodes%s" % (
        len(o2), len(k2), (": " + ", ".join("%s %s s%d %.2f m" % (r["condition"], r["scenario"], r["seed"], r["max_crossing"]) for r in o2)) if o2 else "")))
    # V3
    worse = []
    for c in conds:
        for f in filters:
            a, b = cell(c, f, False), cell(c, f, True)
            if over(b) > over(a) or deepest(b) > deepest(a) + 0.05:
                worse.append("%s %s: %d (%.2f m) without, %d (%.2f m) with" % (c, f, over(a), deepest(a), over(b), deepest(b)))
    results.append(("V3", not worse, "; ".join(worse) if worse else "no cell crosses more often or deeper with the re-join"))
    # V4
    fa = sum(falls(cell(c, f, False)) for c in conds for f in filters)
    fb = sum(falls(cell(c, f, True)) for c in conds for f in filters)
    results.append(("V4", fb < fa, "fallback stops: %d without, %d with the re-join" % (fa, fb)))
    # V5
    lost = [c for c in conds if same(cell(c, "ekf_kerb2", True)) < same(cell(c, "ekf_kerb2", False))]
    sa = sum(same(cell(c, f, False)) for c in conds for f in filters)
    sb = sum(same(cell(c, f, True)) for c in conds for f in filters)
    results.append(("V5", not lost and sb >= sa, "ekf_kerb2 conditions with fewer: %s; all filters: %d without, %d with" % (
        ", ".join(lost) if lost else "none", sa, sb)))
    # V6
    inc = [r for r in k2 if r["incursion_depth"] > 0]
    results.append(("V6", not inc, "ekf_kerb2 closure incursions with the re-join: %d" % len(inc)))
    for name, ok, text in results:
        print("%-3s %-6s %s" % (name, "pass" if ok else "FAILED", text))


if __name__ == "__main__":
    main()
