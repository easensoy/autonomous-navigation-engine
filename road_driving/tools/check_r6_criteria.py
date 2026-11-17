"""Evaluate the predeclared R6 criteria (benchmark/PROTOCOL_R6.md, W1-W6) from a closed_loop.jsonl.

    python check_r6_criteria.py road_driving/benchmark/heldout-r6/closed_loop.jsonl

A pair is the same scenario, condition, seed and filter, driven without any re-join ("off") and with the
verified re-join ("verified"). A fallback stop is an episode whose stack ends in the fallback (still braking,
or held after it). A pair is acted on when the verified episode re-joined at least one correction.
"""
import collections
import json
import sys

FALLBACK = {"FALLBACK", "HOLD_AFTER_FALLBACK"}
GNSS_ONLY = ("ekf", "ekf_int", "ekf_int2")
KERB = ("ekf_kerb", "ekf_kerb2")


def main():
    rows = [json.loads(line) for line in open(sys.argv[1])]
    pairs = collections.defaultdict(dict)
    for r in rows:
        if r["mode"] != "ground_truth":
            pairs[(r["scenario"], r["condition"], r["seed"], r["estimator"])][r["rejoin"]] = r
    incomplete = [k for k, v in pairs.items() if set(v) != {"off", "verified"}]
    if incomplete:
        sys.exit("%d episodes have no partner, e.g. %s" % (len(incomplete), incomplete[0]))
    conds = sorted({k[1] for k in pairs})

    def cell(cond, filt, rejoin):
        return [v[rejoin] for k, v in pairs.items() if k[1] == cond and k[3] == filt]

    over = lambda rs: sum(1 for r in rs if r["clearance_violations"] > 0)
    deepest = lambda rs: max([r["max_crossing"] for r in rs] + [0.0])
    falls = lambda rs: sum(1 for r in rs if r["stack_mode"] in FALLBACK)
    same = lambda rs: sum(1 for r in rs if r["same_as_reference"])
    bad = lambda r: r["clearance_violations"] > 0 or r["stack_mode"] in FALLBACK
    results = []

    # W1
    nom = [v for k, v in pairs.items() if k[1].startswith("nominal")]
    ident = sum(1 for v in nom if v["off"]["trace_hash"] == v["verified"]["trace_hash"])
    results.append(("W1", ident == len(nom) and len(nom) > 0, "identical nominal pairs: %d of %d" % (ident, len(nom))))
    # W2
    gnss = [v for k, v in pairs.items() if k[3] in GNSS_ONLY]
    ig = sum(1 for v in gnss if v["off"]["trace_hash"] == v["verified"]["trace_hash"])
    results.append(("W2", ig == len(gnss) and len(gnss) > 0, "identical GNSS-only pairs: %d of %d" % (ig, len(gnss))))
    # W3
    worse = []
    for c in conds:
        for f in KERB:
            a, b = cell(c, f, "off"), cell(c, f, "verified")
            if over(b) > over(a) or deepest(b) > deepest(a) + 0.05 or falls(b) > falls(a):
                worse.append("%s %s: over %d (%.2f m), fallbacks %d without; over %d (%.2f m), fallbacks %d with" % (
                    c, f, over(a), deepest(a), falls(a), over(b), deepest(b), falls(b)))
    results.append(("W3", not worse, "; ".join(worse) if worse else "no kerb-filter cell crosses more often or deeper, or falls back more, with the verified re-join"))
    # W4
    acted = [v for v in pairs.values() if v["verified"]["pose_corrections"] > 0]
    ba = sum(1 for v in acted if bad(v["off"]))
    bb = sum(1 for v in acted if bad(v["verified"]))
    results.append(("W4", bb < ba, "pairs acted on: %d; over a kerb or a fallback stop: %d without, %d with" % (len(acted), ba, bb)))
    # W5
    lost = [c for c in conds if same(cell(c, "ekf_kerb2", "verified")) < same(cell(c, "ekf_kerb2", "off"))]
    results.append(("W5", not lost, "ekf_kerb2 conditions ending fewer episodes as the reference: %s" % (", ".join(lost) if lost else "none")))
    # W6
    k2 = [v["verified"] for k, v in pairs.items() if k[3] == "ekf_kerb2"]
    inc = [r for r in k2 if r["incursion_depth"] > 0]
    results.append(("W6", not inc, "ekf_kerb2 closure incursions with the verified re-join: %d" % len(inc)))
    for name, ok, text in results:
        print("%-3s %-6s %s" % (name, "pass" if ok else "FAILED", text))


if __name__ == "__main__":
    main()
