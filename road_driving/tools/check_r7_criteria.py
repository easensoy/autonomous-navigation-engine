"""Evaluate the predeclared R7 criteria (benchmark/PROTOCOL_R7.md, X1-X6) from a closed_loop.jsonl.

    python check_r7_criteria.py road_driving/benchmark/heldout-r7/closed_loop.jsonl

A pair is the same scenario, condition, seed and filter, driven without any re-join ("off") and with the
unique re-join ("unique"). A fallback stop is an episode whose stack ends in the fallback (still braking, or
held after it). A pair is acted on when the unique episode re-joined at least one correction. The kerb filters
are also driven with the R6 version ("verified"); that comparison is reported without a criterion.
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
    for k, v in pairs.items():
        want = {"off", "unique", "verified"} if k[3] in KERB else {"off", "unique"}
        if set(v) != want:
            sys.exit("episode set %s has %s, expected %s" % (k, sorted(v), sorted(want)))
    conds = sorted({k[1] for k in pairs})

    def cell(cond, filt, rejoin):
        return [v[rejoin] for k, v in pairs.items() if k[1] == cond and k[3] == filt]

    over = lambda rs: sum(1 for r in rs if r["clearance_violations"] > 0)
    deepest = lambda rs: max([r["max_crossing"] for r in rs] + [0.0])
    falls = lambda rs: sum(1 for r in rs if r["stack_mode"] in FALLBACK)
    same = lambda rs: sum(1 for r in rs if r["same_as_reference"])
    bad = lambda r: r["clearance_violations"] > 0 or r["stack_mode"] in FALLBACK
    results = []

    # X1
    nom = [v for k, v in pairs.items() if k[1].startswith("nominal")]
    ident = sum(1 for v in nom if v["off"]["trace_hash"] == v["unique"]["trace_hash"])
    results.append(("X1", ident == len(nom) and len(nom) > 0, "identical nominal pairs: %d of %d" % (ident, len(nom))))
    # X2
    gnss = [v for k, v in pairs.items() if k[3] in GNSS_ONLY]
    ig = sum(1 for v in gnss if v["off"]["trace_hash"] == v["unique"]["trace_hash"])
    results.append(("X2", ig == len(gnss) and len(gnss) > 0, "identical GNSS-only pairs: %d of %d" % (ig, len(gnss))))

    # X3, and the same test for the R6 version (reported without a criterion)
    def harm(rejoin):
        worse = []
        for c in conds:
            for f in KERB:
                a, b = cell(c, f, "off"), cell(c, f, rejoin)
                if over(b) > over(a) or deepest(b) > deepest(a) + 0.05 or falls(b) > falls(a):
                    worse.append("%s %s: over %d (%.2f m), fallbacks %d without; over %d (%.2f m), fallbacks %d with" % (
                        c, f, over(a), deepest(a), falls(a), over(b), deepest(b), falls(b)))
        return worse

    worse = harm("unique")
    results.append(("X3", not worse, "; ".join(worse) if worse else "no kerb-filter cell crosses more often or deeper, or falls back more, with the unique re-join"))
    # X4
    acted = [v for v in pairs.values() if v["unique"]["pose_corrections"] > 0]
    ba = sum(1 for v in acted if bad(v["off"]))
    bb = sum(1 for v in acted if bad(v["unique"]))
    results.append(("X4", bb < ba, "pairs acted on: %d; over a kerb or a fallback stop: %d without, %d with" % (len(acted), ba, bb)))
    # X5
    lost = [c for c in conds if same(cell(c, "ekf_kerb2", "unique")) < same(cell(c, "ekf_kerb2", "off"))]
    results.append(("X5", not lost, "ekf_kerb2 conditions ending fewer episodes as the reference: %s" % (", ".join(lost) if lost else "none")))
    # X6
    k2 = [v for k, v in pairs.items() if k[3] == "ekf_kerb2"]
    new = [v for v in k2 if v["unique"]["incursion_depth"] > 0 and v["off"]["incursion_depth"] <= 0]
    both = sum(1 for v in k2 if v["unique"]["incursion_depth"] > 0 and v["off"]["incursion_depth"] > 0)
    results.append(("X6", not new, "ekf_kerb2 closure incursions with the unique re-join and not without it: %d (with and without: %d)" % (len(new), both)))
    for name, ok, text in results:
        print("%-3s %-6s %s" % (name, "pass" if ok else "FAILED", text))

    # Reported without a criterion: the R6 version on the same episodes.
    print()
    old = harm("verified")
    print("R6 re-join, the X3 test: %s" % ("; ".join(old) if old else "no kerb-filter cell crosses more often or deeper, or falls back more"))
    kp = [v for k, v in pairs.items() if k[3] in KERB]
    actv = [v for v in kp if v["verified"]["pose_corrections"] > 0]
    print("R6 re-join, kerb-filter pairs acted on: %d; over a kerb or a fallback stop: %d without, %d with" % (
        len(actv), sum(1 for v in actv if bad(v["off"])), sum(1 for v in actv if bad(v["verified"]))))
    diff = sum(1 for v in kp if v["unique"]["trace_hash"] != v["verified"]["trace_hash"])
    print("kerb-filter episodes where the unique and the R6 re-join drive differently: %d of %d" % (diff, len(kp)))


if __name__ == "__main__":
    main()
