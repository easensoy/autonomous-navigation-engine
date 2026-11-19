"""The San Francisco evaluation's verdicts and summary (benchmark/PROTOCOL_SF.md), from the campaign's records.

    python road_driving/tools/sf_analysis.py EVIDENCE_DIR

EVIDENCE_DIR holds, as the protocol's commands write them:
    sf/runs.jsonl, sf/closed_loop.jsonl              the evaluation (Cybercab-inspired proxy, SF-A and SF-B, seeds 1-15)
    sf-ipace/runs.jsonl, sf-ipace/closed_loop.jsonl  the vehicle transfer (the I-PACE proxy, the same routes, conditions and seeds)

It writes EVIDENCE_DIR/summary.json (machine-readable: every criterion's inputs, value and verdict, and the tables) and
EVIDENCE_DIR/criteria.md, and prints the verdicts. Standard library only; the same numbers on any machine.
Every run counts: failed and incomplete runs stay in every statistic, and every table names its denominator.
"""
import json
import pathlib
import statistics
import sys

ESTIMATORS = ["dr", "gnss_hold", "gnss_cv", "ekf", "ekf_int", "ekf_int2", "ekf_kerb", "ekf_kerb2"]
CONDITIONS = ["nominal_sf", "gnss_outage_sf", "gnss_drift_sf", "kerb_map_error_sf"]


def load(path):
    p = pathlib.Path(path)
    return [json.loads(line) for line in p.read_text(encoding="utf-8").splitlines() if line.strip()] if p.is_file() else []


def med(v):
    return statistics.median(v) if v else None


def pct(v, p):
    if not v:
        return None
    s = sorted(v)
    return s[min(len(s) - 1, int(p * len(s)))]


def r3(v):
    return None if v is None else round(v, 3)


def open_loop_tables(runs):
    """Per condition and estimator: runs, failed, median RMSE, pooled-free p95 of the run p95s, median along/lateral p95,
    end-of-window error (median, p95), reacquired share, NEES means; per condition the paired comparisons."""
    out = {}
    for c in CONDITIONS:
        rows = {}
        for e in ESTIMATORS:
            rs = [r for r in runs if r["condition"] == c and r["estimator"] == e]
            if not rs:
                continue
            ends = [r["fault_end_error"] for r in rs if r["fault_end_error"] >= 0.0]
            reacq = [r["reacquire"] for r in rs if r["reacquire"] >= 0.0]
            rows[e] = {"runs": len(rs), "failed": sum(1 for r in rs if r["failed"]), "rmse_median": r3(med([r["rmse"] for r in rs])),
                       "p95_median": r3(med([r["p95"] for r in rs])), "max_max": r3(max(r["max"] for r in rs)),
                       "along_p95_median": r3(med([r["along_p95"] for r in rs])), "lateral_p95_median": r3(med([r["lateral_p95"] for r in rs])),
                       "heading_rmse_deg_median": r3(med([r["heading_rmse_deg"] for r in rs])),
                       "fault_end_error_median": r3(med(ends)), "fault_end_error_p95": r3(pct(ends, 0.95)),
                       "fault_end_along_abs_median": r3(med([abs(r["fault_end_along"]) for r in rs if r["fault_end_error"] >= 0.0])),
                       "reacquired": sum(1 for v in reacq if v < 999.0), "reacquire_median_s": r3(med([v for v in reacq if v < 999.0])),
                       "nees_mean_median": r3(med([r["nees_mean"] for r in rs if r["nees_mean"] >= 0.0])),
                       "along_nees_median": r3(med([r["along_nees"] for r in rs if r["along_nees"] >= 0.0])),
                       "lateral_nees_median": r3(med([r["lateral_nees"] for r in rs if r["lateral_nees"] >= 0.0])),
                       "detect_delay_median_s": r3(med([r["detect_delay"] for r in rs if r["detect_delay"] >= 0.0])),
                       "detected": sum(1 for r in rs if r["detect_delay"] >= 0.0), "false_alarms": sum(r["false_alarms"] for r in rs),
                       "gnss_rejected": sum(r["gnss_rejected"] for r in rs), "kerb_rejected": sum(r["kerb_rejected"] for r in rs),
                       "kerb_used": sum(r["kerb_used"] for r in rs), "relocalisations": sum(r["kerb_relocalisations"] for r in rs),
                       "unique_corrections": sum(r["unique_corrections"] for r in rs), "max_correction": r3(max(r["max_correction"] for r in rs)),
                       "segment_max_lateral_median": r3(med([r["segment_max_lateral"] for r in rs if r["segment_max_lateral"] >= 0.0]))}
        out[c] = rows
    return out


def paired(runs, condition, a, b, key="fault_end_error"):
    """Pairs (route, seed): a's and b's value of `key`; how often a is lower, and the median of b - a."""
    by = {}
    for r in runs:
        if r["condition"] == condition and r["estimator"] in (a, b) and r[key] >= 0.0:
            by.setdefault((r["route"], r["seed"]), {})[r["estimator"]] = r[key]
    pairs = [(v[a], v[b]) for v in by.values() if a in v and b in v]
    return {"pairs": len(pairs), "a_lower": sum(1 for x, y in pairs if x < y), "median_b_minus_a": r3(med([y - x for x, y in pairs])),
            "a_median": r3(med([x for x, _ in pairs])), "b_median": r3(med([y for _, y in pairs]))}


def closed_tables(cl):
    """Per condition and pose source: episodes, completed, same outcome as the reference, over a kerb, deepest, smallest
    clearance, fallback stops (with reasons), localisation and clearance stops, unwarranted stops, association errors."""
    ref = {(r["scenario"], r["seed"]): r for r in cl if r["mode"] == "ground_truth"}
    out = {"reference": {"episodes": len(ref), "completed": sum(1 for r in ref.values() if r["outcome"] == "COMPLETED"),
                         "over_kerb": sum(1 for r in ref.values() if r["clearance_violations"] > 0),
                         "min_clearance": r3(min((r["min_clearance"] for r in ref.values()), default=None))}}
    for c in CONDITIONS:
        rows = {}
        for e in ("ekf_kerb2", "ekf_int2"):
            rs = [r for r in cl if r["mode"] == "estimated" and r["condition"] == c and r["estimator"] == e]
            if not rs:
                continue
            fb = [r for r in rs if r["stack_mode"] == "HOLD_AFTER_FALLBACK"]
            rows[e] = {"episodes": len(rs), "completed": sum(1 for r in rs if r["outcome"] == "COMPLETED"),
                       "same_as_reference": sum(1 for r in rs if r["same_as_reference"]), "over_kerb": sum(1 for r in rs if r["clearance_violations"] > 0),
                       "deepest": r3(max(r["max_crossing"] for r in rs)), "min_clearance": r3(min(r["min_clearance"] for r in rs)),
                       "fallback_stops": len(fb), "fallback_reasons": sorted({r["fallback_reason"].split(" ")[0] + " " + r["fallback_reason"].split(" ")[1]
                                                                              for r in fb if r["fallback_reason"].count(" ") >= 1}),
                       "localisation_stop_episodes": sum(1 for r in rs if r["localisation_stops"] > 0),
                       "clearance_stop_episodes": sum(1 for r in rs if r["clearance_stops"] > 0), "unwarranted_stops": sum(r["unwarranted_stops"] for r in rs),
                       "association_errors": sum(r["association_errors"] for r in rs), "association_samples": sum(r["association_samples"] for r in rs),
                       "pose_corrections": sum(r["pose_corrections"] for r in rs), "est_rmse_median": r3(med([r["est_rmse"] for r in rs if r["est_rmse"] >= 0.0])),
                       "est_max_max": r3(max(r["est_max"] for r in rs)), "illegal": sum(r["illegal"] for r in rs)}
        out[c] = rows
    return out


def verdicts(runs, cl, ipace_runs, ipace_cl, ol, ct):
    v = {}
    # P1, the primary transfer question: under the outage, ekf_kerb2's error when GNSS returns against dead reckoning's.
    p = paired(runs, "gnss_outage_sf", "ekf_kerb2", "dr")
    v["P1"] = {"criterion": "gnss_outage_sf: ekf_kerb2's end-of-outage error below dead reckoning's in at least 27 of 30 pairs, and its median at most 0.50 m",
               "value": p, "passed": p["pairs"] == 30 and p["a_lower"] >= 27 and p["a_median"] is not None and p["a_median"] <= 0.50}
    v["P1_other"] = {name: paired(runs, "gnss_outage_sf", name, "dr") for name in ("ekf", "ekf_int2", "ekf_kerb", "gnss_cv")}
    # P2, the nominal trade-off.
    nk = [r for r in runs if r["condition"] == "nominal_sf" and r["estimator"] == "ekf_kerb2"]
    nf = [r for r in runs if r["condition"] == "nominal_sf" and r["estimator"] in ("ekf", "ekf_int", "ekf_int2", "ekf_kerb", "ekf_kerb2") and r["failed"]]
    nees = {e: r3(med([r["nees_mean"] for r in runs if r["condition"] == "nominal_sf" and r["estimator"] == e and r["nees_mean"] >= 0.0]))
            for e in ("ekf", "ekf_int2", "ekf_kerb2")}
    p95 = r3(max(r["p95"] for r in nk)) if nk else None
    # The pooled lateral 95th percentile needs the samples, which the records do not carry; the runs' lateral 95th
    # percentiles bound it from above (the pooled value never exceeds the largest run's).
    lat95 = r3(max(r["lateral_p95"] for r in nk)) if nk else None
    v["P2"] = {"criterion": "nominal_sf: ekf_kerb2's lateral 95th percentile at most 0.10 m (bounded by its largest run's) and every run's 95th "
                            "percentile position error at most 0.50 m; no EKF run fails; median NEES (3 dof) of ekf, ekf_int2 and ekf_kerb2 within 0.5 to 6",
               "value": {"ekf_kerb2_worst_run_p95": p95, "ekf_kerb2_lateral_p95_bound": lat95, "failed_ekf_runs": len(nf), "nees_median": nees},
               "passed": bool(nk) and p95 <= 0.50 and lat95 <= 0.10 and not nf and all(x is not None and 0.5 <= x <= 6.0 for x in nees.values())}
    # P3, the closed-loop reference outcome.
    p3 = {}
    ok = True
    for c in ("nominal_sf", "gnss_outage_sf"):
        row = ct.get(c, {}).get("ekf_kerb2", {})
        p3[c] = {k: row.get(k) for k in ("episodes", "same_as_reference", "over_kerb", "fallback_stops")}
        ok = ok and row.get("episodes") == 30 and row.get("same_as_reference", 0) >= 29 and row.get("over_kerb") == 0 and row.get("fallback_stops") == 0
    v["P3"] = {"criterion": "nominal_sf and gnss_outage_sf, closed loop: ekf_kerb2 ends at least 29 of 30 episodes as the reference does, with no episode over "
                            "a kerb and no fallback stop", "value": p3, "passed": ok}
    # P4, the ambiguous kerb: the misplaced map segment, ekf_kerb2 against ekf_int2 (reported both ways; one safety criterion).
    seg = paired(runs, "kerb_map_error_sf", "ekf_kerb2", "ekf_int2", key="segment_max_lateral")
    end = paired(runs, "kerb_map_error_sf", "ekf_kerb2", "ekf_int2")
    rowk = ct.get("kerb_map_error_sf", {}).get("ekf_kerb2", {})
    rowi = ct.get("kerb_map_error_sf", {}).get("ekf_int2", {})
    v["P4"] = {"criterion": "kerb_map_error_sf: no ekf_kerb2 episode with the body over a kerb (the comparison with ekf_int2 is reported, not judged)",
               "value": {"segment_max_lateral": seg, "end_of_outage_error": end,
                         "closed_loop": {"ekf_kerb2": {k: rowk.get(k) for k in ("episodes", "completed", "fallback_stops", "over_kerb", "min_clearance")},
                                         "ekf_int2": {k: rowi.get(k) for k in ("episodes", "completed", "localisation_stop_episodes", "over_kerb")}}},
               "passed": rowk.get("episodes", 0) == 30 and rowk.get("over_kerb", 1) == 0}
    # P5, the R7-A seed 7 mechanism: along-road error left by a drift on the straight, turned lateral in the next turn.
    v["P5"] = mechanism(runs, cl)
    # P6, integrity: no false alarm under nominal GNSS.
    fa = sum(r["false_alarms"] for r in runs if r["condition"] == "nominal_sf" and r["estimator"] in ("ekf_int", "ekf_int2", "ekf_kerb", "ekf_kerb2"))
    v["P6"] = {"criterion": "nominal_sf: no integrity alarm (FAULTED or INCONSISTENT) by ekf_int, ekf_int2, ekf_kerb or ekf_kerb2", "value": {"false_alarms": fa},
               "passed": fa == 0}
    # P7, the vehicle transfer: P1 on the I-PACE proxy's own logs.
    pi = paired(ipace_runs, "gnss_outage_sf", "ekf_kerb2", "dr")
    v["P7"] = {"criterion": "the I-PACE proxy, on its own sensor logs: P1's test (at least 27 of 30 pairs, median at most 0.50 m)", "value": pi,
               "passed": pi["pairs"] == 30 and pi["a_lower"] >= 27 and pi["a_median"] is not None and pi["a_median"] <= 0.50}
    return v


def mechanism(runs, cl):
    """Open loop: how much along-road and lateral error ekf_kerb2 carries when the drift's segment ends (the turn's entry).
    Closed loop: whether those episodes then stop (fallback), cross, or complete. The judgement rule is the protocol's."""
    ol = [r for r in runs if r["condition"] == "gnss_drift_sf" and r["estimator"] == "ekf_kerb2" and r["fault_end_error"] >= 0.0]
    big = [r for r in ol if abs(r["fault_end_along"]) > 1.0 and abs(r["fault_end_lateral"]) < 0.3]
    cle = {(r["scenario"], r["seed"]): r for r in cl if r["mode"] == "estimated" and r["condition"] == "gnss_drift_sf" and r["estimator"] == "ekf_kerb2"}
    outcomes = {}
    for r in big:
        e = cle.get((r["route"], r["seed"]))
        if e:
            k = "fallback" if e["stack_mode"] == "HOLD_AFTER_FALLBACK" else ("over a kerb" if e["clearance_violations"] > 0 else e["outcome"].lower())
            outcomes[k] = outcomes.get(k, 0) + 1
    converted = [e for e in cle.values() if "lateral error" in e["fallback_reason"] or e["clearance_violations"] > 0]
    reproduced = len(big) > 0 and len(converted) > 0
    return {"criterion": "reported: SF reproduces the R7-A seed 7 mechanism if in some run ekf_kerb2 enters the turn after the drift with more than 1.0 m of "
                         "along-road error and less than 0.3 m lateral (open loop), and in closed loop a drift episode then ends in a fallback stop for "
                         "lateral error or with the body over a kerb; it reproduces the R7 failure only if one is over a kerb",
            "value": {"open_loop_runs": len(ol), "along_over_1m_lateral_under_0_3m": len(big),
                      "end_along_abs_median": r3(med([abs(r["fault_end_along"]) for r in ol])), "end_lateral_abs_median": r3(med([abs(r["fault_end_lateral"]) for r in ol])),
                      "closed_loop_outcomes_of_those": outcomes, "closed_loop_episodes": len(cle), "converted_episodes": len(converted),
                      "over_kerb": sum(1 for e in cle.values() if e["clearance_violations"] > 0)},
            "mechanism_reproduced": reproduced, "failure_reproduced": any(e["clearance_violations"] > 0 for e in cle.values()), "passed": None}


def main(argv):
    if len(argv) != 2:
        print(__doc__)
        return 2
    d = pathlib.Path(argv[1])
    runs, cl = load(d / "sf" / "runs.jsonl"), load(d / "sf" / "closed_loop.jsonl")
    ipace_runs, ipace_cl = load(d / "sf-ipace" / "runs.jsonl"), load(d / "sf-ipace" / "closed_loop.jsonl")
    ol, ct = open_loop_tables(runs), closed_tables(cl)
    oli, cti = open_loop_tables(ipace_runs), closed_tables(ipace_cl)
    v = verdicts(runs, cl, ipace_runs, ipace_cl, ol, ct)
    summary = {"format": "ane-sf-summary/1", "protocol": "road_driving/benchmark/PROTOCOL_SF.md",
               "denominators": {"open_loop_runs": len(runs), "closed_loop_episodes": len(cl), "ipace_open_loop_runs": len(ipace_runs),
                                "ipace_closed_loop_episodes": len(ipace_cl)},
               "verdicts": v, "open_loop": ol, "closed_loop": ct, "transfer": {"open_loop": oli, "closed_loop": cti}}
    (d / "summary.json").write_text(json.dumps(summary, indent=1) + "\n", encoding="utf-8")
    lines = ["# San Francisco: criteria", "", "Computed by `road_driving/tools/sf_analysis.py` from the records in this folder.", "",
             "| # | Criterion | Result | Verdict |", "|---|---|---|---|"]
    for k in ("P1", "P2", "P3", "P4", "P5", "P6", "P7"):
        x = v[k]
        verdict = "reported" if x["passed"] is None else ("passed" if x["passed"] else "**failed**")
        if k == "P5":
            verdict = ("mechanism reproduced" if x["mechanism_reproduced"] else "mechanism not reproduced") + (
                "; failure reproduced" if x["failure_reproduced"] else "; no crossing")
        lines.append("| %s | %s | `%s` | %s |" % (k, x["criterion"], json.dumps(x["value"], separators=(",", ":")), verdict))
    (d / "criteria.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    for k in ("P1", "P2", "P3", "P4", "P5", "P6", "P7"):
        x = v[k]
        print("%s %s: %s" % (k, "reported" if x["passed"] is None else ("passed" if x["passed"] else "FAILED"), json.dumps(x["value"], separators=(",", ":"))))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
