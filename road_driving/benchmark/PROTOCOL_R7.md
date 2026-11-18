# Held-out evaluation on R7: predeclared protocol

This file was written, committed and pushed together with the code it evaluates, before anything drove on network R7 or under any R7 sensor condition. The results are added afterwards in [`heldout-r7/`](heldout-r7), produced once by that code unchanged. The earlier evaluations are unchanged: [R2](PROTOCOL.md), [R3](PROTOCOL_R3.md), [R4](PROTOCOL_R4.md), [R5](PROTOCOL_R5.md) and [R6](PROTOCOL_R6.md).

## What is evaluated

**The question.**
- On R6 the stack re-joined only corrections the estimator verified against the map. It kept the gain where the kerbs corrected a drifted estimate, and changed nothing for filters without a map reference.
- It failed W3. After long dead reckoning, the first kerb filter, `ekf_kerb`, twice reported a wrong kerb match as a verified correction. The re-join followed each one, and the crossings deepened from 3.64 to 8.94 m and from 6.24 to 11.21 m.
- This evaluation asks whether counting only unambiguous kerb matches removes that harm and keeps the gain.

**The change.** The estimator also reports, separately, the corrections whose kerb scan fits the map uniquely (`StateEstimate::uniqueCorrections`). The test is the one the kerb re-localisation already applies:
- Every other return of the scan is scored. A return counts when it ends within 0.3 m of a mapped kerb.
- At least 10 returns must count at the pose, and at least half of those scored.
- The pose must beat by 4 counted returns every pose on a grid around it (offsets along and across its heading up to 6 m, at 0.5 m) that is 1.5 m or more across its heading.
- Along a straight kerb every offset along it fits equally well, so only the offset across can be unique. That is also the part of a correction the stack follows.

A re-localisation by `ekf_kerb2` has passed this test in its search, around the filter's pose. Every other registration is tested at the registered pose (`kerbUnique` in `src/estimators.cpp`).

A fourth stack setting, `unique`, re-joins only those corrections. In every other respect it is the R6 re-join, `verified`, which stays available unchanged. The estimates themselves are unchanged.

**Why not the filter's own uncertainty.** A match could instead count when the filter's covariance rules out the neighbouring kerb. In the R6 gyro-fault episode the filter reported a position sigma of 0.50 m while it was 3.72 m off. Its covariance would have ruled out the true pose and accepted the wrong one. The uniqueness test uses only the scan and the map.

**A matched comparison.**
- Every filter drives every episode twice on the same seeds: without any re-join (`off`) and with the unique re-join.
- `ekf_kerb` and `ekf_kerb2` also drive each episode with the R6 version (`verified`). That comparison is reported without a criterion.
- The filters are those evaluated on R4 to R6, unchanged: `ekf`, `ekf_int`, `ekf_int2`, `ekf_kerb` and `ekf_kerb2`.

## Frozen before the R7 run

- **Code:** everything under `road_driving/` at the commit that adds this file.
- **The uniqueness test:** 0.3 m; at least 10 returns and half of those scored; a grid of plus or minus 6 m at 0.5 m; 1.5 m across; a margin of 4 returns. A re-localisation counts as tested by its search.
- **The re-join:** as on R6: corrections above 0.1 m in the estimator; in `rejoinpolicy`, above 0.15 m, at least 10 m and 20 m per metre, at most 1.5 m.
- **Everything else:** unchanged from R6, namely the estimators, `locpolicy`, `clearpolicy`, and the sensors.

## What development did

Development used R3, R4, R5 and R6, all used before. Nothing ran on R7 except static checks: map validation, the exact footprint check of every movement, and route reachability.

**R6's two failures.** Both wrong corrections fail the test at its minimum count. The scans had only 17 and 19 returns, and 8 and 6 of the scored ones ended within 0.3 m of a mapped kerb, short of the 10 required. With the unique re-join both drives (`ekf_kerb`; R6-A under the gyro fault, seed 9, and R6-B under combined faults, seed 2) are identical to the drives without any re-join: crossings of 6.24 and 3.64 m instead of 11.21 and 8.94 m.

**A first version, revised on development data.** The first version tested every correction at the registered pose, re-localisations included. On R6 it rejected three re-localisations by `ekf_kerb2` that the search had found unique, all of them correct: at the refined pose each missed the margin by one return (11 against 8). Those drives ended in fallback stops where the R6 re-join completed them (R6-B with the kerbs returning, seeds 3 and 9), and so did R3-B under the slow excursion, seed 2. A re-localisation now counts as tested by its own search.

**Development reruns.** The closed loops of R3 (GCC), R4, R5 and R6 (MSVC) were rerun with the unique re-join and compared, pair by pair, with their recorded runs without any re-join:
- **GNSS-only filters:** identical in every pair (R3: 720 of 720; R4, R5 and R6: 840 of 840 each).
- **No kerb-filter cell** crosses a kerb more often or deeper, or ends in more fallback stops, on any of the four networks (the X3 test below).
- **R6, where the R6 re-join failed:**
  - With the kerbs returning, `ekf_kerb2` ended 26 of 40 episodes as the reference did (without the re-join: 16), with 12 fallback stops (22): the R6 re-join's gain, kept.
  - 49 pairs acted on: 43 over a kerb or in a fallback stop without the re-join, 40 with.
  - What it gives up against the R6 re-join: two shallow crossings by `ekf_kerb` with the kerbs returning (0.05 and 0.08 m) that the R6 re-join had removed. And under the gyro fault `ekf_kerb2`'s deepest crossing stays at 6.24 m. The R6 re-join had brought it to 3.72 m, by following the same wrong 1.38 m match and then a correction that is not unique either.
- **R4:** under combined faults `ekf_kerb2` ended 40 of 40 episodes as the reference did, with no crossing (without the re-join: 39, and the held-out crossing of 0.91 m), as with the R6 re-join.
- **R3:** under the slow excursion `ekf_kerb2` ended 39 of 40 episodes as the reference did (without the re-join: 36), with 2 episodes over a kerb (4) and no fallback stop (4), as with the R6 re-join. The 2 left are one drive, R3-B seed 2, whose re-localisation came too late.
- **R5:** one pair changed, with the same outcome.

**The checker** gives all six verdicts on R6, rearranged into R7's layout (the recorded runs without and with the R6 re-join, plus the unique rerun): all six pass. On the same data it reproduces the R6 re-join's W3 failure.

**The switch** still reproduces every earlier held-out closed loop value for value, and its `closed_loop.md` byte for byte: R4 with the re-join off (1,440 episodes), R5 off and on, and R6 off and verified (2,840 episodes each).

**Regression.** On simulator truth nothing jumps: the 192 closure episodes are byte-identical. `road_driving_localisation_tests` has 185 checks and passes with MSVC 19.51 and GCC 14.2.

**Reproducing the reruns**, for example on R6:

```bash
road_drive bench-closed --split r6 --seeds 10 --scenario R6-A --rejoin unique --out out/unique/r6-R6-A
```

## Network R7

A 5 by 4 grid at x = 0, 90, 185, 270, 365 m and y = 0, 100, 195, 290 m:
- one-way streets J11 to J21 (eastbound) and J33 to J32 (southbound);
- no right turn from J12>J22 into J22>J21, and no left turn from J30>J31 into J31>J21;
- tight junctions J23 and J41;
- a 40 m cul-de-sac east of J42.

Junction design is R2's (16 m mouths, 14 m right-turn arcs, 4 m tight kerbs). The exact footprint check passes for every movement: at least 0.023 m with mirrors, at right-turn exits.

**Routes** (as the reference planner finds them):
- R7-A: J00>J10 at 10 m to J33>J43 at 30 m, 582 m: ahead at J10, left at J20, ahead at J21 and J22, right at the tight J23, ahead at J33.
- R7-B: J43>J33 at 10 m to J01>J00 at 30 m, 583 m: ahead at J33, the tight J23 and J13, left at J03, ahead at J02 and J01.
- R7-C: J03>J13 at 10 m to J40>J30 at 30 m, 680 m: ahead at J13, the tight J23 and J33, right at J43, ahead at J42 and the tight J41, right at J40.

**Conditions.** As on R6, several are designed to produce large kerb corrections: some correct, when kerbs return after a blackout during which GNSS drifted, and some at risk of being wrong, after long dead reckoning with a gyro fault. One adds a kerb map error, the case a check against the map cannot catch.

| Condition | Definition |
|---|---|
| `nominal_r7` | as development nominal |
| `gnss_outage_r7` | no fixes over 25–65% of the route |
| `multipath_r7` | a 7 m constant offset with 1.0 m extra noise over 35–60% |
| `kerb_return_r7` | no kerb scans over 35–65%, while the GNSS offset grows at 0.35 m/s to 5 m over the same span |
| `imu_fault_r7` | a gyro-bias step of +0.018 rad/s at 20 s; no kerb scans over 20–50% and no GNSS over 25–50% |
| `kerb_faults_r7` | the kerb on the scanner's left lies 0.5 m further out than mapped over 15–35%; no kerb scans over 45–70%, while the GNSS offset grows at 0.3 m/s to 4 m over the same span |
| `combined_r7` | outage over 10–25%; a 6 m offset with 1.0 m noise over 40–55%; a 0.3 m/s excursion to 4 m over 60–85%; 25% wheel slip over 30–40%; parked cars over 50–70%; 10% short kerb returns; gyro-bias step +0.012 rad/s at 45 s |

**Seeds.** 1 to 10, matched across conditions, filters and stacks.

**Closed loop.**
- Scenarios: R7-A, R7-B, R7-C, plus R7-B_late_closure, a barrier 35 m into J23>J13 notified 30 m before it.
- Each driven on ground truth (the reference), and under every R7 condition on each of the five filters, without the re-join and with the unique re-join, and on the two kerb filters with the R6 re-join.
- 4 x (1 + 12 x 7) x 10 = 3,400 episodes.

**Open loop**, reported without a criterion: 3 routes x 7 conditions x 10 seeds = 210 drives, each scored for 8 estimators.

**Commands.** Run once, with the MSVC 19.51 build of the frozen code. Open loop:

```bash
road_drive bench-loc --split r7 --seeds 10 --out road_driving/benchmark/heldout-r7
```

Closed loop, one process per scenario (R7-A, R7-B, R7-C and R7-B_late_closure), for example:

```bash
road_drive bench-closed --split r7 --seeds 10 --scenario R7-A --out out/r7-cl/R7-A
```

Then merged in the split's scenario order:

```bash
python road_driving/tools/merge_closed_loop.py road_driving/benchmark/heldout-r7 out/r7-cl/R7-A out/r7-cl/R7-B out/r7-cl/R7-C out/r7-cl/R7-B_late_closure
```

## Success criteria

A pair is the same scenario, condition, seed and filter, driven without any re-join and with the unique re-join. A fallback stop is an episode that ends in one. A pair is acted on when the unique episode re-joined at least one correction (`pose_corrections` in `closed_loop.jsonl`). [`tools/check_r7_criteria.py`](../tools/check_r7_criteria.py), committed with this file, computes every verdict.

| # | Criterion |
|---|---|
| X1 | Nothing changes where nothing jumps: under `nominal_r7`, every pair has identical traces (200 of 200) |
| X2 | No change without a map reference: every pair of `ekf`, `ekf_int` and `ekf_int2` has identical traces, in every condition (840 of 840) |
| X3 | No harm: in every condition, `ekf_kerb` and `ekf_kerb2` with the unique re-join have no more episodes with the body over a kerb, no crossing more than 0.05 m deeper, and no more fallback stops than without it |
| X4 | A benefit where it acts: over the pairs acted on, fewer episodes end with the body over a kerb or in a fallback stop with the unique re-join than without it |
| X5 | Completion: in every condition `ekf_kerb2` ends at least as many episodes as the reference does with the unique re-join as without it |
| X6 | Closures: no closure incursion by `ekf_kerb2` with the unique re-join that does not also occur without it |

X6 differs from R6's W6, which counted every incursion. W6 failed on an incursion that was identical without the re-join, so it measured the estimator, not the re-join. X6 counts only incursions the re-join could have caused; the checker also reports the others.

**Expectations, stated before the run:**
- X2 holds by construction: a filter without a map reference reports no kerb correction. A failure would be a defect in the implementation. X1 held on R3 to R6.
- X3 is what the uniqueness test is for. It can still fail in two ways:
  - a wrong match that is unique, as under the kerb map error, which no check against the map can detect;
  - a correct correction whose re-join ends worse than the stack without it.
- X4 needs the targeted situation to arise: a large unique correction that the stack without the re-join handles badly. `kerb_return_r7` is designed to produce it. If it does not arise, X4 fails, and the evaluation shows no harm and no benefit.
- The re-join cannot rescue a drive whose estimate is metres off before the kerbs return. On R6, under the gyro fault, no kerb-filter episode ended as the reference did, with or without the re-join.

**Reported without a criterion:**
- the R6 re-join on the same episodes: the X3 test, and the pairs it acts on;
- every other cell: errors in the open loop; crossings, stops (unwarranted ones included) and completion for every filter and stack;
- the baselines; latency and runtime.

**Analysis rules.**
- Every run counts, failed runs included, and every table states its denominator.
- Nothing is excluded or re-run after the results are seen.
- A criterion that fails is reported as failed.
