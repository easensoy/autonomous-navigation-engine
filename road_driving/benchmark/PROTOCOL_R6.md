# Held-out evaluation on R6: predeclared protocol

This file was written, committed and pushed together with the code it evaluates, before anything drove on network R6 or under any R6 sensor condition. The results are added afterwards in [`heldout-r6/`](heldout-r6), produced once by that code unchanged. The earlier evaluations are unchanged: [R2](PROTOCOL.md), [R3](PROTOCOL_R3.md), [R4](PROTOCOL_R4.md) and [R5](PROTOCOL_R5.md).

## What is evaluated

**The question.**
- The re-join after pose corrections removed a crossing on development data that followed a correct kerb re-localisation in a turn.
- Held out on R5 it showed no gain, and a small cost with GNSS-only filters. Their jumps are often corruptions, which the stack cannot tell from corrections by the jump alone.
- This evaluation asks whether a re-join limited to corrections the estimator verified against the map keeps the gain and removes the cost.

**The change.** The stack now distinguishes three responses to a pose correction (`Rejoin` in `include/road_driving/episode.hpp`):
- `off`: the stack evaluated on R3 and R4;
- `on`: the R5 version, which re-joins every jump;
- `verified`: the R6 subject.

**The verified re-join.**
- The estimator reports each kerb registration (re-localisations included) that moved its estimate by more than 0.1 m, and by how much (`StateEstimate::verifiedCorrections`).
- When the stack sees a new one, it projects the pose without that correction and measures the change in its cross-track error.
- If that change exceeds 0.15 m, the controller's reference moves by it and returns to the path along a raised cosine, over at least 10 m and 20 m per metre of correction, up to 1.5 m.
- The monitor measures from the path plus that re-join, and nothing else.
- Every other jump meets exactly the R3 and R4 stack, monitor included. A filter without a map reference therefore drives exactly as it did there.

**A matched comparison.**
- Every filter drives every episode twice on the same seeds: without any re-join (`off`) and with the verified re-join.
- The filters are those evaluated on R4 and R5, unchanged: `ekf`, `ekf_int`, `ekf_int2`, `ekf_kerb` and `ekf_kerb2`.
- The estimators only gain the report; their estimates are unchanged.

## Frozen before the R6 run

- **Code:** everything under `road_driving/` at the commit that adds this file.
- **The verified re-join:**
  - in the estimator, corrections above 0.1 m;
  - in `rejoinpolicy` in `src/stack.hpp`: above 0.15 m, at least 10 m and 20 m per metre, at most 1.5 m.
- **Everything else:** unchanged from R5, namely the estimators, `locpolicy`, `clearpolicy`, and the sensors.

## What development did

Development used R3, R4 and R5, all used before. Nothing ran on R6 except static checks: map validation, the exact footprint check of every movement, and route reachability.

**Development reruns.** The closed loops of R3 (GCC), R4 and R5 (MSVC) were rerun with the verified re-join and compared, pair by pair, with their recorded runs without any re-join:
- **GNSS-only filters:** identical in every pair (R3: 720 of 720; R4 and R5: 840 of 840 each).
- **`ekf_kerb2`:**
  - On R4 the held-out crossing is gone: under combined faults, 40 of 40 episodes ended as the reference did (before: 39), with no crossing and no fallback stop.
  - On R3, under the slow excursion: 39 of 40 as the reference (before: 36), 2 episodes over a kerb (before: 4), no fallback stop (before: 4). The 2 left are one drive, R3-B seed 2, whose re-localisation came too late.
- **Changed pairs:** 14 of 1,200 on R3, 6 of 1,400 on R4 and 1 of 1,400 on R5. In no condition does any filter cross a kerb more often or deeper, or fall back more often.
- **What it gives up against the R5 version.** On R4 the kerb filters' 6 fallback stops under kerb faults remain. They follow GNSS corrections made while the kerbs were blacked out; those are unverified, so they are left to the monitor.

**The switch** reproduces both earlier held-out closed loops, value for value, and their `closed_loop.md` byte for byte:
- the R4 run with the re-join off (1,440 episodes);
- the R5 run with it off and on (2,840 episodes).

**The checker** gives all six verdicts below on the R4 development pair, all passing (W4: 2 episodes over a kerb or in a fallback stop without the re-join, 1 with). The pairs acted on were approximated there, because that run predates the `pose_corrections` field.

**Regression.** On simulator truth nothing jumps: the 192 closure episodes are byte-identical.

## Network R6

A 4 by 4 grid at x = 0, 85, 205, 300 m and y = 0, 110, 190, 300 m:
- one-way streets J10 to J11 (northbound) and J32 to J22 (westbound);
- no left turn from J11>J21 into J21>J22, and no right turn from J22>J12 into J12>J13;
- tight junctions J22 and J01;
- a 45 m cul-de-sac east of J30.

Junction design is R2's (16 m mouths, 14 m right-turn arcs, 4 m tight kerbs). The exact footprint check passes for every movement: at least 0.023 m with mirrors, at right-turn exits.

**Routes** (as the reference planner finds them):
- R6-A: J00>J10 at 10 m to J23>J33 at 30 m, 527 m: left at J10, ahead at J11 and J12, right at J13, ahead at J23.
- R6-B: J33>J32 at 10 m to J01>J00 at 30 m, 519 m: right at J32, ahead at the tight J22 and at J12, left at J02, ahead at the tight J01.
- R6-C: J03>J13 at 10 m to J20>J30 at 30 m, 527 m: ahead at J13, right at J23, ahead at the tight J22 and at J21, left at J20.

**Conditions.** Several are designed to produce large map-verified corrections, the situation the verified re-join is for: an estimate dragged along a straight and then corrected by kerbs at a junction, or when kerbs return after a blackout.

| Condition | Definition |
|---|---|
| `nominal_r6` | as development nominal |
| `gnss_outage_r6` | no fixes over 30–70% of the route |
| `multipath_r6` | an 8 m constant offset with 1.2 m extra noise over 25–50% |
| `gnss_ramp_r6` | a faster excursion with no jump: the offset grows at 0.45 m/s inside 20–60% to 6 m, then decays |
| `kerb_return_r6` | no kerb scans over 30–60%, while the GNSS offset grows at 0.3 m/s to 4 m over the same span |
| `imu_fault_r6` | a gyro-bias step of +0.015 rad/s at 25 s; no kerb scans over 25–50% and no GNSS over 30–50% |
| `combined_r6` | outage over 10–25%; a 7 m offset with 1.0 m noise over 40–55%; a 0.3 m/s excursion to 4 m over 60–85%; 25% wheel slip over 30–40%; 10% short kerb returns; gyro-bias step +0.010 rad/s at 50 s |

**Seeds.** 1 to 10, matched across conditions, filters and both stacks.

**Closed loop.**
- Scenarios: R6-A, R6-B, R6-C, plus R6-B_late_closure, a barrier 35 m into J22>J12 notified 30 m before it.
- Each driven on ground truth (the reference), and under every R6 condition on each of the five filters, without the re-join and with the verified re-join.
- 4 x (1 + 10 x 7) x 10 = 2,840 episodes.

**Open loop**, reported without a criterion: 3 routes x 7 conditions x 10 seeds = 210 drives, each scored for 8 estimators.

**Commands.** Run once, with the MSVC 19.51 build of the frozen code. Open loop:

```bash
road_drive bench-loc --split r6 --seeds 10 --out road_driving/benchmark/heldout-r6
```

Closed loop, one process per scenario (R6-A, R6-B, R6-C and R6-B_late_closure), for example:

```bash
road_drive bench-closed --split r6 --seeds 10 --scenario R6-A --out out/r6-cl/R6-A
```

Then merged in the split's scenario order:

```bash
python road_driving/tools/merge_closed_loop.py road_driving/benchmark/heldout-r6 out/r6-cl/R6-A out/r6-cl/R6-B out/r6-cl/R6-C out/r6-cl/R6-B_late_closure
```

## Success criteria

A pair is the same scenario, condition, seed and filter, driven without and with the verified re-join. A fallback stop is an episode that ends in one. A pair is acted on when the verified episode re-joined at least one correction (`pose_corrections` in `closed_loop.jsonl`). [`tools/check_r6_criteria.py`](../tools/check_r6_criteria.py), committed with this file, computes every verdict.

| # | Criterion |
|---|---|
| W1 | Nothing changes where nothing jumps: under `nominal_r6`, every pair has identical traces (200 of 200) |
| W2 | No change without a map reference: every pair of `ekf`, `ekf_int` and `ekf_int2` has identical traces, in every condition (840 of 840) |
| W3 | No harm: in every condition, `ekf_kerb` and `ekf_kerb2` with the verified re-join have no more episodes with the body over a kerb, no crossing more than 0.05 m deeper, and no more fallback stops than without it |
| W4 | A benefit where it acts: over the pairs acted on, fewer episodes end with the body over a kerb or in a fallback stop with the verified re-join than without it |
| W5 | Completion: in every condition `ekf_kerb2` ends at least as many episodes as the reference does with the verified re-join as without it |
| W6 | Closures: no closure incursion by `ekf_kerb2` with the verified re-join |

**Expectations, stated before the run:**
- W1 and W2 hold by construction. A failure would be a defect in the implementation.
- W4 needs the targeted situation to arise. If R6's faults produce no large verified correction that the stack without the re-join handles badly, W4 fails. The evaluation then shows no harm and no benefit.
- The re-join cannot rescue a re-localisation that comes too late. On R3 one drive crossed that way.

**Reported without a criterion:**
- every other cell: errors in the open loop; crossings, stops (unwarranted ones included) and completion for every filter and stack;
- the baselines; latency and runtime.

**Analysis rules.**
- Every run counts, failed runs included, and every table states its denominator.
- Nothing is excluded or re-run after the results are seen.
- A criterion that fails is reported as failed.
