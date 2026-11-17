# Held-out evaluation on R5: predeclared protocol

This file was written, committed and pushed together with the code it evaluates, before anything drove on network R5 or under any R5 sensor condition. The results are added afterwards in [`heldout-r5/`](heldout-r5), produced once by that code unchanged. The earlier evaluations are unchanged: [R2](PROTOCOL.md), [R3](PROTOCOL_R3.md) and [R4](PROTOCOL_R4.md), in [`heldout/`](heldout), [`heldout-r3/`](heldout-r3) and [`heldout-r4/`](heldout-r4).

## What is evaluated

**The question.** The R4 evaluation failed one criterion (U9) through the driving stack, not the estimate. After a correct 0.8 m re-localisation in the middle of a turn, pure pursuit oscillated until the monitor's fallback stop, and the body went 0.91 m over the kerb. The stack then gained a re-join after pose corrections, developed on R3 and R4. This evaluation asks whether it helps on roads and faults it was not developed on, and whether it costs anything.

**The change** (`rejoinpolicy` in `src/stack.hpp`):
- **Detection.** A cross-track error that changes by more than 0.15 m between two control ticks (20 ms) is a pose correction, not motion.
- **Re-join.** The controller's reference moves by that change and returns to the path along a raised cosine, over at least 10 m and 20 m per metre of correction. Corrections above 1.5 m are not re-joined.
- **Monitor.** It measures its 1 m limit from the path plus the clearance shift (before: the unshifted path). It does not use the re-join, so a jump of more than 1 m still stops the car.

**A matched comparison.** Every filter drives every episode twice, on the same seeds: once with the stack evaluated on R3 and R4 ("no re-join"), and once with the re-join. The filters are those evaluated on R4, unchanged: `ekf`, `ekf_int`, `ekf_int2`, `ekf_kerb` and `ekf_kerb2`.

## Frozen before the R5 run

- **Code:** everything under `road_driving/` at the commit that adds this file.
- **The re-join:** `rejoinpolicy` in `src/stack.hpp`: 0.15 m per control tick; at least 10 m, 20 m per metre; at most 1.5 m.
- **Everything else:** unchanged from R4, namely the estimators and their configuration, `locpolicy`, `clearpolicy`, and the sensors.

## What development did

Development used R3 and R4, both used before. Nothing ran on R5 except static checks: map validation, the exact footprint check of every movement, and route reachability.

- **Development reruns** ([`rejoin-dev/`](rejoin-dev)):
  - On R4, `ekf_kerb2` crossed a kerb in none of 280 episodes (held-out run: 1), and its kerb-fault fallback stops fell from 6 to 2.
  - On R3, its slow-excursion crossings fell from 4 to 2. Those 2 are one drive whose re-localisation came too late.
  - No filter crossed more often or deeper, in any condition.
- **A rejected first version.** The first version also hid the re-join from the monitor. It drove the timed-reset `ekf`, which had snapped onto multipath fixes, 9.3 m off the road; the old monitor had stopped it at that jump. Hence the monitor without the re-join.
- **The switch.** Each split runs the stack it was evaluated with, so the re-join is off in every earlier split, and `--rejoin on` turns it on. Both recorded R4 closed loops are reproduced exactly:
  - with the re-join off, the held-out run's 1,440 episodes, value for value;
  - with it on, the development rerun's.

  Both `closed_loop.md` files are byte-identical. On that development pair the criteria below all hold: fallback stops 456 without the re-join and 330 with it, and episodes ending as the reference 673 and 708.
- **Regression.** On simulator truth nothing jumps: the 192 closure episodes are byte-identical.

## Network R5

A 3 by 5 grid at x = 0, 105, 230 m and y = 0, 75, 160, 235, 330 m:
- one-way streets J11 to J12 (northbound) and J23 to J13 (westbound);
- no left turn from J02>J12 into J12>J13, and no right turn from J13>J03 into J03>J04;
- tight junctions J11 and J23;
- a 45 m cul-de-sac north of J24.

Junction design is R2's (16 m mouths, 14 m right-turn arcs, 4 m tight kerbs). The exact footprint check passes for every movement: at least 0.023 m with mirrors, at right-turn exits.

**Routes** (as the reference planner finds them):
- R5-A: J00>J10 at 10 m to J14>J24 at 30 m, 463 m: left at J10, ahead at the tight J11, at J12 and at J13, right at J14.
- R5-B: J24>J23 at 10 m to J01>J00 at 30 m, 518 m: ahead at the tight J23, right at J22, ahead at J12, left at J02, ahead at J01.
- R5-C: J04>J03 at 10 m to J23>J13 at 30 m, 494 m: ahead at J03, left at J02, ahead at J12, left at J22, left at the tight J23.

**Conditions.** New magnitudes and combinations:

| Condition | Definition |
|---|---|
| `nominal_r5` | as development nominal |
| `gnss_outage_r5` | no fixes over 20–55% of the route |
| `multipath_r5` | a 9 m constant offset with 1.5 m extra noise over 30–55% |
| `gnss_ramp_r5` | a slow excursion with no jump, growing at 0.35 m/s to 6 m inside 30–70%, then decaying |
| `imu_fault_r5` | a gyro-bias step of +0.012 rad/s at 30 s, and an outage over 35–55% |
| `kerb_faults_r5` | the kerb on the scanner's left 0.4 m further out than mapped over 10–30%; no kerb scans over 40–70%; a 7 m offset with 0.8 m noise over 45–60%, inside that blackout |
| `combined_r5` | outage over 15–30%; a 0.3 m/s excursion to 4 m over 45–75%; 25% wheel slip over 35–45%; parked cars over 50–70% and 10% short kerb returns; gyro-bias step +0.010 rad/s at 45 s |

**Seeds.** 1 to 10, matched across conditions, filters and both stacks.

**Closed loop.**
- Scenarios: R5-A, R5-B, R5-C, plus R5-B_late_closure, a barrier 35 m into J12>J02 notified 30 m before it.
- Each driven on ground truth (the reference), and under every R5 condition on each of the five filters, with and without the re-join.
- 4 x (1 + 10 x 7) x 10 = 2,840 episodes.

**Open loop**, reported without a criterion: 3 routes x 7 conditions x 10 seeds = 210 drives, each scored for 8 estimators.

**Commands.** Run once, with the MSVC 19.51 build of the frozen code. Open loop:

```bash
road_drive bench-loc --split r5 --seeds 10 --out road_driving/benchmark/heldout-r5
```

Closed loop, one process per scenario (R5-A, R5-B, R5-C and R5-B_late_closure), for example:

```bash
road_drive bench-closed --split r5 --seeds 10 --scenario R5-A --out out/r5-cl/R5-A
```

Then merged in the split's scenario order:

```bash
python road_driving/tools/merge_closed_loop.py road_driving/benchmark/heldout-r5 out/r5-cl/R5-A out/r5-cl/R5-B out/r5-cl/R5-C out/r5-cl/R5-B_late_closure
```

## Success criteria

A pair is the same scenario, condition, seed and filter, driven without and with the re-join. A fallback stop is an episode that ends in one. [`tools/check_r5_criteria.py`](../tools/check_r5_criteria.py), committed with this file, computes every verdict from `closed_loop.jsonl`.

| # | Criterion |
|---|---|
| V1 | Nothing changes where nothing jumps: under `nominal_r5`, every pair has identical traces (200 of 200) |
| V2 | The fix: with the re-join, the body of `ekf_kerb2` is never over a kerb (0 of 280) |
| V3 | No harm: in every condition and for every filter, the re-join has no more episodes with the body over a kerb than without it, and its deepest crossing is no more than 0.05 m deeper |
| V4 | Fewer fallback stops: summed over every condition and filter, fewer episodes end in one with the re-join than without it |
| V5 | Completion: in every condition `ekf_kerb2` ends at least as many episodes as the reference does with the re-join as without it; and so do all filters together, summed over every condition |
| V6 | Closures: no closure incursion by `ekf_kerb2` with the re-join |

**Expectations from development, stated before the run:**
- The re-join cannot rescue a re-localisation that comes too late. On R3 that left 2 crossings, so V2 may fail the same way.
- A GNSS-only filter that drifts or is captured can be wrong by metres. The monitor still stops it at a jump of more than 1 m, so with the re-join some such episodes end in a fallback stop rather than completing over a kerb, with the same crossing. That is why completion is not required per filter.

**Reported without a criterion:**
- every other cell: errors in the open loop; crossings, stops (unwarranted ones included) and completion for every filter and stack;
- the baselines; latency and runtime.

**Analysis rules.**
- Every run counts, failed runs included, and every table states its denominator.
- Nothing is excluded or re-run after the results are seen.
- A criterion that fails is reported as failed.
