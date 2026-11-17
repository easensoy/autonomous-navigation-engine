# Held-out evaluation on R4: predeclared protocol

This file was written, committed and pushed together with the code it evaluates, before any estimator ran on network R4 or on any R4 sensor condition. The results are added afterwards in [`heldout-r4/`](heldout-r4), produced once by that code unchanged. The earlier evaluations are unchanged:
- R2, under [its protocol](PROTOCOL.md), in [`heldout/`](heldout);
- R3, under [its protocol](PROTOCOL_R3.md), in [`heldout-r3/`](heldout-r3).

R3 has since become development data for this increment.

## What is evaluated

The R3 evaluation left two failures. This increment adds a second version of the filter behind each:
- **T1.** Under combined faults `ekf_int` failed 20 of 30 runs. Once the filter had drifted, the fault state had no way out.
- **T8.** A slow GNSS excursion dragged `ekf_kerb` along the road. The kerb scans that then disagreed with it were rejected, and in closed loop the car crossed kerbs.

Five filters are compared, all on the same recorded sensor logs, plus the baselines `dr`, `gnss_cv` and `gnss_hold`:
- **`ekf`, `ekf_int`, `ekf_kerb`:** as evaluated on R3, unchanged (see Regression below).
- **`ekf_int2`:** `ekf_int` with a way out of a fault. Three changes:
  - A receiver that jumps back by most of the onset jump, or returns after a gap of more than 2 s no longer fitting the remembered offset, may have recovered. The fault ends only when the next 20 fixes look like a clean receiver: no jump, and receiver-noise scatter. Until then it stands.
  - The disagreement that remains is then tested as an inconsistency, as in `ekf_int`. It may also be re-anchored when the receiver's track and the filter's predicted track differ in direction by more than 0.07 rad over at least 10 m. A drifted heading shows as such an angle; a constant receiver offset does not.
  - Fixes that clear a fault in the first version's way must also be free of jumps.
- **`ekf_kerb2`:** `ekf_kerb` with the same GNSS changes, plus kerb re-localisation. After 3 kerb scans in a row are rejected, a grid search within 6 m of the filter's pose looks for the pose the scan fits:
  - A match unique in every direction (a corner, a junction mouth) is adopted.
  - A match unique only across the road (a straight kerb fits any offset along it) is adopted across, and the filter's position along the road is kept.
  - The filter's covariance is widened by the correction, and the receiver, which agreed with the rejected pose, must earn trust again.

Where nothing goes wrong, each second version is its first version exactly.

The stack, its clearance layer and the localisation policy are unchanged from R3.

## Frozen before the R4 runs

- **Code:** everything under `road_driving/` at the commit that adds this file.
- **Estimator configuration:** the defaults of `EstimatorConfig` in `include/road_driving/localisation.hpp`.
  - Unchanged for the first versions.
  - Second versions: heading evidence 0.07 rad over tracks of at least 10 m. A jump back must come within 75% of the onset jump, then 20 fixes with no jump and at most 0.6 m scatter.
  - Re-localisation:
    - starts after 3 rejected scans, at most every 0.5 s;
    - searches ±6 m in 0.5 m steps, then heading offsets of ±0.02 and ±0.04 rad;
    - a return counts within 0.3 m of a mapped kerb;
    - the best match must count at least 10 returns and half of those scored, beat the filter's own pose by 5, and beat every pose 1.5 m or more away (across only, on a straight) by 4.
- **Policies:** `locpolicy` and `clearpolicy` in `src/stack.hpp`, unchanged.
- **Sensors and conditions:** in `src/sensors.cpp`.

## What development did

Development used R1 to R3. Nothing ran on R4 except static checks: map validation, the exact footprint check of every movement, and route reachability. The kerb map error's mechanics test runs on R1 and runs no estimator.

**Regression.**
- On 7 stored logs, the R3 held-out logs among them, the traces of `ekf`, `ekf_int`, `ekf_kerb` and the baselines are byte-identical to those of the code evaluated on R3: 42 traces, both GCC builds.
- On simulator truth the 192 closure episodes are byte-identical.

**R3 development results** ([`r3-dev/`](r3-dev); the code of this commit, GCC 14.2; 210 drives open loop):
- **`combined_r3`:** `ekf_int2` leaves the fault in all 20 runs where `ekf_int` stayed faulted. Its largest errors are 6.9–11.2 m instead of 28.4–44.1 m, and its median-run RMSE 2.07 m instead of 10.36 m. It still exceeds 5 m in those 20 runs: the drift while GNSS is rightly excluded after a gyro-bias step.
- **`gnss_ramp_r3`:** `ekf_kerb2` re-localises 5 times, all on R3-B. Its pooled lateral 95th percentile is 0.045 m instead of 1.378 m, its largest error 1.60 m instead of 4.98 m, and its NEES 2.1 instead of 34.
- **Everywhere else** the second versions equal their first versions run for run.
- **Closed loop** (1,240 episodes, with the clearance layer):
  - Under `gnss_ramp_r3`, `ekf_kerb2` ended 36 of 40 episodes as the reference did (`ekf_kerb`: 33). Its body was over a kerb in 4 (`ekf_kerb`: 8), with one closure incursion (`ekf_kerb`: 3) and no stop (`ekf_kerb`: 4). All 4 crossings come from two drives, R3-B seeds 2 and 9, each in both R3-B scenarios.
  - Under every other condition `ekf_kerb2` equals `ekf_kerb`: 40 of 40 as the reference, no crossing, no stop.
  - `ekf_int2` equals `ekf_int` in every condition. Under combined faults the GNSS-only filters stop for clearance inside the outage, so the multipath, and `ekf_int2`'s way out of it, never comes.

**What did not work on R3, and what changed because of it:**
- **A jump back taken alone** ended a fault early. With 1 m of multipath noise, chance jump-backs came inside the fault, and three biased fixes that happened to agree with the drifted filter were trusted (R3-B, seed 9: 40 m). Hence the 20-fix confirmation.
- **A strict jump-back test** (within half the onset jump) missed the real end of a fault whose onset jump was itself noisy (R3-A, seed 1). Hence 75%.
- **The first version's fault exit** accepted 5 multipath fixes that happened to fit a drifted filter (R3-A, seed 6: 30.9 m). Hence jump-free exit fixes.
- **A shorter re-anchoring window** (5 or 10 fixes instead of 20) re-anchored onto multipath fixes (17–24 m).
- **Re-localisation that demanded a match unique in every direction** found one only at junctions; in closed loop that came too late. Hence the across-only match on straights.

## Network R4

A 5 by 3 grid at x = 0, 80, 170, 260, 380 m and y = 0, 100, 180 m:
- one-way streets J21 to J31 (eastbound) and J02 to J01 (southbound);
- no left turn from J01>J11 into J11>J12, and no right turn from J22>J32 into J32>J31;
- tight junctions J21 and J40;
- a 40 m cul-de-sac north of J42.

Junction design is R2's (16 m mouths, 14 m right-turn arcs, 4 m tight kerbs). The exact footprint check passes for every movement: at least 0.023 m with mirrors, at right-turn exits.

**Routes** (as the reference planner finds them):
- R4-A: J00>J10 at 10 m to J41>J42 at 30 m, 496 m: ahead at J10 and J20, left at J30, right at J31, left at J41.
- R4-B: J41>J31 at 10 m to J10>J00 at 30 m, 422 m: left at J31, right at J30, ahead at J20 and J10.
- R4-C: J02>J01 at 10 m to J22>J32 at 30 m, 353 m: left at J01, ahead at J11 (the banned left keeps it on), left at the tight J21, right at J22.

**Conditions.** New magnitudes and combinations, and one new fault type, a kerb map error:

| Condition | Definition |
|---|---|
| `nominal_r4` | as development nominal |
| `gnss_outage_r4` | no fixes over 25–70% of the route (R3: 35–75%) |
| `multipath_r4` | fixes shifted by a 10 m constant offset with 2.0 m extra noise over 35–60% (R3: 8 m, 1.0 m) |
| `gnss_ramp_r4` | a slow excursion with no jump: the offset grows at 0.4 m/s inside 25–65% up to 7 m, then decays at the same rate (R3: 0.25 m/s, 5 m) |
| `imu_fault_r4` | a gyro-bias step of +0.015 rad/s at 25 s, and a 6 m offset with 1.0 m noise over 30–55% |
| `kerb_faults_r4` | a kerb map error: over 20–45% the kerb on the scanner's left lies 0.5 m further out than mapped. No kerb scans over 55–80%, and an 8 m offset with 0.5 m noise over 60–75%, inside that blackout |
| `combined_r4` | outage over 10–25%; a 0.3 m/s excursion up to 5 m over 40–70%; 30% wheel slip over 50–60%; 10% of kerb returns short; gyro-bias step +0.008 rad/s at 40 s |

**Seeds.** 1 to 10, matched across conditions and pose sources.

**Open loop.** 3 routes x 7 conditions x 10 seeds = 210 drives, each scored for 8 estimators.

**Closed loop.**
- Scenarios: R4-A, R4-B, R4-C, plus R4-B_late_closure, a barrier 35 m into J20>J10 notified 30 m before it.
- Each driven on ground truth (the reference), and under every R4 condition on `ekf`, `ekf_int`, `ekf_int2`, `ekf_kerb` and `ekf_kerb2`, all with the clearance layer.
- 4 x (1 + 5 x 7) x 10 = 1,440 episodes.

**Commands.** Run once, with the MSVC 19.51 build of the frozen code. Open loop:

```bash
road_drive bench-loc --split r4 --seeds 10 --out road_driving/benchmark/heldout-r4
```

Closed loop, one process per scenario (R4-A, R4-B, R4-C and R4-B_late_closure), for example:

```bash
road_drive bench-closed --split r4 --seeds 10 --scenario R4-A --out out/r4-cl/R4-A
```

Then merged in the split's scenario order:

```bash
python road_driving/tools/merge_closed_loop.py road_driving/benchmark/heldout-r4 out/r4-cl/R4-A out/r4-cl/R4-B out/r4-cl/R4-C out/r4-cl/R4-B_late_closure
```

Each scenario's episodes, its reference included, run in its own process, so the merge reproduces a single process's files. Checked on R3 development data with one seed: both merged files were byte-identical to one process's (124 episodes).

## Success criteria

| # | Criterion |
|---|---|
| U1 | No change where nothing goes wrong: under `nominal_r4`, every `ekf_int2` run has `ekf_int`'s RMSE and largest error, and every `ekf_kerb2` run `ekf_kerb`'s (30 of 30 each) |
| U2 | Recovery after drift: under `imu_fault_r4`, `ekf_int2`'s pooled 95th-percentile error is at most half of `ekf_int`'s |
| U3 | Integrity: no `ekf_int2` run fails (error above 5 m) under `multipath_r4` (0 of 30) |
| U4 | Slow excursion: under `gnss_ramp_r4`, no `ekf_kerb2` run fails and its pooled lateral 95th percentile is at most 0.20 m |
| U5 | Kerb faults: under `kerb_faults_r4`, no `ekf_kerb2` run fails and its pooled lateral 95th percentile is at most 0.50 m (the size of the map error) |
| U6 | Consistency: mean NEES (3 dof) of `ekf_int2` and `ekf_kerb2` between 0.5 and 6 under `nominal_r4`, `gnss_outage_r4` and `multipath_r4` |
| U7 | No regression, open loop: in every condition `ekf_int2` fails no more runs than `ekf_int`, and `ekf_kerb2` no more than `ekf_kerb` |
| U8 | Closed loop, `nominal_r4`: every `ekf_kerb2` episode ends as the reference does, with the body never over a kerb and no stop |
| U9 | Closed loop, every condition: the body is never over a kerb in any `ekf_kerb2` episode (280); no closure incursion with any pose source |
| U10 | Closed loop: `ekf_kerb2` ends as the reference does in at least 38 of 40 episodes under each of `gnss_outage_r4`, `multipath_r4`, `gnss_ramp_r4` and `combined_r4` |
| U11 | No regression, closed loop: in every condition, `ekf_int2` has no more episodes with the body over a kerb than `ekf_int`, and `ekf_kerb2` no more than `ekf_kerb` |

**Expectations from development, stated before the run:**
- U9 and U10 may fail under `gnss_ramp_r4`. On R3, `ekf_kerb2` still crossed in 4 of 40 ramp episodes, from two R3-B drives:
  - In one, the only unique match came mid-turn, too late.
  - In the other, a 1.2 m correction made mid-turn started a steering oscillation that the stack's monitor ended with a fallback stop, over the kerb.
- GNSS-only filters cannot bound dead-reckoning drift while GNSS is rightly excluded. Under `imu_fault_r4`, `ekf_int2` is expected to exceed 5 m in some runs before it recovers.

**Reported without a criterion:**
- every other cell of the tables: errors, consistency, crossings, stops (unwarranted ones included) and mission completion for every pose source;
- the baselines; latency and runtime.

**Analysis rules.**
- Every run counts, failed runs included, and every table states its denominator.
- Nothing is excluded or re-run after the results are seen.
- A criterion that fails is reported as failed.
