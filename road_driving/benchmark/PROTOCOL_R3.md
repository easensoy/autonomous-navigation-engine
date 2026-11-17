# Held-out evaluation on R3: predeclared protocol

This file was written, committed and pushed together with the code it evaluates, before any estimator ran on network R3 or on any R3 sensor condition. The results are added afterwards in [`heldout-r3/`](heldout-r3), produced once by that code unchanged. The first held-out evaluation, on R2, is unchanged in [`heldout/`](heldout) under [its own protocol](PROTOCOL.md). R2 then became development and regression data for this increment ([`r2-dev/`](r2-dev)).

## What is evaluated

Three estimators, all fed the same recorded sensor logs:
- **`ekf`:** the filter evaluated on R2, unchanged. It trusts GNSS again 10 s after it starts rejecting fixes.
- **`ekf_int`:** the same filter with GNSS integrity instead of the 10 s reset. A fix-to-fix jump that the odometry track cannot explain marks the receiver faulted, and its offset is remembered. Fixes are excluded until several in a row fit the filter and not that offset. Persistent disagreement with no jump is re-anchored only if the recent fixes agree with each other and with odometry, and their offset is plausible for a filter up to three times overconfident.
- **`ekf_kerb`:** `ekf_int` plus scan-to-map registration of a kerb-height range scanner on the front bumper. A robust Gauss-Newton search over position and heading is anchored by the filter's prediction. Its information matrix decides which directions a scan constrains. The scan's pose carries a floor for the kerb's correlated deviation from the map.

The baselines `dr`, `gnss_cv` and `gnss_hold` are scored as before.

The stack gains a clearance layer, used by the reference and by every estimator row except "ekf, previous stack":
- The body swept along the path ahead must keep, from every kerb, two footprint sigmas plus 0.10 m of tracking allowance. The footprint sigma is the lateral sigma combined with the heading sigma acting at the front bumper.
- Where the path is too close to a kerb, the stack shifts it sideways, by at most 0.03 m per metre travelled.
- Where no permitted shift is enough, it stops before that point and waits.

## Frozen before the R3 runs

- **Code:** everything under `road_driving/` at the commit that adds this file.
- **Estimator configuration:** the defaults of `EstimatorConfig` in `include/road_driving/localisation.hpp`:
  - unchanged from R2 for `ekf`;
  - integrity: jump gate 18.4; 5 fixes to clear a fault; 20 rejected fixes before inconsistency; re-anchoring window of 20 fixes, 0.6 m scatter, 9 times inflation;
  - kerb: 0.04 m per return; 15 independent returns per scan; floor 0.03 m and 0.004 rad per 5 m of travel; gyro-bias walk 3e-4 rad/s per root second; at least 8 matched returns; gate 16.3.
- **Policies:** in `src/stack.hpp`: `locpolicy` (unchanged from R2) and `clearpolicy`.
- **Sensors and conditions:** in `src/sensors.cpp`.

## What development did

All development used R1 and R2; nothing ran on R3 except the static footprint check of its geometry and the mechanics tests of the R3 fault generators, which run no estimator.

- **A measurement artifact.** The old road-distance function understated the clearance of points just inside a street's edge line near a rounded corner, by up to 3.6 m. The "0.001 m with mirrors" figure for R2's left turns was this artifact; the exact figure is 0.24 m. The distance is now exact: kerb lines and corner arcs, checked against the old function's sign and zero set. All 200 recorded R2 closed-loop episodes re-evaluated with it keep their traces, outcomes and body-crossing counts.
- **Where crossings happen.** On R2 they happen at right-turn exits, where the front corner swings within 0.02 m (mirrors) of the exit street's far kerb, and in the first seconds after departure, when heading is unobservable at rest. They do not happen at left turns.
- **R2 development results** ([`r2-dev/`](r2-dev), the code of this commit, GCC 14.2; 210 drives open loop):
  - `ekf` reproduces its recorded R2 results, including the 8 multipath failures.
  - `ekf_int` has no failure (largest error 0.66 m, the start-up transient every filter shares). It is identical to `ekf` wherever GNSS is clean. Under the IMU-bias step one re-anchor brings the worst lateral 95th percentile from 2.14 to 0.84 m.
  - `ekf_kerb` has a median lateral RMSE of 0.024 m against 0.135 m, and a lower RMSE than both other filters in 30 of 30 paired runs in every condition. Its NEES is 1.2–1.3, and 4.4 under the IMU-bias step.
  - Closed loop, on the R2-A and R2-B scenarios run so far: `ekf_kerb` ended every episode as the reference did, with no crossing and no stop. With the clearance layer, the GNSS-only filters stopped in every outage and combined episode (most of those stops unwarranted). `ekf_int` stopped in most multipath episodes. The full closed-loop tables follow in `r2-dev/`.

## Network R3

A 4 by 4 grid at x = 0, 110, 200, 330 m and y = 0, 85, 190, 270 m:
- one-way streets J12 to J22 and J31 to J30;
- no right turn from J11>J21 into J21>J20, or from J02>J12 into J12>J11;
- tight junctions J21 and J13;
- a 50 m cul-de-sac east of J33.

Junction design is R2's (16 m mouths, 14 m right-turn arcs, 4 m tight kerbs). The exact footprint check passes for every movement: at least 0.023 m with mirrors, at right-turn exits.

**Routes.**
- R3-A: J00>J10 at 10 m to J23>J33 at 40 m (left turn at J10, right turn at the tight J13).
- R3-B: J30>J20 at 10 m to J02>J03 at 30 m (right turn at J00).
- R3-C: J13>J12 at 10 m to J21>J31 at 30 m (left turn at J11).

**Conditions.** Every fault type except the outage is new:

| Condition | Definition |
|---|---|
| `nominal_r3` | as development nominal |
| `gnss_outage_r3` | no fixes over 35–75% of the route (R2: 50–80%) |
| `multipath_r3` | fixes shifted by an 8 m constant offset with 1.0 m extra noise over 40–70% (R2: 6 m, 55–75%) |
| `gnss_jumps_r3` | intermittent multipath: a 3 m offset with 0.5 m extra noise switching on and off every 3 s over 30–70% |
| `gnss_ramp_r3` | a slow excursion with no jump: the offset grows at 0.25 m/s inside 45–80% up to 5 m, then decays at the same rate |
| `kerb_clutter_r3` | parked cars against the opposite kerb over 15–65%; 15% of kerb returns short (spray, debris); no kerb scans over 70–80% |
| `combined_r3` | outage over 15–35%; 7 m multipath with 1.0 m noise over 55–70%; 25% wheel slip over 40–50%; parked cars over 10–50% and 10% clutter; gyro-bias step +0.010 rad/s at 35 s |

**Seeds.** 1 to 10, matched across conditions and pose sources.

**Open loop.** 3 routes x 7 conditions x 10 seeds = 210 drives, each scored for 6 estimators.

**Closed loop.**
- Scenarios: R3-A, R3-B, R3-C, plus R3-B_late_closure, a barrier 35 m into J01>J02 notified 30 m before it.
- Each driven on ground truth (the reference) and, under `nominal_r3`, `gnss_outage_r3`, `multipath_r3`, `gnss_ramp_r3`, `kerb_clutter_r3` and `combined_r3`, on "ekf, previous stack", `ekf`, `ekf_int` and `ekf_kerb`.
- 4 x (1 + 4 x 6) x 10 = 1,000 episodes.

**Commands.** Run once, with the MSVC 19.51 build of the frozen code:

```bash
road_drive bench-loc --split r3 --seeds 10 --out road_driving/benchmark/heldout-r3
```

```bash
road_drive bench-closed --split r3 --seeds 10 --out road_driving/benchmark/heldout-r3
```

## Success criteria

| # | Criterion |
|---|---|
| T1 | Integrity: no `ekf_int` or `ekf_kerb` run fails (error above 5 m) under `multipath_r3`, `gnss_jumps_r3` or `combined_r3` (0 of 90 each) |
| T2 | No cost on clean GNSS: under `nominal_r3`, `ekf_int`'s pooled 95th-percentile error is within 5% of `ekf`'s |
| T3 | Kerb accuracy: under `nominal_r3`, `ekf_kerb`'s pooled lateral 95th percentile is at most 0.10 m and at most half of `ekf`'s |
| T4 | Outage: under `gnss_outage_r3`, `ekf_kerb`'s median error when GNSS returns is at most half of `ekf`'s |
| T5 | Consistency: mean NEES (3 dof) of `ekf_int` and `ekf_kerb` between 0.5 and 6 under `nominal_r3`, `gnss_outage_r3` and `multipath_r3` |
| T6 | Kerb faults: under `kerb_clutter_r3`, no `ekf_kerb` run fails and its pooled lateral 95th percentile is at most 0.20 m |
| T7 | Closed loop, `nominal_r3`: every `ekf_kerb` episode ends as the reference does, with the body never over a kerb and no stop |
| T8 | Closed loop, every condition: the body is never over a kerb in any `ekf_kerb` episode (240); no closure incursion with any pose source |
| T9 | Closed loop: `ekf_kerb` ends as the reference does in at least 38 of 40 episodes under each of `gnss_outage_r3`, `multipath_r3` and `combined_r3` |
| T10 | Closed loop, `nominal_r3`: no unwarranted stop with `ekf_kerb` (a stop is unwarranted when the true position error at the decision was below 0.3 m) |

**Reported without a criterion:**
- the closed-loop results of "ekf, previous stack", `ekf` and `ekf_int`: crossings, stops and unwarranted stops, mission completion;
- `gnss_ramp_r3` for `ekf_int`. A slow excursion with no jump is not detectable from GNSS and odometry alone; the expectation is that `ekf_int` follows it and `ekf_kerb`'s kerbs hold the lateral part;
- the baselines; latency and runtime.

**Analysis rules.**
- Every run counts, failed runs included, and every table states its denominator.
- Nothing is excluded or re-run after the results are seen.
- A criterion that fails is reported as failed.
