# Held-out localisation evaluation: predeclared protocol

This file was written, committed and pushed together with the code it evaluates, before any estimator was run on network R2. The held-out results are added afterwards in [`heldout/`](heldout), produced once by that code unchanged. Development results are in [`dev/`](dev).

## Frozen before the held-out runs

- **Code:** everything under `road_driving/` at the commit that adds this file. That covers the sensors, estimators, stack policy, network R2 and the benchmark.
- **Estimator configuration:** the defaults of `EstimatorConfig` in `include/road_driving/localisation.hpp`:

  | Parameter | Value |
  |---|---|
  | GNSS noise | white 0.28 m, plus a slow-error state of 0.15 m with a 60 s correlation time |
  | Wheel noise | 0.03 m/s |
  | Gyro noise | 0.0015 rad/s per sample |
  | Accelerometer noise | 0.03 m/s² per sample |
  | Gyro-bias walk | 1e-4 rad/s per √s |
  | Accelerometer-bias walk | 2e-4 m/s² per √s |
  | Scale walk | 1e-5 per √s |
  | Position process noise | 0.0006 m² per metre travelled |
  | GNSS gate | 11.0 (2 degrees of freedom) |
  | Wheel gate | 10.83 |
  | Robust down-weighting | NIS above 3.0 |
  | GNSS reset | after 100 consecutive rejected fixes |
  | GNSS-only constant-velocity filter | 1.5 m/s² |
- **Localisation policy of the stack:** in `src/stack.hpp`:
  - slow to 5 m/s above a lateral 1-sigma of 0.35 m or a longitudinal one of 1.5 m;
  - stop above 0.75 m lateral or 3.0 m longitudinal, or if the estimate is invalid, stale (over 0.25 s) or has no covariance;
  - resume after 2 s below 0.30 m lateral.
- **Sensor conditions:** in `src/sensors.cpp`, listed below.

## What development did

All tuning used network R1 (routes R1-A, R1-B, R1-C) and development seeds; no estimator saw R2.

**Configuration sweep.** Fourteen configurations were tried on seeds 1 and 2: the gyro-bias walk, the robust threshold, the gate and the position process noise. The selection rule, declared before the last rounds, was the lowest worst-condition pooled 95th percentile with nominal NEES between 1 and 5.

**Closed-loop development check.** This check overruled the open-loop choice of position process noise. That noise is per metre travelled, and a time-based value kept the car stopped in every development GNSS outage. The same check set the stop thresholds above. While dead reckoning, the EKF's lateral sigma overstated its actual error about threefold.

**Seed-dependent overconfidence.** The full development run showed a NEES of 1.7 to 11.7 depending on the seed. The cause was the receiver's slowly varying error, which the filter treated as white. Two Gauss-Markov error states were added, giving nominal NEES 1.92.

**Defects found and fixed during development:**
- the accelerometer reported the brake-hold command at standstill;
- dead reckoning never calibrated its gyro at standstill;
- the degraded-speed cap suppressed the speed envelope's braking for curves.

**Final development results** are in [`dev/summary.md`](dev/summary.md) (90 drives, 360 scored runs) and [`dev/closed_loop.md`](dev/closed_loop.md) (80 episodes).

## Held-out evaluation

**Network R2.** A 4 by 3 grid with blocks of 90, 150 and 110 m by 70 and 100 m, two one-way streets, two banned turns and two tight junctions. Its geometry was fixed using the static footprint check alone (`road_drive map R2`), with no estimator data:
- junction mouths of 16 m;
- right-turn arcs of 14 m;
- tight junctions with 4 m kerbs and 5.5 m left-turn arcs at 2.5 m/s.

**Routes.**
- R2-A: J01>J11 at 10 m to J31>J32 at 40 m.
- R2-B: J30>J20 at 10 m to J01>J02 at 30 m.
- R2-C: J02>J12 at 10 m to J11>J01 at 30 m.

**Conditions.** Different placements and magnitudes from development, plus one combination:

| Condition | Definition |
|---|---|
| `nominal_h` | as development nominal |
| `gnss_outage_h` | no fixes over 50–80% of the route (development: 35–60%) |
| `multipath_h` | fixes shifted by a 6 m constant offset with 1.5 m extra noise over 55–75% (development: 4 m, 1.0 m, 30–45%) |
| `wheel_slip_h` | wheel speed reads 30% high over 40–60% (development: 20%, 25–40%) |
| `imu_bias_h` | gyro bias 0.016 rad/s, accelerometer bias 0.20 m/s², gyro-bias step +0.010 rad/s at 45 s (development: 0.012, 0.15, +0.008 at 30 s) |
| `delay_drop_h` | GNSS latency 0.50 ± 0.12 s, 40% of fixes dropped (development: 0.40 ± 0.10 s, 30%) |
| `combined_h` | outage over 20–40%; 5 m multipath with 1.2 m noise over 60–75%; 25% wheel slip over 45–55%; GNSS latency 0.30 s with 15% dropped |

**Seeds.** 1 to 10, matched. A seed fixes the start jitter and the sensor-noise realisation in every condition, and every estimator processes the identical log.

**Open loop.** 3 routes x 7 conditions x 10 seeds = 210 drives, each scored for the 4 estimators: 840 runs.

**Closed loop.**
- Scenarios: R2-A, R2-B and R2-C, plus R2-A_late_closure, a barrier 30 m into J31>J32 notified when the car is 30 m before it.
- Each driven on ground truth (the reference) and on the EKF under `nominal_h`, `gnss_outage_h`, `multipath_h` and `combined_h`.
- Seeds 1 to 10: 200 episodes.

**Commands.** Run once, with the MSVC 19.51 build of the frozen code:

```bash
road_drive bench-loc --split heldout --seeds 10 --out road_driving/benchmark/heldout
```

```bash
road_drive bench-closed --split heldout --seeds 10 --out road_driving/benchmark/heldout
```

## Success criteria

| # | Criterion | Development value |
|---|---|---|
| S1 | No EKF run fails (position error above 5 m after initialising) | 0 of 90 |
| S2 | In every condition, the EKF's RMSE is lower than each baseline's in at least 27 of the 30 paired runs | 15 of 15 in every condition |
| S3 | `nominal_h`: EKF pooled 95th-percentile position error at most 0.50 m | 0.37 m |
| S4 | `gnss_outage_h`: EKF median error when GNSS returns at most half of dead reckoning's, and median recovery to below 0.5 m within 5 s | 0.51 against 4.26 m; 0.1 s |
| S5 | `multipath_h`: EKF pooled 95th-percentile error at most 1.0 m | 0.47 m (4 m offset; held out uses 6 m) |
| S6 | EKF mean NEES (x, y, heading; 3 dof) between 0.5 and 6 under `nominal_h`, `gnss_outage_h`, `wheel_slip_h` and `delay_drop_h` | 1.9 in each |
| S7 | Closed loop, EKF under `nominal_h`: every episode reaches the same outcome as the ground-truth reference with the same seed, with no lane-association error | 20 of 20, no errors |
| S8 | Closed loop: no closure incursion in any EKF episode | 0 |

**Reported without a criterion:**
- NEES under `multipath_h`, `imu_bias_h` and `combined_h`. In development the filter was overconfident after a gyro-bias step, with NEES 27.
- Body off the carriageway and the minimum clearance. In development, estimate error put the body up to 0.18 m over R1's kerb-tight left turns.
- Localisation stops, and measurement-processing latency.

**Analysis rules.**
- Every run counts, failed runs included, and every table states its denominator.
- Nothing is excluded or re-run after the results are seen.
- A criterion that fails is reported as failed.
