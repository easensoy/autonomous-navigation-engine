# Localisation in road_driving

The car can drive on its own state estimate instead of simulator truth. The world simulates GNSS, an IMU, wheel encoders and a steering-angle sensor from the truth. Estimators see only those measurements, plus the lane map for one purpose (the initial heading of a car parked in a lane). The stack can then drive on the EKF's output through an explicit interface. Ground-truth driving remains as the labelled reference.

Labels: **SIMULATED_SENSORS** and **ESTIMATED_POSE**. There is no camera, lidar or map matching in this increment; nothing here is lidar localisation.

## Frames, units and time

- **Map frame:** x east, y north, metres.
- **Body frame:** origin at the rear-axle centre, x forward, y left.
- **Angles:** radians, anticlockwise from map x, wrapped to (−π, π] after every prediction and update.
- **Time:** seconds since the episode started.
- **Measurements:** each carries its time of validity (`tMeas`) and its arrival time (`tArrival`). Estimators process measurements in arrival order and apply each at its time of validity.

## Sensors (`src/sensors.cpp`)

| Sensor | Rate | Latency | Mounting | Errors |
|---|---|---|---|---|
| GNSS position | 10 Hz | 0.10 ± 0.02 s | antenna 1.60 m ahead of the rear axle | 0.25 m white noise per axis; 0.15 m Gauss-Markov error with a 60 s time constant; 2% of fixes dropped. The receiver always reports 0.30 m accuracy: it does not detect multipath |
| IMU: longitudinal and lateral specific force, yaw rate | 100 Hz | 0 | 1.40 m ahead of the rear axle | gyro 0.0015 rad/s noise, bias ±0.0025 rad/s plus a random walk; accelerometer 0.03 m/s² noise, bias ±0.04 m/s² plus a random walk. The lever arm puts −ω²·l in the longitudinal reading |
| Wheel speed (rear axle) | 50 Hz | 0.02 s | — | ±1% scale error, 0.02 m/s noise, 0.005 m/s resolution; exactly 0 at rest |
| Steering angle | 50 Hz | 0.02 s | — | 0.002 rad noise, ±0.004 rad offset |

The signs of biases and errors are drawn per seed. Each measurement is rounded through the log's text format when it is generated, so an estimator fed live and one replayed from `sensors.csv` receive bit-identical inputs.

### Conditions

Spans are fractions of the route, turned into map rectangles around the route.

| Development | Held out |
|---|---|
| `nominal` | `nominal_h` |
| `gnss_outage`: no fixes over 35–60% of the route | `gnss_outage_h`: 50–80% |
| `multipath`: 4 m constant offset, 1.0 m extra noise, over 30–45% | `multipath_h`: 6 m, 1.5 m, over 55–75% |
| `wheel_slip`: wheel speed 20% high over 25–40% | `wheel_slip_h`: 30% high over 40–60% |
| `imu_bias`: gyro bias 0.012 rad/s, step +0.008 rad/s at 30 s | `imu_bias_h`: 0.016 rad/s, step +0.010 rad/s at 45 s |
| `delay_drop`: GNSS latency 0.40 ± 0.10 s, 30% of fixes dropped | `delay_drop_h`: 0.50 ± 0.12 s, 40% dropped |
| — | `combined_h`: outage, multipath, slip and latency together |

## Estimators (`src/estimators.cpp`)

**Common start.** Every estimator starts from the same information: its first GNSS fix, the antenna lever arm, and the heading of the lane that fix lies in.

### EKF: 9 states

| State | Why it is there |
|---|---|
| x, y (rear axle) | the pose the stack needs |
| heading | the pose the stack needs |
| speed | integrates the accelerometer; tied to the wheels |
| gyro bias | the yaw-rate input drifts |
| accelerometer bias | the acceleration input is offset |
| wheel-speed scale error | the tyre radius is not known exactly |
| slow GNSS error, x and y | the receiver's correlated error. Without these states the filter averaged it away, and its NEES ranged from 1.7 to 11.7 with the seed |

**Prediction.** The IMU drives it. Yaw rate is gyro minus bias. Speed integrates the accelerometer, corrected for the lever arm (+ω²·l) and minus its bias. Position moves along the midpoint heading. The covariance follows the Jacobian of the same equations, plus process noise:
- IMU white noise (per-sample variance × sample period × dt);
- bias random walks;
- unmodelled motion per metre travelled, so a parked car gains no position uncertainty.

**Updates.**
- Wheel speed: h = v(1 + s).
- GNSS antenna position: h = position + R(ψ)·lever arm + slow error.
- At standstill (encoders silent for 0.2 s and the gyro quiet): zero-velocity and zero-rate updates. The gyro and accelerometer then read their biases.

**Delays.** A late measurement rewinds the filter to its time of validity and replays everything after it. The replay buffer is 1 s. A test confirms that 0.4 s latency with 0.1 s jitter gives the same filter as in-order processing.

**Measurement rejection.**
- Chi-square gates: GNSS 11.0 (2 dof), wheel 10.83 (1 dof).
- Robust down-weighting: inside the gate, a GNSS fix with NIS above 3.0 has its noise scaled by NIS / 3.
- Reset: after 100 consecutive rejected fixes (10 s), GNSS is trusted again. The position uncertainty is reopened to cover the disagreement and the fix is accepted.

**Numerical safeguards.**
- Joseph-form updates, with symmetrisation and a floor on variances.
- The innovation covariance is checked to be positive definite before it is inverted.
- A non-finite state or a negative variance restarts the filter from the next fix, counted as a reset.

**Observability.**
- Heading is observable from GNSS only while moving.
- The gyro bias is observable from turning and from standstill.
- The scale error is observable from GNSS distance.
- During an outage, position and the slow GNSS error are unobservable, so their uncertainty grows. While dead reckoning, the lateral 1-sigma overstated the actual error about threefold in development.
- A step change of gyro bias is followed slowly. Under `imu_bias` the filter is overconfident (development NEES 27).

### Baselines

All three get the same log and the same start.

| Baseline | Definition |
|---|---|
| `dr` | wheel speed and gyro dead reckoning, with the gyro bias calibrated at standstill (the EKF's standstill rule). No covariance |
| `gnss_cv` | a constant-velocity Kalman filter on the antenna position, with the same gate, robust weighting and reset as the EKF. Heading from its velocity |
| `gnss_hold` | the latest fix, held, with heading from fix-to-fix motion |

## Driving on the estimate

**The interface.** In estimated mode the stack's pose is the EKF output at 50 Hz: position, heading, speed, acceleration (IMU, bias-corrected), the steering sensor, and the estimate's validity, age and position covariance. The stack never receives truth. The world records the truth and an independent evaluator scores it afterwards.

**Policy** (`src/stack.hpp`, frozen in [the protocol](benchmark/PROTOCOL.md)):
- **Slow:** cap the speed at 5 m/s when the lateral 1-sigma exceeds 0.35 m or the longitudinal exceeds 1.5 m.
- **Stop:** stop in lane at 2.5 m/s² when the estimate is invalid or stale (over 0.25 s), or has no covariance, or is above 0.75 m lateral or 3.0 m longitudinal.
- **Resume:** after 2 s below 0.30 m lateral.

There is no fallback to truth. A car that stops inside a GNSS outage waits there until the estimate recovers.

**Closed-loop metrics** (from truth):
- outcome and time to goal, and stop margin and incursion at closures;
- true cross-track error;
- whole-vehicle clearance: the footprint with mirrors against the carriageway edges, with the body counted when it crosses them;
- lane association: the stack's element against the evaluator's lane, per control tick.

## Benchmark method (`src/benchmark.cpp`)

**Open loop.** Each drive is recorded once with ground-truth driving. Every estimator then processes the same sensor log, in the same order, and is scored against the same truth in the fixed map frame. Nothing is aligned per trajectory. Metrics:
- position RMSE, median, 95th percentile and maximum;
- heading and speed error;
- relative position error over 5 s and 20 s windows;
- error when GNSS returns from the longest outage, the largest error inside it, and the time back below 0.5 m;
- failures, meaning error above 5 m;
- NEES with the share inside the 95% chi-square interval, and NIS for GNSS and wheel;
- runtime and per-measurement wall-clock latency.

Failed runs stay in every statistic.

**Replay without the simulator.**

```bash
road_drive localise <episode dir>
```

This reruns every estimator from that folder's `sensors.csv` and `truth.csv` and writes their traces. A test confirms it reproduces the in-process estimates to 1e-9 m.

## Results

Measured on 17 November 2026, freshly run.

| | Development | Held out |
|---|---|---|
| Network | R1 | R2, never used for tuning |
| Build | GCC 14.2 | MSVC 19.51, frozen commit `62a742a` |
| Seeds | 5 | 10 |
| Open-loop drives | 90 | 210 |
| Closed-loop episodes | 80 | 200 |
| Full tables | [`benchmark/dev/`](benchmark/dev) | [`benchmark/heldout/`](benchmark/heldout) |

The criteria were [predeclared](benchmark/PROTOCOL.md) and pushed in that commit before any held-out run.

### Held out, open loop: position error

The table gives median-run RMSE and pooled 95th percentile, in metres, then the number of failed runs out of 30. A run fails when its error exceeds 5 m.

| Condition | EKF | GNSS constant-velocity | GNSS hold | Dead reckoning |
|---|---|---|---|---|
| `nominal_h` | **0.18 / 0.36**, 0 | 0.62 / 1.23, 0 | 1.25 / 1.98, 0 | 4.23 / 10.5, 16 |
| `gnss_outage_h` | **0.22 / 0.45**, 0 | 7.14 / 18.7, 30 | 43.5 / 113, 30 | 4.23 / 10.5, 16 |
| `multipath_h` | 0.29 / **5.65**, **8** | 3.63 / 11.4, 30 | 2.99 / 6.89, 30 | 4.23 / 10.5, 16 |
| `wheel_slip_h` | **0.18 / 0.36**, 0 | 0.62 / 1.23, 0 | 1.25 / 1.98, 0 | 19.7 / 33.6, 30 |
| `imu_bias_h` | **0.23 / 0.47**, 0 | 0.62 / 1.23, 0 | 1.25 / 1.98, 0 | 4.51 / 11.6, 18 |
| `delay_drop_h` | **0.20 / 0.38**, 0 | 0.80 / 1.51, 0 | 4.28 / 6.91, 30 | 4.41 / 10.5, 16 |
| `combined_h` | **0.21 / 0.43**, 0 | 5.73 / 15.7, 30 | 21.3 / 60.7, 30 | 9.49 / 18.5, 30 |

**Outages.** When GNSS returns, the EKF's median error is 0.40 m, against dead reckoning's 5.57 m. The EKF is back below 0.5 m 0.01 s later (median).

**NEES** (3 dof, ideal 3):

| Condition | NEES |
|---|---|
| `nominal_h`, `wheel_slip_h` | 1.78 |
| `delay_drop_h` | 1.74 |
| `combined_h` | 1.64 |
| `gnss_outage_h` | 1.63 |
| `imu_bias_h` | 19.0 (overconfident) |
| `multipath_h` | 39.9 (overconfident) |

**Measurement-processing latency** (wall clock, on the campaign machine). Per run, the median is 0.8–2.6 µs per measurement. The 99th percentile is about 19–20 µs, rising to 54 µs under `combined_h` and 88 µs under `delay_drop_h`, where late fixes replay the buffer.

### Predeclared criteria

| # | Criterion | Held-out result | Verdict |
|---|---|---|---|
| S1 | No EKF failure | 8 of 210 runs failed, all R2-B under `multipath_h` (6.1–8.0 m) | **failed** |
| S2 | EKF better than each baseline in at least 27 of 30 paired runs, every condition | 29 or 30 of 30 everywhere | passed |
| S3 | `nominal_h` pooled 95th percentile at most 0.50 m | 0.355 m | passed |
| S4 | Outage error at most half of dead reckoning's; recovery within 5 s | 0.40 against 5.57 m; 0.01 s | passed |
| S5 | `multipath_h` pooled 95th percentile at most 1.0 m | 5.65 m | **failed** |
| S6 | NEES 0.5–6 in nominal, outage, wheel-slip and delay conditions | 1.63–1.78 | passed |
| S7 | Closed loop, `nominal_h`: same outcome as the reference, no association error | 40 of 40, no errors | passed |
| S8 | No closure incursion in any EKF episode | 0 of 160 | passed |

### Held out, closed loop: driving on the EKF

Each cell is out of 10 seeds, against the ground-truth reference with the same seed.

- **Under `nominal_h`:**
  - Every mission ends as the reference does.
  - Closure stops end at least 1.73 m from the barrier (reference: at least 1.86 m).
  - Estimate error has a median RMSE of 0.17 m.
  - In 13 of 40 episodes the body crosses a carriageway edge, and the footprint with mirrors reaches 0.24 m past it. The reference crosses in none.
  - There is little margin for estimate error. Driven exactly, R2's tightest left turn clears the edge by 0.001 m with mirrors, and its straight-ahead movements by 0.43 m.
- **Under `gnss_outage_h`:**
  - R2-B and R2-C complete.
  - On R2-A the outage lies on one 127 m straight, and the lateral 1-sigma grows with distance driven. In all 20 episodes it passes the 0.75 m stop threshold after about 107 m.
  - The car stops in lane inside the outage, about 13 m short of where GNSS returns, and waits there. Those episodes end `STOPPED_EN_ROUTE`.
  - R2-B's outage is longer (143 m) but contains a 90° turn, which turns lateral uncertainty into along-track uncertainty. Its lateral 1-sigma peaked at 0.71 m (seed 1).
- **Under `multipath_h`:**
  - 28 of 40 episodes end away from the reference outcome.
  - The 6 m offset outlasts the 10 s rejection window, so the filter resets onto the biased fixes, and the estimate error reaches 9.2 m.
  - In every episode that stopped en route, the constraint monitor saw a large lateral error from the path on the estimate and braked to a fallback stop.
  - In one episode (R2-B, seed 3) the car left the carriageway before that stop; its footprint with mirrors reached 4.36 m past the edge.
  - The filter's covariance did not flag this (NEES 39.9), so no localisation stop fired.
- **Under `combined_h`:** every mission ends as the reference does.

### Failure cases and the next improvement

**Two weaknesses** (the failure cases above):
- **Persistent multipath.** A bias longer than the 10 s reset window can capture the filter, and nothing in the covariance warns of it.
- **Lateral accuracy.** GNSS-grade lateral accuracy (0.2–0.4 m) is not enough for kerb clearance on 3 m lanes, nor for keeping lateral uncertainty bounded through a long outage on a straight.

**The next improvement the evidence supports** is a measured lateral reference that does not depend on GNSS. This means lane-marking or kerb detection from simulated camera or lidar measurements, matched to the map, as a further EKF measurement. It targets all three failure modes: multipath capture, outage stops and kerb clearance. A plausibility check of GNSS jumps against odometry at the onset of multipath is the smaller, immediate step.

## Not done

- **UKF:** not implemented. The EKF's nominal NEES is near its ideal and the motion is gentle, so no question remains that a UKF would answer here.
- **Lidar and camera map matching:** a later increment.
- **Comparison with published results:** none is made. No public localisation result uses this simulator, these sensors and these metrics. The measured numbers are reported against the baselines run here.
