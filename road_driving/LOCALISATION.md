# Localisation in road_driving

The car can drive on its own state estimate instead of simulator truth. The world simulates five sensors from the truth: GNSS, an IMU, wheel encoders, a steering-angle sensor, and a range scanner at kerb height. Estimators see only those measurements and the map. The stack drives on an estimator's output through an explicit interface, and keeps from every kerb a margin set by that estimate's own uncertainty. Ground-truth driving remains as the labelled reference.

Labels: **SIMULATED_SENSORS** and **ESTIMATED_POSE**. The kerb scanner is simulated: its ranges are cast from the scanner's true pose to the mapped kerbs, shifted by a modelled deviation of the real kerb from the map. Kerb registration matches those ranges to the map. Nothing here is processed lidar or camera data.

Six evaluations, each predeclared and pushed before its held-out run:

| Evaluation | Question | Tested | Development | Held out |
|---|---|---|---|---|
| First | does a GNSS + IMU + wheel EKF beat simple baselines? | `ekf` and three baselines | R1 | R2 ([protocol](benchmark/PROTOCOL.md), [results](benchmark/heldout)) |
| Second | do GNSS integrity, kerb registration and a clearance-aware stack fix the first one's failures? | `ekf`, `ekf_int`, `ekf_kerb` and the baselines | R1 and R2 | R3 ([protocol](benchmark/PROTOCOL_R3.md), [results](benchmark/heldout-r3)) |
| Third | do a confirmed way out of GNSS faults and kerb re-localisation fix the second one's failures? | those, plus `ekf_int2` and `ekf_kerb2` | R1 to R3 | R4 ([protocol](benchmark/PROTOCOL_R4.md), [results](benchmark/heldout-r4)) |
| Fourth | does the stack's re-join after pose corrections help on unseen roads, at no cost? | the five filters, each driven with and without the re-join | R3 and R4 | R5 ([protocol](benchmark/PROTOCOL_R5.md), [results](benchmark/heldout-r5)) |
| Fifth | does re-joining only map-verified corrections keep the gain and remove the cost? | the five filters, each driven with and without the verified re-join | R3 to R5 | R6 ([protocol](benchmark/PROTOCOL_R6.md), [results](benchmark/heldout-r6)) |
| Sixth | does re-joining only unique kerb matches remove the harm of wrong ones and keep the gain? | the five filters, each driven with and without the unique re-join; the kerb filters also with the R6 version | R3 to R6 | R7 ([protocol](benchmark/PROTOCOL_R7.md), [results](benchmark/heldout-r7)) |

## Frames, units and time

- **Map frame:** x east, y north, metres.
- **Body frame:** origin at the rear-axle centre, x forward, y left.
- **Angles:** radians, anticlockwise from map x, wrapped to (−π, π] after every prediction and update.
- **Time:** seconds since the episode started.
- **Measurements:** each carries its time of validity (`tMeas`) and its arrival time (`tArrival`). Estimators process measurements in arrival order and apply each at its time of validity.

## Kerbs and clearance

**Geometry** (`src/lane_network.cpp`). The kerb is a set of straight lines and corner arcs built from the grid:
- each street has two side kerbs;
- at a junction, a side kerb stops where the corner arc begins, or crosses the junction box when the junction has no arm on that side;
- every pair of perpendicular arms gets an arc of the junction's kerb radius;
- a cul-de-sac gets an end kerb.

Clearance is the exact signed distance to the nearest kerb, positive on the carriageway. A ray from inside the carriageway is intersected with every kerb line and arc.

**A correction.** Until this increment, clearance came from a conservative function: the largest of the street rectangles' and corner fillets' own distances. It has the right sign everywhere, but near a rounded corner it understates the clearance of points just inside a street's edge line, by up to 3.6 m. Two published statements rested on it:
- "Driven exactly, R2's tightest left turn clears the edge by 0.001 m with mirrors" was this artifact; the exact figure is 0.24 m.
- The 13 nominal crossings in the first evaluation were attributed to tight turns. They happened at right-turn exits, where the front corner swings within 0.02 m (mirrors) of the exit street's far kerb, and in the first seconds after departure.

Checks run on R1 and R2:
- Over 400,000 random points, the exact distance has the old one's sign everywhere and is never smaller inside the carriageway.
- Every kerb line and arc lies on the old function's zero set, and every kerb point found by bisection lies on one of them.
- Re-evaluating all 200 recorded R2 closed-loop episodes changes no trace, outcome or body-crossing count. Only positive clearances grow: the reference's smallest clearance with mirrors is 0.017–0.128 m, not 0.000–0.001.

## Sensors (`src/sensors.cpp`)

| Sensor | Rate | Latency | Mounting | Errors |
|---|---|---|---|---|
| GNSS position | 10 Hz | 0.10 ± 0.02 s | antenna 1.60 m ahead of the rear axle | 0.25 m white noise per axis; 0.15 m Gauss-Markov error with a 60 s time constant; 2% of fixes dropped. The receiver always reports 0.30 m accuracy: it does not detect multipath |
| IMU: longitudinal and lateral specific force, yaw rate | 100 Hz | 0 | 1.40 m ahead of the rear axle | gyro 0.0015 rad/s noise, bias ±0.0025 rad/s plus a random walk; accelerometer 0.03 m/s² noise, bias ±0.04 m/s² plus a random walk. The lever arm puts −ω²·l in the longitudinal reading |
| Wheel speed (rear axle) | 50 Hz | 0.02 s | — | ±1% scale error, 0.02 m/s noise, 0.005 m/s resolution; exactly 0 at rest |
| Steering angle | 50 Hz | 0.02 s | — | 0.002 rad noise, ±0.004 rad offset |
| Kerb scanner: 37 beams over ±90° | 10 Hz | 0.05 s | front bumper, 3.84 m ahead of the rear axle | 0.03 m range noise; 2% dropout; 0.5–25 m. Returns come from the real kerb, which deviates from the map by 0.03 m RMS along 15–25 m wavelengths, or from an object standing in the carriageway |

The signs of biases and errors are drawn per seed. Each measurement is rounded through the log's text format when it is generated, so an estimator fed live and one replayed from `sensors.csv` receive bit-identical inputs. The kerb scanner and the new fault types draw from their own random stream, so every other measurement is unchanged by them. The previous filter therefore reproduces its recorded R2 results exactly.

### Conditions

Spans are fractions of the route, turned into map rectangles around the route.

| Development (R1) | Held out on R2, then development | Held out on R3 |
|---|---|---|
| `nominal` | `nominal_h` | `nominal_r3` |
| `gnss_outage`: no fixes over 35–60% of the route | `gnss_outage_h`: 50–80% | `gnss_outage_r3`: 35–75% |
| `multipath`: 4 m constant offset, 1.0 m extra noise, over 30–45% | `multipath_h`: 6 m, 1.5 m, over 55–75% | `multipath_r3`: 8 m, 1.0 m, over 40–70% |
| `wheel_slip`: wheel speed 20% high over 25–40% | `wheel_slip_h`: 30% high over 40–60% | — |
| `imu_bias`: gyro bias 0.012 rad/s, step +0.008 rad/s at 30 s | `imu_bias_h`: 0.016 rad/s, step +0.010 rad/s at 45 s | — |
| `delay_drop`: GNSS latency 0.40 ± 0.10 s, 30% of fixes dropped | `delay_drop_h`: 0.50 ± 0.12 s, 40% dropped | — |
| — | `combined_h`: outage, multipath, slip and latency together | `combined_r3`: outage, 7 m multipath, slip, parked cars and kerb clutter, gyro-bias step |
| — | — | `gnss_jumps_r3`: a 3 m offset switching on and off every 3 s over 30–70% |
| — | — | `gnss_ramp_r3`: a slow excursion with no jump, growing at 0.25 m/s to 5 m, then decaying |
| — | — | `kerb_clutter_r3`: parked cars against the opposite kerb, 15% short returns, no scans over 70–80% |

Held out on R4, for the second versions: new magnitudes and combinations, and a kerb map error.

| Condition | Definition |
|---|---|
| `nominal_r4` | as development nominal |
| `gnss_outage_r4` | no fixes over 25–70% |
| `multipath_r4` | 10 m constant offset, 2.0 m extra noise, over 35–60% |
| `gnss_ramp_r4` | a slow excursion growing at 0.4 m/s to 7 m over 25–65%, then decaying |
| `imu_fault_r4` | gyro-bias step +0.015 rad/s at 25 s; 6 m offset with 1.0 m noise over 30–55% |
| `kerb_faults_r4` | over 20–45% the kerb on the scanner's left lies 0.5 m further out than mapped; no kerb scans over 55–80%; 8 m offset with 0.5 m noise over 60–75% |
| `combined_r4` | outage over 10–25%; a 0.3 m/s excursion to 5 m over 40–70%; 30% wheel slip over 50–60%; 10% short kerb returns; gyro-bias step +0.008 rad/s at 40 s |

## Estimators (`src/estimators.cpp`)

**Common start.** Every estimator starts from the same information: its first GNSS fix, the antenna lever arm, and the heading of the lane that fix lies in.

### `ekf`: 9 states

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
- Reset: after 100 consecutive rejected fixes (10 s), GNSS is trusted again. The position uncertainty is reopened to cover the disagreement and the fix is accepted. This is what a persistent multipath offset captures.

**Numerical safeguards.**
- Joseph-form updates, with symmetrisation and a floor on variances.
- The innovation covariance is checked to be positive definite before it is inverted.
- A non-finite state or a negative variance restarts the filter from the next fix, counted as a reset.

**Observability.**
- Heading is observable from GNSS only while moving.
- The gyro bias is observable from turning and from standstill.
- The scale error is observable from GNSS distance.
- During an outage, position and the slow GNSS error are unobservable, so their uncertainty grows.
- A step change of gyro bias is followed slowly. Under `imu_bias` the filter is overconfident (development NEES 27).

### `ekf_int`: GNSS integrity instead of the reset

The same filter. No fix is ever trusted because time has passed. Three tests decide:
- **Jump test.** The filter keeps an odometry-only track of the antenna, which GNSS and kerbs never correct, re-aligned in heading at each fix. A fix-to-fix displacement that this track cannot explain (chi-square above 18.4, about 1.7 m) marks the receiver FAULTED, and the offset it jumped to is remembered. Fixes are then excluded until 5 in a row fit the filter's gate and clearly not that offset: the fault has ended.
- **Inconsistency.** 20 rejected fixes in a row with no jump mean that GNSS and the filter disagree, and either could be wrong.
- **Re-anchoring.** Only then, and only if all of the following hold:
  - the last 20 fixes all passed the jump test;
  - about a straight-line trend in time, their innovations scatter by at most 0.6 m. That is receiver noise, not multipath; a drifting filter makes the trend;
  - the latest offset is plausible for a filter up to three times overconfident;
  - no recent kerb scan supported the filter.

  The filter then admits the disagreement as uncertainty: in position by the offset, in heading by the drift rate it implies, and in the gyro bias. It then uses the fix normally.

Development tests on R1 cover both cases:
- **Persistent bias:** a clean 6 m offset over most of the route. `ekf` adopts it; `ekf_int` detects the jump and never trusts it.
- **Legitimate recovery:** an unmodelled gyro-bias step of 0.015 rad/s inside an outage leaves the filter 6.5 m off and overconfident. `ekf_int` re-anchors once and is back below 0.5 m 2.2 s after GNSS returns. `ekf`'s timed reset never recovers it, because it reopens position but not heading.

A slow excursion with no jump cannot be told apart from filter drift with GNSS and odometry alone. That limit is evaluated on R3 (`gnss_ramp_r3`).

### `ekf_kerb`: scan-to-map kerb registration

`ekf_int` plus each kerb scan:
- **Residual.** Each return's end point should lie on a mapped kerb, so its signed distance to the nearest kerb is the residual. Returns beyond a gate are not used: the gate is 3 lateral sigmas within 0.4–1.5 m, shrinking to 0.15 m.
- **Registration.** A Huber-weighted Gauss-Newton search over (x, y, heading), anchored by the filter's own prediction, finds the pose that fits the scan.
- **Constrained directions.** The scan's information matrix decides which directions it constrains. Along a straight kerb these are lateral position and heading; at a corner, all three. Each constrained direction becomes a scalar measurement of the filter.
- **Information.** The returns carry 0.04 m noise and are counted as at most 15 independent per scan. The real kerb's deviation from the map is correlated along it, so each scan's pose also carries a floor of 0.03 m and 0.004 rad, counted once per 5 m travelled.
- **Rejection.** A scan with fewer than 8 matched returns, or that disagrees with the filter beyond a chi-square of 16.3 (3 dof), is rejected and gives the filter no support.

With a heading reference the gyro bias is observable, so `ekf_kerb` lets it change faster (random walk 3e-4 instead of 1e-4 rad/s per root second). Development on R2 set that value and the floor: with 1e-4, a gyro-bias step left the heading 20 to 40 times overconfident.

### Second versions: `ekf_int2` and `ekf_kerb2`

They answer R3's two failures. Where nothing goes wrong each is its first version exactly; the tests check this trace for trace.

**`ekf_int2`: a way out of a fault.** In `ekf_int` a fault ends only when 5 fixes fit the filter and not the remembered offset. If the filter drifted while GNSS was excluded, neither ever fits again, and the fault never ends: R3's combined condition (T1). `ekf_int2` changes three things:
- **A candidate end.** The receiver jumps back by most of the onset jump (within 75% of it). Or it returns after a gap of more than 2 s and no longer fits the remembered offset.
- **Confirmation.** The fault ends only if the next 20 fixes look like a clean receiver: no jump, and at most 0.6 m scatter about a straight-line trend. Multipath noise makes jumps, and a noisy jump back can come by chance; until confirmed, the fault stands. The 5 fixes that clear a fault the first version's way must also be free of jumps.
- **Heading evidence.** The disagreement that remains is an inconsistency, tested as in `ekf_int`. The 20 fixes may also be re-anchored when the receiver's track and the filter's predicted antenna track over them differ in direction by more than 0.07 rad, both at least 10 m long. A drifted heading shows as such an angle; a constant receiver offset does not, so the persistent-bias case stays excluded.

GNSS integrity cannot bound the drift while GNSS is rightly excluded. With an unmodelled gyro-bias step the error still grows until the fault ends, and `ekf_int2` returns sooner after it.

**`ekf_kerb2`: kerb re-localisation.** A slow GNSS excursion drags `ekf_kerb` along the road, where kerbs say nothing. At the next junction the scans disagree with it beyond the gate, and every one is rejected: R3's T8. After 3 rejected scans in a row (at most every 0.5 s), `ekf_kerb2` searches for the pose the scan fits:
- a grid within 6 m along and across the filter's heading, in 0.5 m steps, then heading offsets of ±0.02 and ±0.04 rad at the best;
- a return counts when it ends within 0.3 m of a mapped kerb;
- the best must count at least 10 returns and half of those scored, and beat the filter's own pose by 5.

Uniqueness decides what is adopted:
- **Unique in every direction** (a corner, a junction mouth): the best must beat every pose 1.5 m or more away by 4, and the whole offset is adopted.
- **Unique only across the road** (a straight kerb fits any offset along it): the offset across is adopted, and the filter's position along is kept.

The covariance is widened by the correction, the scan is registered from the found pose, and the receiver, which agreed with the rejected pose, must earn trust again.

### Baselines

All three get the same log and the same start.

| Baseline | Definition |
|---|---|
| `dr` | wheel speed and gyro dead reckoning, with the gyro bias calibrated at standstill (the EKF's standstill rule). No covariance |
| `gnss_cv` | a constant-velocity Kalman filter on the antenna position, with the same gate, robust weighting and reset as `ekf`. Heading from its velocity |
| `gnss_hold` | the latest fix, held, with heading from fix-to-fix motion |

## Driving on the estimate

**The interface.** In estimated mode the stack's pose is the estimator's output at 50 Hz. It carries:
- position, heading, speed and acceleration (IMU, bias-corrected);
- the steering sensor;
- the estimate's validity, age, position covariance and heading variance.

The stack never receives truth. The world records the truth and an independent evaluator scores it afterwards.

**Localisation policy** (`locpolicy` in `src/stack.hpp`, unchanged since [the first protocol](benchmark/PROTOCOL.md)):
- **Slow:** cap the speed at 5 m/s when the lateral 1-sigma exceeds 0.35 m or the longitudinal exceeds 1.5 m.
- **Stop:** stop in lane at 2.5 m/s² when the estimate is invalid or stale (over 0.25 s), or has no covariance, or is above 0.75 m lateral or 3.0 m longitudinal.
- **Resume:** after 2 s below 0.30 m lateral.

**Clearance layer** (`clearpolicy` in `src/stack.hpp`, new in the second evaluation):
- **Margin.** The body swept along the next 60 m of path must keep from every kerb two footprint sigmas plus 0.10 m of tracking allowance. The footprint sigma combines the lateral sigma with the heading sigma acting at the front bumper; on simulator truth it is zero.
- **Shift.** Where the path is too close to a kerb, the stack shifts it sideways, as raised-cosine bumps:
  - on turns, within the carriageway, by up to 0.5 m;
  - on lanes, only towards the centre line, keeping 0.10 m from it.

  The shift changes by at most 0.03 m per metre the car moves, never at standstill, after a step at departure set off an oscillation in development.
- **Stop.** Where no permitted shift gives the margin, the stack stops before that point (`NO_CLEAR_CONTINUATION`). It resumes after 2 s with 0.05 m to spare.
- **Monitor.** The constraint monitor keeps watching the lateral error during such a stop.

On simulator truth the layer changes nothing on R1's 192 closure episodes (byte-identical traces). On R2's right-turn exits it shifts the reference by about 0.01 m.

**Pose corrections** (`rejoinpolicy` in `src/stack.hpp`, added after the third evaluation):
- **The problem.** An estimate can jump: a kerb re-localisation, a GNSS re-anchor, GNSS returning after dead reckoning. The car did not move. Chasing the old reference turned such a jump into a steering step. Pure pursuit, steering-rate limited, then oscillated until the monitor's fallback stop.
- **Detection.** The cross-track error from the reference changes between two control ticks (20 ms) by more than 0.15 m, which no motion explains. Ordinary filter updates stay well below that: none triggered it in nominal or outage drives.
- **Re-join.** The controller's reference moves sideways by exactly that change, so its error stays continuous. The reference then returns to the path along a raised cosine over 10 m, or over 20 m per metre of correction if longer. Corrections above 1.5 m are not re-joined.
- **Monitor.** It measures from the path the car should follow, clearance shift included. Before, it measured from the unshifted path, so a clearance shift ate into its 1 m limit. It deliberately ignores the re-join. A jump can also be a corruption, such as the timed-reset EKF snapping onto multipath fixes, and the stack cannot tell which. A first version that also re-joined the monitor drove one such car 9.3 m off the road, where the old monitor had stopped it at the jump.

On simulator truth nothing jumps, and R1's 192 closure episodes stay byte-identical.

**Which corrections are re-joined** (`Rejoin` in `include/road_driving/episode.hpp`):
- `on`: every jump, as evaluated on R5;
- `verified`: only corrections the estimator verified against the map by kerb registration, as evaluated on R6;
- `unique`, the default: only those whose kerb scan fits the map uniquely, as evaluated on R7;
- `off`: none, the R3 and R4 stack.

With `verified` and `unique`, every other jump meets the R3 and R4 stack, monitor included.

There is no fallback to truth. A car that stops inside a GNSS outage waits there until the estimate recovers.

**Closed-loop metrics** (from truth):
- outcome and time to goal, and stop margin and incursion at closures;
- true cross-track error;
- whole-vehicle clearance, with mirrors, to the exact kerb;
- runs with the body (without mirrors) over a kerb, and how deep;
- lane association: the stack's element against the evaluator's lane, per control tick;
- localisation stops and clearance stops. A stop is unwarranted when the true position error at the decision was below 0.3 m: the stop came from reported uncertainty, not actual error.

## Benchmark method (`src/benchmark.cpp`)

**Open loop.** Each drive is recorded once with ground-truth driving. Every estimator then processes the same sensor log, in the same order, and is scored against the same truth in the fixed map frame. Nothing is aligned per trajectory. Metrics:
- position RMSE, median, 95th percentile and maximum;
- lateral error, the part across the true heading;
- heading and speed error;
- relative position error over 5 s and 20 s windows;
- error when GNSS returns from the longest outage, the largest error inside it, and the time back below 0.5 m;
- failures, meaning error above 5 m;
- NEES with the share inside the 95% chi-square interval, and NIS for GNSS, wheel and kerb;
- integrity events (jumps detected, faults cleared, re-anchors) and kerb scans used;
- for the second versions, faults left by the new path, re-anchors on the track angle, and kerb re-localisations;
- runtime and per-measurement wall-clock latency.

Failed runs stay in every statistic.

**Closed loop in parts.** `bench-closed --scenario` runs some of a split's scenarios, and `tools/merge_closed_loop.py` merges the parts. Each scenario's episodes, its reference included, run in one part, so the merged files equal those of one process. On R3 development data with one seed, both merged files were byte-identical to one process's. The third evaluation's closed loops, development and held out, ran as four parts each.

**Replay without the simulator.**

```bash
road_drive localise <episode dir>
```

This reruns every estimator from that folder's `sensors.csv` and `truth.csv` and writes their traces. A test confirms it reproduces the in-process estimates to 1e-9 m.

## Results

All measured on 17 and 18 November 2026 on one Windows 11 laptop, with seeds 1 to 10.

| Evaluation | Network | Build | Open loop | Closed loop | Full tables |
|---|---|---|---|---|---|
| First, held out | R2 | MSVC 19.51 at `62a742a` | 210 drives, 840 runs | 200 episodes | [`benchmark/heldout/`](benchmark/heldout) |
| Second, development | R2 | GCC 14.2 at `7781fe2` | 210 drives, 1,260 runs | 680 episodes | [`benchmark/r2-dev/`](benchmark/r2-dev) |
| Second, held out | R3 | MSVC 19.51 at `9332dd3` | 210 drives, 1,260 runs | 1,000 episodes | [`benchmark/heldout-r3/`](benchmark/heldout-r3) |
| Third, development | R3 | GCC 14.2 at `b7ee7f2` | 210 drives, 1,680 runs | 1,240 episodes | [`benchmark/r3-dev/`](benchmark/r3-dev) |
| Third, held out | R4 | MSVC 19.51 at `87436de` | 210 drives, 1,680 runs | 1,440 episodes | [`benchmark/heldout-r4/`](benchmark/heldout-r4) |
| Fourth, development | R3 and R4 | GCC 14.2 and MSVC 19.51 at `f674183` | — | 2,680 episodes | [`benchmark/rejoin-dev/`](benchmark/rejoin-dev) |
| Fourth, held out | R5 | MSVC 19.51 at `2966c0a` | 210 drives, 1,680 runs | 2,840 episodes | [`benchmark/heldout-r5/`](benchmark/heldout-r5) |
| Fifth, held out | R6 | MSVC 19.51 at `aa20e04` | 210 drives, 1,680 runs | 2,840 episodes | [`benchmark/heldout-r6/`](benchmark/heldout-r6) |
| Sixth, held out | R7 | MSVC 19.51 at `27a1ec8` | 210 drives, 1,680 runs | 3,400 episodes | [`benchmark/heldout-r7/`](benchmark/heldout-r7) |

Each protocol was pushed before its held-out run: [first](benchmark/PROTOCOL.md) in `62a742a`, [second](benchmark/PROTOCOL_R3.md) in `9332dd3`, [third](benchmark/PROTOCOL_R4.md) in `87436de`, [fourth](benchmark/PROTOCOL_R5.md) in `2966c0a`, [fifth](benchmark/PROTOCOL_R6.md) in `aa20e04`, [sixth](benchmark/PROTOCOL_R7.md) in `27a1ec8`. No held-out network was used for tuning. The development reruns of the fifth and sixth evaluations are described in their protocols ([fifth](benchmark/PROTOCOL_R6.md#what-development-did), [sixth](benchmark/PROTOCOL_R7.md#what-development-did)).

### First evaluation: held out on R2

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

Outages: when GNSS returns, the EKF's median error is 0.40 m, against dead reckoning's 5.57 m. NEES (3 dof, ideal 3) was 1.63–1.78, except 19.0 under `imu_bias_h` and 39.9 under `multipath_h` (overconfident).

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

Closed loop, driving on the EKF, against the ground-truth reference with the same seed:
- **Under `nominal_h`:** every mission ended as the reference did, and closure stops ended at least 1.73 m from the barrier (reference: 1.86 m). The body crossed a kerb in 13 of 40 episodes, with the footprint with mirrors up to 0.24 m past it.
  - **Corrected after the second evaluation's geometry fix:** 10 of the 13 crossings happened at right-turn exits (R2-B at J00, R2-C at J11 and J12), and 3 in the first seconds after departure. None happened at a left turn.
  - The published explanation, a tight left turn clearing the edge by 0.001 m when driven exactly, was an artifact of the conservative distance (see [Kerbs and clearance](#kerbs-and-clearance)).
- **Under `gnss_outage_h`:** R2-B and R2-C completed. On R2-A the outage lies on one 127 m straight; the lateral 1-sigma passed the 0.75 m stop threshold after about 107 m, and the car stopped inside the outage and waited (20 of 20 episodes).
- **Under `multipath_h`:** 28 of 40 episodes ended away from the reference outcome. The 6 m offset outlasted the 10 s rejection window, the filter reset onto the biased fixes, and the monitor braked to a fallback stop. In one episode (R2-B, seed 3) the car left the carriageway first, its footprint with mirrors 4.36 m past the edge.
- **Under `combined_h`:** every mission ended as the reference did.

### Second evaluation: development on R2

Median-run RMSE in metres, then failed runs out of 30, and the pooled lateral 95th percentile:

| Condition | `ekf` | `ekf_int` | `ekf_kerb` |
|---|---|---|---|
| `nominal_h` | 0.18, 0; 0.27 | 0.18, 0; 0.27 | **0.08, 0; 0.044** |
| `gnss_outage_h` | 0.22, 0; 0.32 | 0.22, 0; 0.32 | **0.08, 0; 0.044** |
| `multipath_h` | 0.29, **8**; 4.22 | 0.20, 0; 0.29 | **0.08, 0; 0.044** |
| `wheel_slip_h` | 0.18, 0; 0.27 | 0.18, 0; 0.27 | **0.08, 0; 0.044** |
| `imu_bias_h` | 0.23, 0; 0.45 | 0.23, 0; 0.44 | **0.08, 0; 0.044** |
| `delay_drop_h` | 0.20, 0; 0.29 | 0.20, 0; 0.29 | **0.08, 0; 0.044** |
| `combined_h` | 0.20, 0; 0.32 | 0.21, 0; 0.32 | **0.09, 0; 0.044** |

`ekf` reproduces the first evaluation's runs exactly. `ekf_int` equals it wherever no integrity event occurs. `ekf_kerb` had a lower RMSE than both in 30 of 30 paired runs in every condition.

Closed loop: `ekf_kerb` ended all 160 episodes as the reference did, with the body never over a kerb and no stop. With the clearance layer:
- **Nominal:** the GNSS-only filters had 6 departure crossings (0.14 m deep), where the previous stack had 13 episodes.
- **Outages and combined faults:** they stopped in every episode, 49 of the 80 stops unwarranted.

### Second evaluation: held out on R3

Median-run RMSE in metres, then failed runs out of 30, and the pooled lateral 95th percentile:

| Condition | `ekf` | `ekf_int` | `ekf_kerb` |
|---|---|---|---|
| `nominal_r3` | 0.18, 0; 0.29 | 0.18, 0; 0.29 | **0.08, 0; 0.043** |
| `gnss_outage_r3` | 0.34, 0; 0.61 | 0.34, 0; 0.61 | **0.09, 0; 0.043** |
| `multipath_r3` | 4.40, **30**; 7.67 | 0.25, 0; 0.43 | **0.08, 0; 0.043** |
| `gnss_jumps_r3` | 0.19, 0; 0.30 | 0.18, 0; 0.30 | **0.08, 0; 0.043** |
| `gnss_ramp_r3` | 2.05, **3**; 3.61 | 2.05, **3**; 3.56 | 0.10, 0; **1.38** |
| `kerb_clutter_r3` | 0.18, 0; 0.29 | 0.18, 0; 0.29 | **0.08, 0; 0.055** |
| `combined_r3` | 5.25, **20**; 13.0 | 10.36, **20**; 29.2 | **0.09, 0; 0.047** |

When GNSS returns from the outage, the median error is 0.045 m for `ekf_kerb` and 0.646 m for `ekf`. NEES of `ekf_int` and `ekf_kerb` was 1.25–1.83 under the nominal, outage and multipath conditions. Under the slow excursion `ekf_kerb`'s NEES was 34 (overconfident), and under combined faults 6.8.

| # | Criterion | Held-out result | Verdict |
|---|---|---|---|
| T1 | No `ekf_int` or `ekf_kerb` failure under multipath, jumps or combined faults | `ekf_kerb` 0 of 90; `ekf_int` 20 of 90, all under `combined_r3` (`ekf`: 50 of 90) | **failed** |
| T2 | `ekf_int` costs nothing on clean GNSS (pooled 95th percentile within 5%) | 0.357 against 0.357 m | passed |
| T3 | `nominal_r3`: `ekf_kerb` pooled lateral 95th percentile at most 0.10 m and half of `ekf`'s | 0.043 against 0.287 m | passed |
| T4 | `gnss_outage_r3`: `ekf_kerb` error when GNSS returns at most half of `ekf`'s | 0.045 against 0.646 m | passed |
| T5 | NEES 0.5–6 for `ekf_int` and `ekf_kerb`, nominal, outage and multipath | 1.25–1.83 | passed |
| T6 | `kerb_clutter_r3`: no `ekf_kerb` failure, pooled lateral 95th percentile at most 0.20 m | 0 of 30; 0.055 m | passed |
| T7 | Closed loop, `nominal_r3`: `ekf_kerb` ends as the reference does, never over a kerb, no stop | 40 of 40, 0, 0 | passed |
| T8 | No `ekf_kerb` episode over a kerb; no closure incursion with any pose source | 8 of 240 over a kerb and 21 of 1,000 incursions, all under `gnss_ramp_r3` | **failed** |
| T9 | `ekf_kerb` ends as the reference does in at least 38 of 40 under outage, multipath and combined faults | 40, 40 and 40 of 40 | passed |
| T10 | No unwarranted stop with `ekf_kerb` under `nominal_r3` | 0 | passed |

Closed loop, against the ground-truth reference with the same seed. The reference itself never crossed a kerb or stopped for clearance. Each cell is out of 40 episodes:

| Condition | Measure | `ekf`, previous stack | `ekf` | `ekf_int` | `ekf_kerb` |
|---|---|---|---|---|---|
| `nominal_r3` | same outcome; runs over a kerb | 40; 10 | 40; 3 | 40; 3 | **40; 0** |
| `gnss_outage_r3` | same outcome; over a kerb; stops (unwarranted) | 8; 4; 30 (20) | 0; 3; 44 (29) | 0; 3; 44 (29) | **40; 0; 0** |
| `multipath_r3` | same outcome; over a kerb (deepest) | 0; 18 (1.4 m) | 0; 10 (5.7 m) | 0; 3 (0.14 m) | **40; 0** |
| `gnss_ramp_r3` | same outcome; over a kerb (deepest); incursions | 9; 35 (4.1 m); 6 | 9; 35 (4.1 m); 6 | 9; 35 (4.1 m); 6 | 33; 8 (3.9 m); 3 |
| `kerb_clutter_r3` | same outcome; over a kerb | 40; 10 | 40; 3 | 40; 3 | **40; 0** |
| `combined_r3` | same outcome; over a kerb (deepest) | 5; 31 (11.0 m) | 0; 3 (0.14 m) | 0; 3 (0.14 m) | **40; 0** |

With the clearance layer, the GNSS-only filters stopped in every outage, multipath and combined episode; `ekf_kerb` never stopped.

### Third evaluation: development on R3

R3 became development data for the second versions. In the two conditions where any version differs, here are the median-run RMSE in metres, the failed runs out of 30, and the pooled lateral 95th percentile:

| Condition | `ekf_int` | `ekf_int2` | `ekf_kerb` | `ekf_kerb2` |
|---|---|---|---|---|
| `gnss_ramp_r3` | 2.05, 3; 3.56 | 2.05, 3; 3.56 | 0.10, 0; 1.38 | 0.10, 0; **0.045** |
| `combined_r3` | 10.36, 20; 29.2 | **2.07**, 20; 3.93 | 0.09, 0; 0.047 | 0.09, 0; 0.047 |

- **Combined faults.** `ekf_int2` left the fault in all 20 runs where `ekf_int` never did. Its largest errors were 6.9–11.2 m instead of 28.4–44.1 m. Those 20 runs still fail: until the fault ends, the filter dead-reckons through a gyro-bias step.
- **Slow excursion.** `ekf_kerb2` re-localised once in each of 5 R3-B runs, and its NEES fell from 34 to 2.1.
- **Elsewhere.** In every other condition each second version equals its first, run for run.

Closed loop, 1,240 episodes:
- **Slow excursion:** `ekf_kerb2` ended 36 of 40 episodes as the reference did (`ekf_kerb`: 33). Its body was over a kerb in 4 (`ekf_kerb`: 8), with one closure incursion (`ekf_kerb`: 3) and no stop (`ekf_kerb`: 4).
- **Every other condition:** it equals `ekf_kerb`.
- **`ekf_int2`:** it equals `ekf_int` everywhere. Under combined faults the GNSS-only filters stop for clearance inside the outage, so the multipath, and the way out of it, never comes.

The 4 crossings are two drives, R3-B seeds 2 and 9, each in both R3-B scenarios:
- **Seed 2.** The excursion builds 1.5 m of along-road error on the straight before J00, where kerbs cannot see it. The only clear match came mid-turn: 17 returns against 14 for the best alternative, one short of the required margin. After that the car was too far off the road to see 20 returns.
- **Seed 9.** Re-localisation worked: a 1.2 m correction mid-turn, after which the estimate stayed within 0.04 m. But that correction started a steering oscillation in pure pursuit that grew for 5 s, and the monitor's fallback stop ended with the body 1.4 m over the kerb. With the clearance layer off, the same drive's oscillation decays: the drive completes, the body 0.33 m over the kerb. The stack is lightly damped after a large pose correction.

### Third evaluation: held out on R4

Median-run RMSE in metres, then failed runs out of 30, and the pooled lateral 95th percentile:

| Condition | `ekf` | `ekf_int` | `ekf_int2` | `ekf_kerb` | `ekf_kerb2` |
|---|---|---|---|---|---|
| `nominal_r4` | 0.18, 0; 0.27 | 0.18, 0; 0.27 | 0.18, 0; 0.27 | **0.08, 0; 0.044** | **0.08, 0; 0.044** |
| `gnss_outage_r4` | 0.43, 0; 0.81 | 0.43, 0; 0.81 | 0.43, 0; 0.81 | **0.07, 0; 0.044** | **0.07, 0; 0.044** |
| `multipath_r4` | 4.96, **30**; 9.78 | 0.24, 0; 0.38 | 0.24, 0; 0.38 | **0.08, 0; 0.044** | **0.08, 0; 0.044** |
| `gnss_ramp_r4` | 3.49, **29**; 6.08 | 3.17, **30**; 11.0 | 2.92, **30**; 5.01 | 0.12, **1**; 0.049 | **0.12, 0; 0.044** |
| `imu_fault_r4` | 8.39, **30**; 17.6 | 24.9, **30**; 46.4 | 1.83, **20**; 4.16 | **0.08, 0; 0.045** | **0.08, 0; 0.045** |
| `kerb_faults_r4` | 3.26, **20**; 6.36 | 0.19, 0; 0.29 | 0.20, 0; 0.29 | 0.12, 0; 0.198 | 0.12, 0; 0.201 |
| `combined_r4` | 2.72, **17**; 6.14 | 2.11, **12**; 10.8 | 2.02, **11**; 3.61 | 0.12, 0; 0.600 | **0.12, 0; 0.045** |

- **Gyro fault during multipath.** `ekf_int2` left the fault in 29 of 30 runs; its largest errors were 2.9–11.3 m, against `ekf_int`'s 29.7–69.8 m. In the remaining run, the first biased fix happened to fall short and the onset jump caught only 3.4 m of the 6 m offset. The 6.7 m jump back then missed the 75% test, and the fault never ended.
- **Slow excursion.** `ekf_kerb2` re-localised 3 times and failed no run, with a largest error of 1.0 m (`ekf_kerb`: 1 failure, 6.1 m). Its NEES was 2.0, against `ekf_kerb`'s 305: dragged, the first version was confidently wrong.
- **Combined faults.** `ekf_kerb2` re-localised 6 times. Its lateral 95th percentile was 0.045 m instead of 0.600, and its largest error 2.2 m instead of 4.7.
- **Kerb faults.** A kerb 0.5 m from where the map has it, over a quarter of the route, triggered no re-localisation, and no run failed. With that stretch and the later blackout, both kerb filters' lateral 95th percentile was 0.20 m (nominal: 0.044 m).
- **Consistency.** Under the gyro fault both kerb filters' NEES was 14 (overconfident heading). Under the slow excursion, the gyro fault and combined faults, the GNSS-only filters were inconsistent, `ekf_int2` included (NEES 68–302).

| # | Criterion | Held-out result | Verdict |
|---|---|---|---|
| U1 | Second versions equal the first under `nominal_r4` | 30 of 30 each | passed |
| U2 | `imu_fault_r4`: `ekf_int2` pooled 95th percentile at most half of `ekf_int`'s | 4.80 against 54.8 m | passed |
| U3 | `multipath_r4`: no `ekf_int2` failure | 0 of 30 | passed |
| U4 | `gnss_ramp_r4`: no `ekf_kerb2` failure, pooled lateral 95th percentile at most 0.20 m | 0 of 30; 0.044 m | passed |
| U5 | `kerb_faults_r4`: no `ekf_kerb2` failure, pooled lateral 95th percentile at most 0.50 m | 0 of 30; 0.201 m | passed |
| U6 | NEES 0.5–6 for both second versions under nominal, outage and multipath | 1.32–1.84 | passed |
| U7 | No condition where a second version fails more runs than its first | none | passed |
| U8 | Closed loop, `nominal_r4`: `ekf_kerb2` ends as the reference does, never over a kerb, no stop | 40 of 40, 0, 0 | passed |
| U9 | No `ekf_kerb2` episode over a kerb; no closure incursion with any pose source | 1 of 280 over a kerb (0.91 m); 2 incursions, by `ekf_int` and `ekf_int2` | **failed** |
| U10 | `ekf_kerb2` ends as the reference does in at least 38 of 40 under outage, multipath, slow excursion and combined faults | 40, 40, 40 and 39 of 40 | passed |
| U11 | No condition where a second version crosses a kerb in more episodes than its first | none | passed |

Closed loop, against the ground-truth reference with the same seed (1,440 episodes). The reference never crossed a kerb or stopped. Each cell is out of 40 episodes:

| Condition | Measure | `ekf` | `ekf_int` | `ekf_int2` | `ekf_kerb` | `ekf_kerb2` |
|---|---|---|---|---|---|---|
| `nominal_r4` | same outcome; over a kerb | 40; 4 | 40; 4 | 40; 4 | **40; 0** | **40; 0** |
| `gnss_outage_r4` | same outcome; over a kerb; stops (unwarranted) | 0; 3; 40 (21) | 0; 3; 40 (21) | 0; 3; 40 (21) | **40; 0; 0** | **40; 0; 0** |
| `multipath_r4` | same outcome; over a kerb (deepest) | 0; 5 (1.3 m) | 0; 3 (0.14 m) | 0; 3 (0.14 m) | **40; 0** | **40; 0** |
| `gnss_ramp_r4` | same outcome; over a kerb (deepest) | 0; 38 (7.3 m) | 0; 40 (7.3 m) | 0; 39 (6.8 m) | 37; 2 (2.1 m) | **40; 0** |
| `imu_fault_r4` | same outcome; over a kerb (deepest) | 0; 25 (10.9 m) | 0; 29 (1.0 m) | 0; 29 (1.0 m) | **40; 0** | **40; 0** |
| `kerb_faults_r4` | same outcome; over a kerb; stops (unwarranted) | 10; 4; 31 (18) | 17; 3; 40 (26) | 18; 3; 40 (26) | 14; 0; 40 (34) | 14; 0; 40 (34) |
| `combined_r4` | same outcome; over a kerb (deepest) | 1; 9 (7.7 m) | 4; 9 (7.7 m) | 4; 9 (3.4 m) | 35; 3 (3.9 m) | **39; 1 (0.9 m)** |

- **The one crossing.** It is the development mechanism again (R4-A, seed 10). `ekf_kerb2` re-localised mid-turn by 0.8 m, after which its estimate stayed within 0.17 m. The car's steering kept oscillating until the monitor's fallback stop, 1.0 m off the path, during which the body went 0.91 m over the kerb.
- **Kerb faults.** With no kerb scans and GNSS excluded at once, both kerb filters dead-reckon, and the clearance layer stops the car in every episode, 34 of the 40 stops unwarranted. On R4-C the car is still slowing towards that stop when GNSS returns and the estimate is corrected, by up to 0.9 m. In 6 of 10 episodes the monitor then measured 1 m of lateral error and made a fallback stop, with no crossing.
- **Closure incursions.** Both were by GNSS-only filters, dragged by the slow excursion, on R4-B_late_closure seed 1.

### After the third evaluation: pose corrections in the stack (development)

The held-out crossing (U9) came from the stack, not the estimate. The re-join (see [Driving on the estimate](#driving-on-the-estimate)) was then developed on R3 and R4, both used before, so these are development results. The closed loops were rerun at `f674183` with the same compiler as the earlier runs on each network: MSVC 19.51 on R4, compared with the held-out run, and GCC 14.2 on R3, compared with [`r3-dev/`](benchmark/r3-dev). Full files: [`benchmark/rejoin-dev/`](benchmark/rejoin-dev).

Only the cells that changed are shown. Each is out of 40 episodes; fallback stops count the episodes that ended in one.

| Network | Condition | Pose source | Same outcome as reference | Over a kerb (deepest) | Fallback stops |
|---|---|---|---|---|---|
| R4 | `combined_r4` | `ekf_kerb2` | 39 → **40** | 1 (0.91 m) → **0** | 1 → **0** |
| R4 | `kerb_faults_r4` | `ekf_kerb2` | 14 → **18** | 0 → 0 | 6 → **2** |
| R4 | `kerb_faults_r4` | `ekf_kerb` | 14 → **19** | 0 → 0 | 6 → **1** |
| R4 | `multipath_r4` | `ekf_int`, `ekf_int2` | 0 → **10** | 3 → 3 | 10 → **0** |
| R4 | `multipath_r4` | `ekf` | 0 → 1 | 5 (1.26 m) → **3 (0.14 m)** | 40 → 40 |
| R4 | `gnss_outage_r4` | GNSS-only filters | 0 → 0 | 3 → 3 | 30 → **0** |
| R4 | `combined_r4` | `ekf_int`, `ekf_int2` | 4 → 3 | 9 → 9 | 3 → 4 |
| R3 | `gnss_ramp_r3` | `ekf_kerb2` | 36 → **38** | 4 (3.92 m) → **2** (3.92 m) | 4 → **2** |
| R3 | `multipath_r3` | `ekf` | 0 → 0 | 10 (5.66 m) → 10 (**1.45 m**) | 40 → 40 |
| R3 | `multipath_r3` | `ekf_int`, `ekf_int2` | 0 → 0 | 3 → 3 | 11 → **0** |

- **`ekf_kerb2`** crossed a kerb in none of R4's 280 episodes, and in 2 of R3's 240. Those 2 are one drive, R3-B seed 2, in both R3-B scenarios: the re-localisation that came too late.
- **Unchanged:** nominal driving everywhere. In no condition does any pose source cross a kerb more often or deeper than before. 413 of 1,440 episodes changed on R4, and 76 of 1,240 on R3.
- **The one cell that got worse.** On R4-C under combined faults, seed 4, the GNSS-only integrity filters used to complete the route with the body 2.18 m over a kerb, on an estimate 4.2 m off. The monitor now stops them, with the same crossing.
- **Not shown here:** whether the gain holds on roads it was not developed on. The fourth evaluation tests that.

### Fourth evaluation: the re-join, held out on R5

Every filter drove every R5 episode twice on the same seeds, with and without the re-join: 1,400 pairs, plus the reference.

| # | Criterion | Held-out result | Verdict |
|---|---|---|---|
| V1 | Under `nominal_r5` every pair has identical traces | 200 of 200 | passed |
| V2 | With the re-join, `ekf_kerb2`'s body is never over a kerb | 0 of 280 | passed |
| V3 | No filter, in any condition, crosses a kerb in more episodes, or deeper, with the re-join | five cells cross in one more episode: `ekf`, `ekf_int` and `ekf_int2` under combined faults; `ekf_int` and `ekf_int2` under the slow excursion. None deeper | **failed** |
| V4 | Fewer fallback stops, summed over all conditions and filters | 339 without, 326 with | passed |
| V5 | `ekf_kerb2` ends as the reference does at least as often in every condition, and all filters together at least as often | `ekf_kerb2`: equal everywhere; all filters: 665 without, 664 with | **failed** |
| V6 | No closure incursion by `ekf_kerb2` with the re-join | 0 | passed |

**What R5 showed.**
- **The failure the re-join was built for did not occur on R5.** `ekf_kerb` and `ekf_kerb2` never crossed a kerb and never made a fallback stop, with or without the re-join, in any condition. Their outcomes were identical in every pair. V2's pass therefore shows no harm, not a benefit.
- **Where the re-join acted** (392 of 1,400 pairs changed), it was almost always on a GNSS-only filter whose estimate was already metres wrong.
  - Under the GNSS outage it removed all 9 of their fallback stops (3 per filter), with outcomes unchanged.
  - Under the slow excursion and combined faults it changed which episodes crossed, in both directions. In one pair (R5-C, slow excursion, seed 2) `ekf_int` and `ekf_int2` crossed by 2.90 m with the re-join and not at all without. There the old monitor's earlier stop happened to come before the crossing.
  - In others the re-join removed a crossing: `ekf` on R5-B under kerb faults, seed 9, 0.37 m.
  - The net was one more crossing episode in five cells, and one completion fewer.
- **Open loop** (reported without a criterion):
  - `ekf_kerb2` failed no run in any condition; its lateral 95th percentile was 0.043–0.046 m, and 0.166 m under kerb faults.
  - Under combined faults `ekf_int2` failed 7 of 30 runs, against `ekf_int`'s 23.
  - Under the gyro fault in an outage, both GNSS integrity filters failed 25 of 30, and the kerb filters none.

**What this means.** On development data the re-join removed crossings that followed correct kerb re-localisations. On unseen roads that situation did not arise, and for filters without an independent reference it was not an improvement: it changes which wrong estimates end in a crossing, at a small net cost. A stack cannot tell a correction from a corruption by the jump alone.

### Fifth evaluation: the verified re-join, held out on R6

**What changed.**
- The estimator reports each kerb registration that moved its estimate by more than 0.1 m, and by how much (`StateEstimate::verifiedCorrections`).
- The stack re-joins only those corrections, by the cross-track change each one made.
- Every other jump meets the R3 and R4 stack, monitor included.

R6's conditions were designed to produce large map-verified corrections. Each filter drove every episode with and without the verified re-join: 1,400 pairs.

| # | Criterion | Held-out result | Verdict |
|---|---|---|---|
| W1 | Under `nominal_r6` every pair has identical traces | 200 of 200 | passed |
| W2 | Every pair of the GNSS-only filters has identical traces, in every condition | 840 of 840 | passed |
| W3 | With the verified re-join, the kerb filters cross no more often, no deeper, and fall back no more often, in every condition | `ekf_kerb` crossed deeper in two episodes: 3.64 → 8.94 m under combined faults, 6.24 → 11.21 m under the gyro fault | **failed** |
| W4 | Over the pairs acted on, fewer episodes end over a kerb or in a fallback stop | 62 pairs: 56 without, 51 with | passed |
| W5 | `ekf_kerb2` ends as the reference does at least as often in every condition | in every condition | passed |
| W6 | No closure incursion by `ekf_kerb2` with the verified re-join | 1 (0.70 m), identical without the re-join | **failed** |

**What R6 showed.**
- **The gain, where the targeted situation arose.** Under `kerb_return_r6` the GNSS offset grows while the kerbs are blacked out, and the kerbs then correct the estimate. There `ekf_kerb2` ended 26 of 40 episodes as the reference did, against 16 without the re-join, with 12 fallback stops against 22.
  - In no condition did `ekf_kerb2` cross more often or deeper with the re-join.
  - Under the gyro fault its deepest crossing fell from 6.24 m to 3.72 m.
- **No cost without a map reference.** The GNSS-only filters drove every pair identically, as designed.
- **The flaw W3 found.** After long dead reckoning, with no kerbs and no GNSS, a registration that starts from a very uncertain estimate can lock onto the wrong kerb.
  - In two such episodes the first kerb filter, `ekf_kerb`, reported a correction that was wrong. The re-join followed it smoothly, and the error grew to 11–14 m.
  - Without the re-join the monitor had stopped the car sooner, though still after crossings of 3.6–6.2 m.
  - On the same two drives, `ekf_kerb2` fared differently.
    - In the gyro-fault episode it also made two verified corrections, and its crossing became shallower: 6.24 → 3.72 m.
    - In the combined-fault episode it never drifted beyond 1.66 m and made none.
- **W6 is not about the re-join.** The `ekf_kerb2` incursion (R6-B late closure, kerbs returning, seed 2) is the same 0.70 m without it, with no correction re-joined. The estimate had been dragged 2.9 m by the GNSS drift during the blackout when the closure stop was planned.
- **Open loop** (reported without a criterion):
  - Under combined faults `ekf_kerb2` failed no run (lateral 95th percentile 0.046 m), and `ekf_int2` failed 7 of 30 against `ekf_int`'s 14.
  - Under the gyro fault with no kerbs and no GNSS, every filter failed 30 of 30; median-run RMSE was 2.57 m for `ekf_kerb2` and 10.42 m for `ekf_kerb`.

**What this means.** Limiting the re-join to map-verified corrections kept the gain and removed the cost for filters without a map reference. But a kerb match is evidence only when it is unambiguous. A verification that counts any registration trusts a match the filter's own uncertainty could not distinguish from a neighbouring kerb.

### Sixth evaluation: the unique re-join, held out on R7

**What changed.**
- The estimator also reports which kerb corrections come from a scan that fits the map uniquely. The registered pose must put at least 10 of the scored returns, and half of them, within 0.3 m of a mapped kerb, and beat every pose 1.5 m or more across by 4. A re-localisation has already passed that test in its search.
- The stack's default now re-joins only those corrections (`unique`); the R6 version (`verified`) stays available.
- On R6, now development data, both wrong matches failed the test: their scans put only 8 and 6 of the scored returns on a kerb. The gain where kerbs returned was kept ([development](benchmark/PROTOCOL_R7.md#what-development-did)).

R7's conditions were designed to produce large kerb corrections: correct ones when kerbs return after a blackout during which GNSS drifted, risky ones after dead reckoning with a gyro fault, and one condition adds a kerb map error. Each filter drove every episode without a re-join and with the unique re-join, and the kerb filters also with the R6 re-join: 3,400 episodes.

| # | Criterion | Held-out result | Verdict |
|---|---|---|---|
| X1 | Under `nominal_r7` every pair has identical traces | 200 of 200 | passed |
| X2 | Every pair of the GNSS-only filters has identical traces, in every condition | 840 of 840 | passed |
| X3 | With the unique re-join, the kerb filters cross no more often, no deeper, and fall back no more often, in every condition | under kerb faults `ekf_kerb2` had 26 episodes over a kerb against 25 (deepest 4.08 m in both), with 11 fallback stops against 19 | **failed** |
| X4 | Over the pairs acted on, fewer episodes end over a kerb or in a fallback stop | 69 pairs: 67 without, 63 with | passed |
| X5 | `ekf_kerb2` ends as the reference does at least as often in every condition | in every condition | passed |
| X6 | No closure incursion by `ekf_kerb2` that the unique re-join caused | none, with or without it | passed |

**What R7 showed.**
- **The gain held on unseen roads.** Where the kerbs returned after GNSS had drifted, `ekf_kerb2` ended 24 of 40 episodes as the reference did, against 18 without the re-join. After a kerb map error and the same blackout it ended 29 against 21. Its fallback stops fell from 21 to 15 and from 19 to 11.
  - Over all kerb-filter episodes the unique re-join changed 69: 15 fallback stops fewer, and 15 more episodes ending as the reference did.
  - It could not undo crossings made during the blackout: in 64 of the 69 episodes, the deepest crossing came before the first correction it followed. With the kerbs returning, 27 of 40 `ekf_kerb2` episodes went over a kerb with or without it.
- **It followed no wrong match.** Under the gyro fault, with neither kerbs nor GNSS over a quarter of the route, the unique re-join followed no correction at all.
  - The R6 version, driven on the same episodes, followed corrections in 68 of the 80 kerb-filter episodes.
  - It made 21 crossings shallower and 8 deeper, two of them by metres: `ekf_kerb` on R7-B, seed 9, from 6.06 to 17.60 m, and seed 5, from 6.75 to 9.59 m. That is R6's failure again, on unseen roads.
- **The map error was never followed.** Under kerb faults every followed correction came when the kerbs returned after the blackout; none came from the kerb section mapped 0.5 m wrong.
- **The failure (X3) followed two correct corrections**, the second of the two ways the protocol expected it could fail. R7-A under kerb faults, seed 7, `ekf_kerb2`: when the kerbs returned, the estimate was 1.4 m off sideways and 2.7 m along the road.
  - Without the re-join, the monitor read the 1.23 m correction as a tracking error, and its fallback stop held the car before the next junction, with the body on the road.
  - With the re-join, the car drove on. Kerbs on a straight cannot see along-road error. In the right turn at the tight J23 that error became a lateral error of up to 1.25 m, and the body went 0.88 m over the kerb. A re-localisation then corrected the estimate, and the car completed the route.
  - From the start of the blackout to the second correction, the filter reported a position sigma of at most 0.33 m. It had trusted the slowly drifting GNSS along the road.
- **Open loop** (reported without a criterion):
  - Under combined faults the kerb filters failed no run (`ekf_kerb2` lateral 95th percentile 0.047 m); `ekf_int2` failed 4 of 30 and `ekf_int` 25.
  - With the kerbs returning after GNSS drift, every filter failed some runs: the kerb filters 5 of 30 each, the GNSS-only filters 9 to 14.
  - Under the gyro fault every filter failed every run; median-run RMSE was 7.8 m for `ekf_kerb2` and 51.8 m for `ekf_kerb`.

**What this means.** Counting only unambiguous kerb matches did what it was meant to do. On unseen roads the re-join kept its gain where kerbs returned, and where the R6 version followed wrong matches into deeper crossings, it followed nothing. The one criterion it failed was failed by a correct correction. Followed, it let the car drive on with an along-road error the kerbs had not yet seen, and that the filter did not report. Without the re-join, the monitor's fallback stop had halted that car first. A lateral correction after GNSS drift is evidence that the along-road position is also wrong.

### Failure cases and the next improvement

#### After the second evaluation (R3)

**What it fixed, on unseen faults:**
- **Multipath capture.** An 8 m offset failed `ekf` in 30 of 30 runs; `ekf_int` and `ekf_kerb` in none.
- **Kerb crossings with good localisation.** `ekf_kerb` drove 200 episodes under five of the six conditions with the body always on the road and the reference's outcome, never stopping.
- **Lateral accuracy.** `ekf_kerb`'s lateral 95th percentile was 0.043–0.055 m under every condition but the slow excursion.

**What failed:**
- **Integrity without a reference (T1).**
  - Under combined faults, an unmodelled gyro-bias step (+0.010 rad/s at 35 s) lands just before the multipath. `ekf_int` excludes GNSS correctly (FAULTED), but dead-reckons with a growing heading error: 15° and 28 m by the end on R3-A.
  - When the multipath ends, the clean fixes match neither the drifted filter nor the remembered fault, so the fault never clears: the FAULTED state has no re-anchoring path.
  - `ekf_kerb` is unaffected: kerbs hold its heading and position.
- **A slow GNSS excursion (T8).**
  - A bias that grows at 0.25 m/s with no jump is invisible to the jump test. Both GNSS-only filters follow it (3 failures each; 35 of 40 closed-loop episodes over a kerb).
  - `ekf_kerb` held on R3-A and R3-C, but not in 5 of 10 runs on R3-B. There the excursion runs along a straight, where kerbs say nothing about along-track position, so the filter followed GNSS along the road. After the next turn that along-track error is lateral, and the filter rejected the kerbs that disagreed with it rather than GNSS.
  - In closed loop that put the body over a kerb in 8 episodes (up to 3.9 m), and a closure stop planned in the dragged frame ended past the barrier in 3. Every pose source had incursions under this condition: 21 in all, none for the reference.
- **The cost of the clearance layer with GNSS-grade localisation.** With a two-sigma margin on 3 m lanes, the GNSS-only filters stopped in every degraded-GNSS episode, most stops unwarranted (the true error was under 0.3 m). It removed their right-turn-exit crossings but not their departure crossings (3 of 40 nominal).

The third increment answered T1 with a way out of FAULTED, and T8 with kerb re-localisation. A persistent kerb-receiver disagreement now counts against the receiver after the kerbs win, but the receiver's slow error is still not a filter state.

#### After the third evaluation (R4)

**What it fixed, on unseen faults:**
- **Recovery after drift.** Under a gyro fault during multipath, `ekf_int` never left the fault (largest errors 30–70 m). `ekf_int2` left it in 29 of 30 runs, and its pooled 95th percentile was 4.80 m against 54.8 m.
- **The slow excursion.** `ekf_kerb2` failed no run and drove 40 of 40 episodes with the reference's outcome and no crossing. `ekf_kerb` had 1 failure, and 2 crossings (2.1 m deep) in closed loop.
- **Combined faults.** `ekf_kerb2`'s lateral 95th percentile was 0.045 m against 0.600 m. Driving, it had 1 crossing against 3 and ended 39 episodes as the reference did against 35.
- **No regression.** In every condition where nothing triggers them, the second versions equal the first; in no condition do they fail or cross more often.

**What failed or remains:**
- **Large pose corrections in the stack (U9).** The one crossing came after a correct mid-turn re-localisation: the estimate was right, but pure pursuit oscillated after the 0.8 m jump until the monitor's fallback stop, which ended with the body over the kerb. The same weakness turns a GNSS-return correction into a fallback stop in 6 of 10 kerb-fault episodes on R4-C. The localisation was right in both; the stack reads a corrected estimate as a tracking error. *Addressed afterwards, on development data:* see [pose corrections in the stack](#after-the-third-evaluation-pose-corrections-in-the-stack-development).
- **GNSS-only filters still drift while GNSS is excluded.** Under the gyro fault `ekf_int2` still exceeded 5 m in 20 of 30 runs before recovering. No GNSS rule can bound dead reckoning: only another absolute reference does, and kerbs did (`ekf_kerb2`: 0 failures, 0.045 m lateral).
- **One fault never ended.** The jump-back test compares the receiver's return with its onset jump. When the first biased fix fell short (3.4 m of a 6 m offset), the return did not match it.
- **Losing kerbs and GNSS together.** Under `kerb_faults_r4` the kerb filters stopped for clearance in every episode, 34 of the 40 stops unwarranted. The stop is the designed behaviour with no absolute reference; its cost is that the true error is usually small.
- **A slow excursion still drags every GNSS-only filter** (29 or 30 of 30 runs failed), as the R3 protocol expected: a slow excursion with no jump cannot be detected from GNSS and odometry alone.

**What the evidence pointed to next, and what was done.** The stack should treat a pose correction as a correction. It now re-joins the path smoothly after a jump, and its monitor measures from the path including the clearance shift (development results above). The monitor deliberately still sees a jump of more than 1 m: a version that hid it from the monitor drove a corrupted estimate 9.3 m off the road. A steering damper was not needed.

**After R6 and R7.** R6 showed that a re-join should follow only kerb matches that are unambiguous. The unique re-join does that, and R7 confirmed it on unseen roads (above).

**What the evidence supports now:**
- **Distrust the along-road position after a large lateral kerb correction.** On R7 the one crossing the re-join caused followed a correct 1.2 m lateral correction after GNSS drift. The same drift had left 2.7 m of along-road error, while the filter reported a sigma under 0.35 m. Inflating the along-road covariance by the size of such a correction would let the localisation policy slow the car until a junction's kerbs fix the along-road position. Claiming it would need a new held-out network, because R7 has now been used.
- **Earlier re-localisation for `ekf_kerb2`.** The remaining development crossing (R3-B, seed 2) came from a match found only mid-turn, one return short of the uniqueness margin.
- **A better fault offset for `ekf_int2`:** remember it from several fixes after the onset, not one.

## Not done

- **UKF:** not implemented. Every failure above is a model, integrity or stack gap (an unobservable bias, a missing recovery path, a controller's response to a pose jump), not linearisation error, so no question remains that a UKF would answer.
- **Real sensor data:** none. The kerb scanner is simulated against the same map the filter uses, up to a modelled 0.03 m deviation. One map error was tested on R4 and R7: a kerb section 0.5 m further out than mapped. Roadworks, kerbs moved inwards and lane markings were not.
- **Comparison with published results:** none is made. No public localisation result uses this simulator, these sensors and these metrics. Every comparison here is between the estimators run here, on identical inputs.
