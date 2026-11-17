# road_driving

A first road-driving loop on top of the engine. An approximately I-PACE-sized kinematic car follows lane-level routes from `Graph` and `NavigationCore` on small synthetic left-hand-traffic networks. When an oracle reports a roadworks closure, the car reroutes from where it actually is or stops before the barrier.

It can drive on simulator truth (the reference) or on its own state estimate. Simulated GNSS, IMU, wheel, steering and kerb-ranging sensors feed one of three EKF variants, and the stack drives on its output. It keeps a margin from the kerb that follows the estimate's uncertainty, and stops explicitly when no clear continuation exists. Every episode is logged and can be replayed from its manifest.

Labels:
- **SYNTHETIC_ROADS:** networks R1, R2 and R3, no real street.
- **ORACLE_CLOSURE:** the scenario tells the stack about the closure; nothing is sensed.
- **Pose source:** either **GROUND_TRUTH_POSE**, the simulator's truth, or **ESTIMATED_POSE**, an estimator fed by **SIMULATED_SENSORS**.

There is no other traffic, and no perception or prediction.

<img src="docs/media/test-drive-kerb.gif" width="100%" alt="Animated test drive. A simulated car seen from above turns right at a junction on a grid network, drawn with its real footprint. Yellow dots mark the kerb scanner's returns along both kerbs. In an orange-shaded GNSS fault zone the GNSS fixes scatter metres off the road, the GNSS indicator turns red and the status reads GNSS integrity faulted, kerb match matched. A dashed teal outline, the estimate the car drives on, stays on the car, a few centimetres off. An amber dashed line, the previous EKF replayed on the same sensor log, peels away metres off the road. A logarithmic error plot shows the driving estimate between 0.01 and 0.4 m and the previous EKF jumping to about 5 m. The car completes the route.">

<sub><b>Driving on kerb registration through a GNSS fault, in simulation:</b> on R2-B, seed 3, a 6 m multipath offset appears. Integrity marks the receiver faulted from 32.0 to 45.6 s and excludes it. Kerb returns hold the estimate within 0.37 m, and the car completes with the body on the road. The previous EKF, replayed on the same sensor log, ends 5.2 m off. The previous system drove this episode 4.36 m off the road. Independent research simulation on development data.<br>
[Localisation design and results](LOCALISATION.md) · [R3 protocol](benchmark/PROTOCOL_R3.md) · [Poster frame](docs/media/test-drive-kerb-poster.png) · [Earlier demonstration: a GNSS outage](docs/media/test-drive.gif)</sub>

## Localisation in brief

Two predeclared evaluations, each run once on a network held out of all tuning:

| | First (held out: R2) | Second (held out: R3) |
|---|---|---|
| Estimators | `ekf` against dead reckoning and two GNSS-only filters | `ekf`, `ekf_int` (GNSS integrity), `ekf_kerb` (integrity and kerb registration) |
| Criteria passed | 6 of 8 | 8 of 10 |
| Failed | multipath capture: a 6 m offset longer than the 10 s reset failed 8 of 210 runs | `ekf_int` under combined faults; a slow GNSS excursion dragging even `ekf_kerb` on one route |

On R3, every estimator processed the same recorded logs, 210 drives under seven conditions, six of them new fault types:
- **Multipath:** an 8 m offset failed `ekf` in 30 of 30 runs; `ekf_int` and `ekf_kerb` in none.
- **Lateral accuracy:** `ekf_kerb`'s pooled lateral 95th percentile was 0.043–0.055 m under every condition but the slow excursion, against 0.29 m for `ekf` on clean GNSS.
- **Outage:** when GNSS returned from an outage over 35–75% of the route, `ekf_kerb`'s median error was 0.045 m, against 0.646 m.
- **Driving (1,000 episodes):** on `ekf_kerb` the car ended all 200 episodes under nominal, outage, multipath, kerb-clutter and combined conditions as the reference did. The body was never over a kerb and there was no stop.

What failed:
- **Combined faults:** an unmodelled gyro-bias step during a GNSS fault left `ekf_int` dead-reckoning off by up to 44 m. Its fault state has no way back once the filter itself has drifted.
- **Slow excursion:** a GNSS offset growing slowly with no jump dragged the GNSS-only filters by metres (3 of 30 runs failed; 35 of 40 episodes over a kerb), and `ekf_kerb` in 5 of 10 runs on one route. Driving, that put `ekf_kerb`'s body over a kerb in 8 of 40 episodes, and caused 21 closure incursions across all pose sources.
- **Cost of the clearance layer:** with GNSS-grade localisation it stops in every degraded-GNSS episode, most stops unwarranted.

Details, the corrected first-evaluation record, and the next improvement: [LOCALISATION.md](LOCALISATION.md#results).

## How the engine is used

| Concern | Mechanism |
|---|---|
| Graph | One engine node per lane segment (at most 20 m) and per turning movement (connector), with the same ids. One-way edges exist only for legal moves, so one-way streets, the banned turn and the absence of U-turns are missing nodes or edges |
| Cost | Time to traverse the element entered at 20 mph, plus 1 s for a left turn and 4 s for a right turn (right turns cross the opposing lane in left-hand traffic), plus a deterministic tie-break below 1 ms |
| Heuristic | Nodes sit at their element's end point, so no edge is shorter in a straight line than the arc it enters, and the engine's rescaled A* heuristic stays admissible |
| Routing and replanning | `startNavigation` for the first route; `handleObstacles` with the closed segments, which replans from the engine's current node; `clearObstacles` and then `startNavigation` from the element the car occupies, after a lift |
| Step rule | `executeNavigationStep` each time the rear axle enters the next route element; the adapter checks that the engine's position matches the car's element |
| Engine limits | Found in an audit of the engine on 16 November 2026. A closure on the node the engine occupies is ignored, so the stack checks the occupied element itself and stops. Graph construction prints one line per node and edge, so `std::cout` is redirected during every engine call (about 13.6 kB per episode). Coordinates are validated against the ±10,000-unit node bound. `Graph::isConnected`, which is not valid on one-way graphs, is not used |
| Boundary | Only `src/engine_router.cpp` includes engine headers. The build enforces this by clearing the inherited include path for this directory |

## The loop

| Part | File | What it does |
|---|---|---|
| Network R1 | `lane_network.*` | 3 by 3 grid of junctions at x = 0, 120, 240 m and y = 0, 80, 160 m. Two-way streets with one 3.0 m lane each way, each lane left of its centreline; a one-way eastbound middle street; no right turn from the northbound approach at J11; a 60 m cul-de-sac with no turning head. Turns are a 5 m clothoid, an arc (7.5 m radius left, 10 m right) and a 5 m clothoid. A validator checks the map's invariants |
| Network R2 | `lane_network.*` | Held out for the first localisation evaluation, development data for the second. 4 by 3 grid at x = 0, 90, 240, 350 m and y = 0, 70, 170 m, from the same generator. Two one-way streets, two banned turns, 16 m junction mouths, 14 m right-turn arcs, and two tight junctions with 4 m kerbs and 5.5 m left-turn arcs taken at 2.5 m/s. Its geometry was fixed with the static footprint check alone (`road_drive map R2`) |
| Network R3 (held out) | `lane_network.*` | 4 by 4 grid at x = 0, 110, 200, 330 m and y = 0, 85, 190, 270 m, with R2's junction design: two one-way streets, two banned turns, two tight junctions and a 50 m cul-de-sac |
| Kerbs | `lane_network.*` | Every kerb as a line or a corner arc. Clearance is the exact distance to them, and a ray is intersected with them |
| Vehicle | `vehicle.*` | Kinematic bicycle about the rear axle. Wheelbase 2.99 m; steering up to 0.55 rad at 0.4 rad/s with a 0.2 s lag; 0.1 s command delay; 0.3 s longitudinal lag; braking up to 8 m/s²; no reverse |
| Sensors | `sensors.cpp` | GNSS at 10 Hz, IMU at 100 Hz, wheel speed and steering angle at 50 Hz, and a kerb-height range scanner at 10 Hz, each with its own latency, noise, bias and mounting. Sensor conditions add outages, multipath (constant, intermittent or slowly growing), wheel slip, IMU bias steps, delayed and dropped fixes, parked cars and kerb clutter. The same seed gives the same noise in every condition |
| Estimators | `estimators.cpp` | `ekf`: a 9-state EKF (pose, speed, gyro and accelerometer biases, wheel scale, two GNSS error states) with gating, robust weighting, a rewind buffer for late measurements, and a 10 s reset. `ekf_int`: the same with GNSS integrity instead of the reset. `ekf_kerb`: `ekf_int` plus scan-to-map kerb registration. Baselines: dead reckoning, a GNSS-only constant-velocity filter, and the latest fix held |
| Driving stack | `stack.*` | Engine routing, the closure executive, a speed envelope (20 mph, and curvature caps at 1.9 m/s² lateral), stop guidance, pure pursuit steering, a constraint monitor and a fallback stop. On estimated pose it adds a localisation policy that slows, stops and resumes on the estimate's covariance and age. A clearance layer keeps the footprint's margin to the kerb in line with that uncertainty: it shifts the path sideways or stops explicitly |
| World | `episode.cpp` | 100 Hz integration, the sensors and estimator in estimated mode, the oracle notification and lift, and the logs |
| Evaluator | `evaluator.cpp` | Outcome and metrics from the truth log, the scenario and the map only, including whole-vehicle clearance, crossing depth and lane association |
| Benchmark | `benchmark.cpp` | Open-loop scoring of every estimator on the same logs, closed-loop campaigns against the ground-truth reference, and the summaries in [`benchmark/`](benchmark) |

**The closure decision.** When closure X1 is reported, the stack sends its segments to `NavigationCore`, which replans from the element the car occupies. The stack accepts the new route only if the car can meet the new path's speed caps within 3.5 m/s², starting from its actual position and speed after a 0.4 s latency. Variant B1 otherwise masks that turning movement in the engine and asks again; variant B0 accepts, and the constraint monitor then stops the car. With no acceptable route the car keeps its lane and stops 2 m before the barrier. The stop is classed COMFORT (at most 2.0 m/s²), FIRM (at most 3.5 m/s²), EMERGENCY (full braking still stops short of the barrier) or UNAVOIDABLE.

**Stop guidance.** The braking command is the constant deceleration that stops at the target under the vehicle's own actuator model: the current acceleration holds for the command delay, then relaxes with the longitudinal lag. It is computed in closed form from the actual speed and acceleration, and recomputed at 50 Hz. A stop decided while the car is still accelerating out of a turn therefore needs no more braking than planned.

## Build, test, run

From the repository root. GCC (MSYS2 UCRT64, with `C:\msys64\ucrt64\bin` first on `PATH`):

```bash
cmake -S . -B build/road-gcc -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++ -DBUILD_ROAD_DRIVING=ON
cmake --build build/road-gcc
ctest --test-dir build/road-gcc --output-on-failure
```

MSVC, from a shell where `vcvars64.bat` has run: the same commands with `-DCMAKE_CXX_COMPILER=cl` and `build/road-msvc`.

```bash
build/road-gcc/road_driving/road_drive list
build/road-gcc/road_driving/road_drive run c3_r30 --variant B1 --seed 2 --out out/c3_r30_s2
build/road-gcc/road_driving/road_drive replay out/c3_r30_s2
build/road-gcc/road_driving/road_drive suite --variant B0 --seeds 6
```

Localisation:
- `drive` drives on an estimator (`ekf`, `ekf_int`, `ekf_kerb`; `--feasibility off` gives the stack without the clearance layer);
- `localise` replays the logged sensors through every estimator without the simulator;
- `bench-loc` and `bench-closed` run the campaigns. The splits are `dev` (R1), `heldout` (the first evaluation, R2), `r2` (R2 as development data) and `r3` (the second evaluation).

```bash
build/road-gcc/road_driving/road_drive drive R2-B --condition multipath_h --estimator ekf_kerb --seed 3 --out out/demo
build/road-gcc/road_driving/road_drive localise out/demo
build/road-gcc/road_driving/road_drive bench-loc --split r3 --seeds 10 --out out/r3
build/road-gcc/road_driving/road_drive bench-closed --split r3 --seeds 10 --out out/r3
```

The test-drive animation is rendered from that episode (Python with matplotlib, numpy and Pillow). `--compare ekf` draws the previous filter replayed on the same log:

```bash
build/road-gcc/road_driving/road_drive map R2 --json out/r2.json
python road_driving/tools/render_demo.py --map out/r2.json --episode out/demo --out out/test-drive-kerb.gif --poster out/test-drive-kerb-poster.png --poster-time 44 --compare ekf
```

The default build (`BUILD_ROAD_DRIVING=OFF`) is unchanged. CI enables the option in all four build jobs (Windows MSVC, Windows MSYS2 GCC, Ubuntu GCC and Clang), so CTest runs the four tests below on each. The Linux jobs also replay a stored sensor log through every estimator.

## Closure results on ground-truth pose

Measured on 16 November 2026 on one Windows 11 laptop, with GCC 14.2 (MSYS2 UCRT64) and MSVC 19.51, using the code in this directory. There are 16 scenarios, each run with 6 seeds and both variants: 192 episodes per compiler. Seed 0 is the nominal start; seeds 1 to 5 jitter the start by up to ±2 m along the lane, ±0.2 m across it and ±2°, and in C3 and C4 the notification distance by up to ±1 m. The start is in the eastbound lane 20 m east of J00 and the goal 60 m north of J21. X1 closes the east street's northbound lane with its barrier 40 m north of J20; after J10, the only legal continuation leads into it. Braking figures are measured while the car moves.

| Scenario | When X1 becomes known | Outcome | Measured |
|---|---|---|---|
| `c0_nominal` | never (no closure) | COMPLETED 12/12 | 50.1 to 50.4 s to goal; braking at most 1.86 m/s² |
| `c1_prior` | before departure | COMPLETED 12/12 | Detour left at J10, north to J12, west, south to J01, east along the one-way street, left at J21. For seed 0 this equals the independent search's best route under full knowledge |
| `c2_advance` | 60 m before J10 | COMPLETED 12/12 | Replans to the same detour |
| `c3_r80` … `c3_r20` | 80, 60, 45, 30, 20 m before the barrier | STOPPED_AT_CLOSURE 60/60 | COMFORT; 1.84 to 1.90 m from the barrier; braking at most 1.86 m/s² |
| `c3_r12` | 12 m before the barrier | STOPPED_AT_CLOSURE 12/12 | EMERGENCY; 2.85 to 4.87 m from the barrier; peak 7.91 m/s² |
| `c3_r6` | 6 m before the barrier | CLOSURE_INCURSION 12/12 | UNAVOIDABLE; up to 3.73 m past the barrier |
| `c4_d40`, `c4_d25`, `c4_d15` | 40, 25, 15 m before J10 | COMPLETED 36/36 | Replan to the detour. At 15 m the required deceleration sits at the comfort boundary, and braking peaks at 2.03 m/s² |
| `c4_d8` | 8 m before J10 | COMPLETED 12/12 | Firm replan; peak 3.37 m/s² |
| `c4_d3` | 3 m before J10 | B1: STOPPED_AT_CLOSURE 6/6; B0: STOPPED_EN_ROUTE 6/6 | B1 masks the left turn it cannot slow for and stops 1.84 to 1.90 m from the barrier. B0 accepts it and the monitor's fallback stop brakes at up to 7.93 m/s² |
| `c5_lift` | 45 m before the barrier; lifted 30 s after the car stops | COMPLETED 12/12 | Replans from the occupied element and completes in 87.7 to 88.6 s |

Across all 192 episodes:
- cross-track error is at most 0.162 m on links (this includes the start jitter of up to 0.2 m) and 0.063 m on connectors;
- lateral acceleration is at most 1.91 m/s², and speed is at most 0.06 m/s over the limit;
- there are no illegal moves and no off-lane samples.

GCC 14.2 and MSVC 19.51 produce byte-identical truth logs in all 192 episodes. The variants differ only in `c4_d3`.

CTest runs four tests with the option on:

- **`road_driving_tests`** (288 checks):
  - map invariants, including a mirrored right-hand copy that must be rejected;
  - engine routes against an independent Dijkstra on 200 queries, with and without closures;
  - routes unchanged at 1 and 10 graph units per metre;
  - vehicle kinematics and limits, and path continuity;
  - nominal and random-mission driving;
  - stop classes and braking distances against a numerical model of the same physics;
  - the commitment branch at J10, the closure lifecycle and determinism;
  - replay from a written episode;
  - an evaluator check on a hand-written truth log.
- **`road_driving_localisation_tests`** (95 checks):
  - sensor rates, latencies, arrival order and noise spread; no fix inside an outage zone; no brake-hold reading at rest;
  - every estimator replayed from `sensors.csv` without the simulator, matching the live estimates to 1e-9 m;
  - late and out-of-order GNSS giving the same EKF as in-order processing;
  - on nominal runs, EKF accuracy, NEES, a positive-definite covariance and a lower RMSE than every baseline;
  - multipath rejection, heading wrap, outage drift and recovery, and identical results on re-evaluation;
  - driving on the EKF: the nominal mission completes with no lane-association error, and an estimate without a covariance stops the car;
  - R2's map invariants, footprint clearance, slow tight turns and legal benchmark routes;
  - exact kerbs: the old function's sign everywhere, never smaller inside, on its zero set, rays stopping at the first kerb;
  - the kerb scanner: 10 Hz, returns on the mapped kerb, exact through the log, and kerb faults leaving the other sensors unchanged;
  - the GNSS fault generators: the ramp grows without a jump; the intermittent offset switches;
  - integrity: a persistent clean 6 m offset never trusted; a legitimate drift recovered by re-anchoring, not by a timer;
  - kerb registration: lateral 95th percentile below 0.10 m with a consistent NEES, including through an outage;
  - the clearance layer: no crossing at R2-B's right-turn exit where the previous stack crossed, an explicit stop where no margin is left, no effect on ground truth;
  - R3's map invariants, exact footprint clearance, legal routes and defined conditions.
- **`road_driving_regression_replay`** replays the stored failing episode.
- **`road_driving_replay_detects_change`** swaps that episode's variant label and requires the replay to fail.

## Episode files and replay

`--out` writes four files:
- `manifest.txt`: inputs, labels, engine commit, toolchain, outcome, metrics and hashes;
- `truth.csv`: 50 Hz truth and commands;
- `events.csv`: world and stack events;
- `routes.csv`: every route adopted, with the reason.

With a sensor condition it also writes `sensors.csv`, every measurement in arrival order, kerb scans included. In estimated mode it writes `estimates.csv` as well: the estimate the stack drove on, with its covariance, GNSS integrity state, kerb-match state and the clearance layer's path shift.

The trace hash is FNV-1a over `truth.csv`. `road_drive replay <dir>` reruns the episode from its manifest and compares the outcome, the route decisions and the trace hash; it prints the first differing truth line when the traces differ. When the recording toolchain is used, all three must match. On another toolchain the trace hash is reported but not required, because floating-point results are not assumed identical across maths libraries.

`regression/c4_d3_B0` is the deliberate failing episode: variant B0, notified 3 m before J10, recorded with MSVC 19.51. It replays byte for byte with GCC 14.2.

## Experimental scope

- **Network and inputs.** The roads are synthetic. The one closure is reported by an oracle, and there is no other road user.
- **Pose and sensors.** Pose is ground truth unless a run is labelled ESTIMATED_POSE. The sensors are simulated from truth with simple error models: GNSS, IMU, wheel speed, steering angle, and a planar range scanner at kerb height. The scanner sees only kerbs and objects standing in the carriageway; there is no point cloud, camera, or processed lidar or camera data. The receiver does not detect multipath.
- **Map.** The estimators and the world share the same kerb map, except for a smooth modelled deviation of 0.03 m RMS. Larger map errors, such as a moved kerb or roadworks, were not tested.
- **Vehicle model.** The kinematic model ignores tyre forces, so wheel slip exists only as a sensor error. The I-PACE parameters are a proxy built from published dimensions, not a model of the vehicle. Nothing here was driven on a physical vehicle.
- **Tuning.** The controller was tuned on R1. The controller settings are a look-ahead of 2.0 m + 0.30 s × speed, a 0.1 s pose prediction, and a speed envelope planned at 1.6 m/s² with a 0.4 s preview.
  - The first EKF was tuned on R1, and R2 was held out ([protocol](benchmark/PROTOCOL.md)).
  - Integrity, kerb registration and the clearance layer were developed on R1 and R2, and R3 was held out ([protocol](benchmark/PROTOCOL_R3.md)).
  - Each held-out network ran once with the frozen code.
- **Comfort limits.** Jerk is not limited: braking onset reaches up to 26 m/s³ at an emergency stop. A replan whose required deceleration sits at the comfort boundary can peak slightly above 2.0 m/s² (2.03 measured).
- **Clearance.** Measured exactly against the kerbs of the synthetic map: no kerb height, verge or parked car. The tightest places are right-turn exits on R2 and R3, where the footprint with mirrors, driven exactly, clears the far kerb by 0.023 m; on R1 it crosses by 0.11 m. Earlier figures came from a conservative function and are corrected in [LOCALISATION.md](LOCALISATION.md#kerbs-and-clearance).
- **Engine commit.** The engine commit is recorded when the build is configured.
- **Deferred to later milestones:**
  - the disconnected-network class C6;
  - campaign tooling with paired summaries for the closure scenarios;
  - perception and prediction;
  - kerb registration against map errors larger than the modelled deviation, and lane markings.
