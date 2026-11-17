# road_driving

A first road-driving loop on top of the engine. An approximately I-PACE-sized kinematic car follows lane-level routes from `Graph` and `NavigationCore` on small synthetic left-hand-traffic networks. When an oracle reports a roadworks closure, the car reroutes from where it actually is or stops before the barrier.

It can drive on simulator truth (the reference) or on its own state estimate. Simulated GNSS, IMU, wheel, steering and kerb-ranging sensors feed one of five EKF variants, and the stack drives on its output. It keeps a margin from the kerb that follows the estimate's uncertainty, and stops explicitly when no clear continuation exists. Every episode is logged and can be replayed from its manifest.

Labels:
- **SYNTHETIC_ROADS:** networks R1 to R5, no real street.
- **ORACLE_CLOSURE:** the scenario tells the stack about the closure; nothing is sensed.
- **Pose source:** either **GROUND_TRUTH_POSE**, the simulator's truth, or **ESTIMATED_POSE**, an estimator fed by **SIMULATED_SENSORS**.

There is no other traffic, and no perception or prediction.

<img src="docs/media/test-drive-relocalise.gif" width="100%" alt="Animated test drive. A simulated car seen from above drives a route on a grid network, drawn with its real footprint. Yellow dots mark the kerb scanner's returns. In an orange-shaded GNSS fault zone the violet GNSS fixes drift steadily several metres to one side of the road, and the status reads GNSS integrity inconsistent, kerb match matched. A dashed teal outline, the estimate the car drives on, stays on the car. An amber dashed line, the first kerb filter replayed on the same sensor log, follows the drifting fixes off the road. A logarithmic error plot shows the driving estimate falling to about 0.01 m when it re-localises, while the first version's error climbs to about 4 m. The car completes the route.">

<sub><b>Kerb re-localisation under a slow GNSS excursion, in simulation:</b> on R3-B, seed 5, the GNSS offset grows at 0.25 m/s along a straight. Kerbs cannot see along-road error there. At the next turn the scans stop fitting the estimate. Within 0.5 s `ekf_kerb2` finds the pose they fit, re-localises (0.56 m to 0.01 m), and makes the receiver earn trust again. The car completes with the body on the road. The first version, replayed on the same sensor log, keeps rejecting the kerbs and drifts to 4.3 m. Independent research simulation on development data.<br>
[Localisation design and results](LOCALISATION.md) · [R4 protocol](benchmark/PROTOCOL_R4.md) · [Poster frame](docs/media/test-drive-relocalise-poster.png) · Earlier demonstrations: [a GNSS fault with kerbs](docs/media/test-drive-kerb.gif), [a GNSS outage](docs/media/test-drive.gif)</sub>

## Localisation in brief

Four predeclared evaluations, each run once on a network held out of all tuning:

| | First (held out: R2) | Second (held out: R3) | Third (held out: R4) | Fourth (held out: R5) |
|---|---|---|---|---|
| Tested | `ekf` against dead reckoning and two GNSS-only filters | `ekf`, `ekf_int` (GNSS integrity), `ekf_kerb` (integrity and kerb registration) | those, plus `ekf_int2` (a way out of GNSS faults) and `ekf_kerb2` (kerb re-localisation) | the stack's re-join after pose corrections, each filter driven with and without it |
| Criteria passed | 6 of 8 | 8 of 10 | 10 of 11 | 4 of 6 |
| Failed | multipath capture: a 6 m offset longer than the 10 s reset failed 8 of 210 runs | `ekf_int` under combined faults; a slow GNSS excursion dragging even `ekf_kerb` on one route | one `ekf_kerb2` episode of 280 over a kerb, after a correct re-localisation mid-turn | with GNSS-only filters, one more crossing episode in five cells and one completion fewer |

On R4, every estimator processed the same recorded logs: 210 drives under seven conditions, with new fault magnitudes and combinations and a kerb map error. The stack then drove 1,440 closed-loop episodes:
- **Recovery after drift:** under a gyro-bias step during multipath, `ekf_int` never left the fault (errors up to 70 m). `ekf_int2` left it in 29 of 30 runs, and its pooled 95th percentile was 4.80 m against 54.8 m.
- **Slow excursion:** `ekf_kerb2` failed no run and drove 40 of 40 episodes as the reference did, with no crossing (`ekf_kerb`: 1 failed run, 2 crossings). Its NEES was 2.0; the dragged first version's was 305.
- **Combined faults:** `ekf_kerb2`'s lateral 95th percentile was 0.045 m against 0.600 m, and it ended 39 of 40 episodes as the reference did (`ekf_kerb`: 35).
- **Kerb map error:** a kerb section 0.5 m from where the map has it triggered no re-localisation, and no run failed.
- **Elsewhere:** where nothing triggers them, the second versions equal the first, run for run.

What failed or remains:
- **The stack after a large pose correction (U9):** a correct 0.8 m re-localisation mid-turn left pure pursuit oscillating until the monitor's fallback stop. During that stop the body went 0.91 m over the kerb. The same weakness turned estimate corrections into 6 fallback stops under kerb faults, with no crossing.
  - *Addressed afterwards:* the stack now re-joins its path smoothly after a pose correction. Rerun on R4, now development data, `ekf_kerb2` crossed in none of 280 episodes, and the kerb-fault fallbacks fell from 6 to 2 ([details](LOCALISATION.md#after-the-third-evaluation-pose-corrections-in-the-stack-development)).
  - *Held out on R5 (fourth evaluation, 4 of 6 criteria):* that failure did not occur there. The kerb filters never crossed or made a fallback stop, with or without the re-join.
  - With GNSS-only filters, already metres wrong, the re-join changed which episodes crossed, in both directions. The net was one more crossing episode in five cells and one completion fewer ([details](LOCALISATION.md#fourth-evaluation-the-re-join-held-out-on-r5)).
  - A stack cannot tell a correction from a corruption by the jump alone.
- **GNSS-only drift:** while GNSS is rightly excluded, nothing bounds dead reckoning. `ekf_int2` still exceeded 5 m in 20 of 30 gyro-fault runs, and in one the fault never ended.
- **Losing kerbs and GNSS together** stopped the kerb filters for clearance in every such episode, 34 of the 40 stops unwarranted.

Details, the earlier evaluations and the next improvement: [LOCALISATION.md](LOCALISATION.md#results).

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
| Network R3 | `lane_network.*` | Held out for the second evaluation, development data for the third. 4 by 4 grid at x = 0, 110, 200, 330 m and y = 0, 85, 190, 270 m, with R2's junction design: two one-way streets, two banned turns, two tight junctions and a 50 m cul-de-sac |
| Network R4 | `lane_network.*` | Held out for the third evaluation, development data for the fourth. 5 by 3 grid at x = 0, 80, 170, 260, 380 m and y = 0, 100, 180 m, with R2's junction design: two one-way streets, two banned turns, two tight junctions and a 40 m cul-de-sac |
| Network R5 (held out) | `lane_network.*` | 3 by 5 grid at x = 0, 105, 230 m and y = 0, 75, 160, 235, 330 m, with R2's junction design: two one-way streets, two banned turns, two tight junctions and a 45 m cul-de-sac |
| Kerbs | `lane_network.*` | Every kerb as a line or a corner arc. Clearance is the exact distance to them, and a ray is intersected with them |
| Vehicle | `vehicle.*` | Kinematic bicycle about the rear axle. Wheelbase 2.99 m; steering up to 0.55 rad at 0.4 rad/s with a 0.2 s lag; 0.1 s command delay; 0.3 s longitudinal lag; braking up to 8 m/s²; no reverse |
| Sensors | `sensors.cpp` | GNSS at 10 Hz, IMU at 100 Hz, wheel speed and steering angle at 50 Hz, and a kerb-height range scanner at 10 Hz, each with its own latency, noise, bias and mounting. Sensor conditions add outages, multipath (constant, intermittent or slowly growing), wheel slip, IMU bias steps, delayed and dropped fixes, parked cars, kerb clutter, and a kerb section that is not where the map has it. The same seed gives the same noise in every condition |
| Estimators | `estimators.cpp` | `ekf`: a 9-state EKF (pose, speed, gyro and accelerometer biases, wheel scale, two GNSS error states) with gating, robust weighting, a rewind buffer for late measurements, and a 10 s reset. `ekf_int`: the same with GNSS integrity instead of the reset. `ekf_kerb`: `ekf_int` plus scan-to-map kerb registration. `ekf_int2` and `ekf_kerb2`: a confirmed way out of a GNSS fault, and kerb re-localisation when scans keep being rejected. Baselines: dead reckoning, a GNSS-only constant-velocity filter, and the latest fix held |
| Driving stack | `stack.*` | Engine routing, the closure executive, a speed envelope (20 mph, and curvature caps at 1.9 m/s² lateral), stop guidance, pure pursuit steering, a constraint monitor and a fallback stop. On estimated pose it adds a localisation policy that slows, stops and resumes on the estimate's covariance and age. A clearance layer keeps the footprint's margin to the kerb in line with that uncertainty: it shifts the path sideways or stops explicitly. When the estimate jumps, the controller re-joins the path smoothly instead of steering at the jump |
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
- `drive` drives on an estimator (`ekf`, `ekf_int`, `ekf_int2`, `ekf_kerb`, `ekf_kerb2`; `--feasibility off` gives the stack without the clearance layer, `--rejoin off` the stack without the re-join after pose corrections);
- `localise` replays the logged sensors through every estimator without the simulator;
- `bench-loc` and `bench-closed` run the campaigns. The splits are `dev` (R1), `heldout` (the first evaluation, R2), `r2` (R2 as development data), `r3` (the second evaluation), `r3dev` (R3 as development data), `r4` (the third evaluation) and `r5` (the fourth). `bench-closed --scenario` runs some of a split's scenarios; `tools/merge_closed_loop.py` merges such parts into the files one process writes. Each split drives the stack it was evaluated with; `bench-closed --rejoin on|off` overrides the re-join.

```bash
build/road-gcc/road_driving/road_drive drive R3-B --condition gnss_ramp_r3 --estimator ekf_kerb2 --seed 5 --out out/demo
build/road-gcc/road_driving/road_drive localise out/demo --estimators ekf_kerb,ekf_kerb2
build/road-gcc/road_driving/road_drive bench-loc --split r4 --seeds 10 --out out/r4
build/road-gcc/road_driving/road_drive bench-closed --split r4 --seeds 10 --scenario R4-A --out out/r4-cl/R4-A
```

The test-drive animation is rendered from that episode (Python with matplotlib, numpy and Pillow). `--compare ekf_kerb` draws the first kerb filter replayed on the same log:

```bash
build/road-gcc/road_driving/road_drive map R3 --json out/r3.json
python road_driving/tools/render_demo.py --map out/r3.json --episode out/demo --out out/test-drive-relocalise.gif --poster out/test-drive-relocalise-poster.png --poster-time 50 --compare ekf_kerb
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
- **`road_driving_localisation_tests`** (142 checks):
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
  - R3's and R4's map invariants, exact footprint clearance, legal routes and defined conditions, and R4's closure lane on its route;
  - the kerb map error: only returns from the left kerb inside its zone change, and only outward;
  - the second versions: equal to the first on clean data, trace for trace; the persistent offset still never trusted; drift still recovered. On an R3 combined-fault drive `ekf_int2` leaves the fault and ends within 1 m, where `ekf_int` ends 28 m off. On an R3 slow-excursion drive `ekf_kerb2` re-localises and keeps its lateral 95th percentile below 0.2 m;
  - pose corrections: a mid-turn re-localisation, and GNSS corrections next to a shifted path, end without a fallback stop and with the body on the road;
  - the re-join switch: off reproduces the recorded fallback over the kerb, and on simulator truth it changes nothing;
  - R5's map invariants, exact footprint clearance, legal routes, closure lane and defined conditions.
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
