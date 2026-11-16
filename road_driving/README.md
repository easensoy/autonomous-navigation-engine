# road_driving

A first road-driving loop on top of the engine. An approximately I-PACE-sized kinematic car follows lane-level routes from `Graph` and `NavigationCore` on a small synthetic left-hand-traffic network. When an oracle reports a roadworks closure, the car reroutes from where it actually is or stops before the barrier. Every episode is logged and can be replayed from its manifest.

Every result carries three labels: **SYNTHETIC_ROADS** (network R1, no real street), **GROUND_TRUTH_POSE** (the driving stack reads simulator truth, including speed and acceleration) and **ORACLE_CLOSURE** (the scenario tells the stack about the closure; nothing is sensed). There is no other traffic, and no perception, prediction or localisation estimate.

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
| Vehicle | `vehicle.*` | Kinematic bicycle about the rear axle. Wheelbase 2.99 m; steering up to 0.55 rad at 0.4 rad/s with a 0.2 s lag; 0.1 s command delay; 0.3 s longitudinal lag; braking up to 8 m/s²; no reverse |
| Driving stack | `stack.*` | Engine routing, the closure executive, a speed envelope (20 mph, and curvature caps at 1.9 m/s² lateral), stop guidance, pure pursuit steering, a constraint monitor and a fallback stop |
| World | `episode.cpp` | 100 Hz integration, the oracle notification and lift, and the logs |
| Evaluator | `evaluator.cpp` | Outcome and metrics from the truth log, the scenario and the map only |

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

The default build (`BUILD_ROAD_DRIVING=OFF`) is unchanged. CI enables the option in all four build jobs (Windows MSVC, Windows MSYS2 GCC, Ubuntu GCC and Clang), so CTest runs the three tests below on each.

## Results

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

CTest runs three tests with the option on:

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
- **`road_driving_regression_replay`** replays the stored failing episode.
- **`road_driving_replay_detects_change`** swaps that episode's variant label and requires the replay to fail.

## Episode files and replay

`--out` writes four files:
- `manifest.txt`: inputs, labels, engine commit, toolchain, outcome, metrics and hashes;
- `truth.csv`: 50 Hz truth and commands;
- `events.csv`: world and stack events;
- `routes.csv`: every route adopted, with the reason.

The trace hash is FNV-1a over `truth.csv`. `road_drive replay <dir>` reruns the episode from its manifest and compares the outcome, the route decisions and the trace hash; it prints the first differing truth line when the traces differ. When the recording toolchain is used, all three must match. On another toolchain the trace hash is reported but not required, because floating-point results are not assumed identical across maths libraries.

`regression/c4_d3_B0` is the deliberate failing episode: variant B0, notified 3 m before J10, recorded with MSVC 19.51. It replays byte for byte with GCC 14.2.

## Experimental scope

- **Network and inputs.** The roads are synthetic and pose is ground truth. The one closure is reported by an oracle, and there is no other road user.
- **Vehicle model.** The kinematic model ignores tyre forces. The I-PACE parameters are a proxy built from published dimensions, not a model of the vehicle.
- **Tuning.** The controller was tuned on R1 only, and no held-out network has been run. The settings are a look-ahead of 2.0 m + 0.30 s × speed, a 0.1 s pose prediction, and a speed envelope planned at 1.6 m/s² with a 0.4 s preview.
- **Comfort limits.** Jerk is not limited: braking onset reaches up to 26 m/s³ at an emergency stop. A replan whose required deceleration sits at the comfort boundary can peak slightly above 2.0 m/s² (2.03 measured).
- **Not measured.** Footprint-to-kerb clearance is not measured.
- **Engine commit.** The engine commit is recorded when the build is configured.
- **Deferred to later milestones:**
  - the held-out network R2;
  - the disconnected-network class C6;
  - campaign tooling with paired summaries;
  - perception, prediction and localisation estimation.
