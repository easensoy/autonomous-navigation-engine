# Changes

Compatibility notes for code that uses the library. Dates are when the change reached `main`.

## November 2026

### Road-driving: the stack re-joins its path after a pose correction (`road_driving/`)

- **Behaviour change in estimated mode only.** When the estimate jumps sideways by more than 0.15 m between
  two control ticks, the controller's reference moves with it and returns to the path along a raised
  cosine (`rejoinpolicy` in `src/stack.hpp`). A `POSE_CORRECTION` event records each one. On simulator
  truth nothing jumps: R1's 192 closure episodes stay byte-identical.
- **Monitor.** Its 1 m lateral limit is now measured from the path plus its clearance shift (before: from
  the unshifted path). It does not use the re-join, so a larger jump still triggers the fallback stop.
- **Tests:** `road_driving_localisation_tests` grows from 121 to 123 checks.

### Road-driving: second filter versions, network R4 and a kerb map error (`road_driving/`)

- **Additive.** With `BUILD_ROAD_DRIVING` off, the build is unchanged. No engine source, public header,
  file under `ros2/`, or road-driving stack file changed.
- **New estimators** (`makeEstimator`):
  - `ekf_int2`: `ekf_int` with a confirmed way out of a GNSS fault (a jump back, then 20 clean fixes)
    and re-anchoring on the track angle;
  - `ekf_kerb2`: the same, plus kerb re-localisation by a grid search when scans keep being rejected.

  `ekf`, `ekf_int` and `ekf_kerb` are unchanged: their traces on stored logs are byte-identical. On
  clean data each second version equals its first version exactly (a test compares the traces).
- **Statistics.** `EstimatorStats` and `LocMetrics` gain `faultRecoveries`, `headingReanchors` and
  `kerbRelocalisations`; `runs.jsonl` lines gain the three fields.
- **Network R4** (`buildR4`, `buildNetwork("R4")`) and its routes and closure scenario.
- **Faults.** A kerb map error (`SensorConditions::kerbShift`): a kerb section lies further out than
  mapped. Its zones are written to the manifest only when present, so earlier logs are unchanged. Seven
  R4 conditions (`conditionNamesR4`).
- **`road_drive`:**
  - `bench-loc` and `bench-closed` gain the splits `r3dev` (R3 as development data) and `r4`;
  - `bench-closed --scenario A,B` runs a subset of a split's scenarios, and
    `tools/merge_closed_loop.py` merges per-scenario parts into the files one process writes;
  - `map` knows R4 and prints each network's benchmark routes.
- **Tests:** `road_driving_localisation_tests` grows from 95 to 121 checks.

### Road-driving: GNSS integrity, kerb registration and a clearance layer (`road_driving/`)

- **Additive.** With `BUILD_ROAD_DRIVING` off, the build is unchanged. No engine source, public header
  or file under `ros2/` changed.
- **Clearance is now exact.** `LaneNetwork::roadClearance` measures the distance to explicit kerb lines
  and corner arcs. The previous function, kept as `coarseClearance`, has the same sign but understated
  clearance near rounded corners by up to 3.6 m. Recorded R2 episodes keep their traces, outcomes and
  body-crossing counts; positive clearances grow.
- **New estimators** (`makeEstimator`):
  - `ekf_int`: GNSS integrity (jump test against odometry, fault exclusion, consistency-checked
    re-anchoring) instead of the 10 s reset;
  - `ekf_kerb`: `ekf_int` plus scan-to-map kerb registration.

  `ekf` is unchanged and reproduces its recorded results exactly.
- **New sensor and faults.** A simulated kerb-height range scanner, on its own random stream, so earlier
  logs and results are unchanged. New faults: intermittent multipath, a slow GNSS excursion, parked cars,
  kerb clutter and scanner blackout. `sensors.csv` gains a seventh column for scan ranges; six-column
  logs still parse.
- **Stack.** A clearance layer: the footprint's margin to the kerb follows the estimate's uncertainty;
  the path shifts away from close kerbs; the car stops explicitly where no clear continuation exists.
  `Scenario::feasibility` turns it off; manifests written before it existed replay with it off. On
  simulator truth, R1's 192 closure episodes are byte-identical.
- **`road_drive`:**
  - `drive` gains `--estimator` and `--feasibility`;
  - `localise` gains `--estimators` and `--config`;
  - `bench-loc` and `bench-closed` gain the splits `r2` (R2 as development data) and `r3` (held out);
  - `map` knows R3.
- **Tests:** `road_driving_localisation_tests` grows from 42 to 95 checks.

### Road-driving localisation (additive, `road_driving/`)

- **Additive.** With `BUILD_ROAD_DRIVING` off, the build is unchanged. No engine source, public header
  or file under `ros2/` changed. Ground-truth driving is unchanged: the stored regression episode
  replays byte for byte.
- **Contents:**
  - simulated GNSS, IMU, wheel-speed and steering sensors with fault conditions;
  - a 9-state EKF and three baselines: dead reckoning, a GNSS-only constant-velocity filter, and the
    latest fix held;
  - an estimated-pose mode, in which the stack drives on the EKF through an explicit interface and a
    localisation policy;
  - whole-vehicle clearance and lane-association metrics;
  - the held-out network R2;
  - open- and closed-loop benchmark campaigns under a predeclared protocol;
  - a renderer for the recorded test drive.

  See [road_driving/LOCALISATION.md](road_driving/LOCALISATION.md).
- **`road_drive`:** new commands `drive`, `localise`, `map`, `bench-loc` and `bench-closed`. Episode
  manifests gain optional keys: network, sensor condition, pose mode, estimator and departure delay.
  Older manifests still replay.
- **Tests:** CTest gains `road_driving_localisation_tests`, so the extension has four entries. The
  Linux CI jobs also replay a stored sensor log through every estimator.

### Road-driving extension (new, `road_driving/`)

- **Additive and off by default.** The root `CMakeLists.txt` gains `option(BUILD_ROAD_DRIVING ... OFF)`;
  with it off, the build is unchanged. No engine source or public header changed. The extension uses
  `Graph` and `NavigationCore` only through their public API (`addNode`, `addEdge`, `startNavigation`,
  `executeNavigationStep`, `handleObstacles`, `clearObstacles`, `retryNavigation`, the state and route
  getters). Only `road_driving/src/engine_router.cpp` includes engine headers.
- **Contents:** a synthetic left-hand-traffic lane network, a kinematic vehicle with approximately
  I-PACE dimensions, a driving stack that replans around an oracle-reported closure or stops before
  it, an evaluator that reads only the truth log, episode logs with replay, the `road_drive` tool and
  the `test_road_driving` program. See [road_driving/README.md](road_driving/README.md).
- **Tests:** with the option on, CTest gains three entries. CI turns the option on in the four build
  jobs.

### ROS 2 and Gazebo integration (new, `ros2/`)

- **Additive.** No engine source or public header changed. The integration links the engine as built
  from this repository and drives it only through `NavigationCore`'s public API (`startNavigation`,
  `executeNavigationStep`, `handleObstacles`, `pauseNavigation`/`resumeNavigation`, `emergencyStop`,
  `retryNavigation`, the state and route getters).
- **Packages (ROS 2 Jazzy):** `ane_interfaces` (navigator status and events, blockage reports, mission
  start, fault injection), `ane_navigation` (the `graph_navigator` adapter, the `cmd_vel_gate` stop
  mechanism, the `fault_injector` test instrument, and `core/`, a ROS-independent C++ library with its
  own unit and closed-loop tests that also builds without ROS), `ane_description` (the `ane_amr`
  differential-drive URDF/Xacro with Gazebo Harmonic plugins) and `ane_sim` (warehouse world and
  roadmap generated from one layout file, launch files, RViz2 view, scenarios, trial and analysis
  scripts).
- **Environment:** `ros2/docker/Dockerfile` (ROS 2 Jazzy, Gazebo Harmonic, software rendering).
  Documentation, contracts and results: [ros2/README.md](ros2/README.md).
- **CI:** two jobs: the adapter core and the simulation evidence without ROS, and the ROS 2 packages
  plus one headless Gazebo trial in the container image.

### `AStar`: open-list fix

- **Behaviour.** `findPath` no longer drops a node whose stale and live frontier entries tie on f.
  Before the fix, A\* could return a costlier route than Dijkstra's, or no route at all, when two
  equal-cost routes to a node summed to values one rounding step apart (details in
  [docs/TECHNICAL_NOTES.md](docs/TECHNICAL_NOTES.md#a-tie-bug-found-by-the-traces)).
- **Private API.** `OpenListManager` is a plain priority queue with lazy deletion. The private helpers
  `isInOpenList` and `removeFromOpenList` were removed, and `getOpenListSize()` now counts frontier
  entries, including stale ones, so `estimateMemoryUsage()` reports the memory the queue actually holds.

### `SearchObserver` (new)

- `AStar`, `Dijkstra` and `BFS` accept an optional, read-only observer through
  `setSearchObserver(SearchObserver*)`. With no observer attached (the default), each hook is one
  pointer test and routes are unchanged.

### `JumpPointSearch`: new contract

- **Supported domain, enforced.** JPS now accepts only uniform-cost grid graphs in its movement model:
  one node per integer cell; edges exactly the orthogonal moves (weight s) and, with diagonal movement
  enabled, the diagonal moves whose two side cells hold nodes (weight s * sqrt(2)); every edge in both
  directions. `findPath` and `getJumpPoints` throw `std::invalid_argument` naming the first violation
  for any other graph; `supportsGraph(std::string*)` reports it without throwing. Graphs that relied
  on corner-cutting diagonals, weighted cells or one-way edges must use `AStar` or `Dijkstra`.
- **Results.** On supported graphs `findPath` returns a least-cost route; with no marked cells its
  cost equals Dijkstra's. The previous implementation returned no route for 1,319 of 2,379 sampled
  8-connected queries and 303 of 1,189 sampled 4-connected queries that had one, and a costlier route
  for 323 more 4-connected queries.
- **Marked cells.** `addObstacle` closes a cell: routes avoid it and no diagonal passes its corner.
- **New members.** `supportsGraph`, `getPathCost` (sum of edge weights), `getLastSearchExpansions`.
  `getPathLength` is unchanged (sum of `Node::euclideanDistance`, which is 0.1 x the coordinate distance).
- **Removed private members.** The unused helper classes (`JumpPointIdentifier`, `GridPruningManager`) and
  about thirty private method declarations that had no callers. The class no longer prints to `std::cout`.

### `NavigationCore`: explicit states and failed-replan semantics

- **New.** `enum class NavigationState { IDLE, ACTIVE, PAUSED, COMPLETED, BLOCKED }`,
  `getNavigationState()`, `navigationStateName()`, `getInvalidatedRoute()`, `getGoal()`,
  `retryNavigation()`, `clearObstacles()`, `clearAllObstacles()`.
- **Failed replan.** When `handleObstacles` leaves no route, the navigator enters `BLOCKED` (it used to
  report `IDLE`). In `BLOCKED`, `getCurrentPath()` returns only the travelled part, ending at the current
  node (it used to return the old route through the blocked nodes); `getInvalidatedRoute()` returns the
  abandoned route ahead; `getRemainingDistance()` returns infinity; `executeNavigationStep()` returns
  false. `getNavigationStatus()` starts with `BLOCKED` and its console message names the node and goal.
- **Recovery.** `retryNavigation()` replans from the current node to the goal around the obstacles still
  known, for example after `clearObstacles()`; a new `startNavigation()` also replaces a blocked navigation.
- **Other transitions.** A failed `startNavigation` now leaves the current navigation unchanged (it used
  to deactivate it). `emergencyStop()` pauses only an `ACTIVE` navigation (it used to mark any navigation
  with a route as paused). `isNavigationComplete()` is true exactly in `COMPLETED`.

### Public headers: declared but undefined members resolved

An audit of every header under `include/` against the symbols the library defines found 84 member
functions that were declared and never defined (64 public, 20 private); calling any of them failed at
link time. Each was implemented where the class supports a precise contract and removed otherwise.
`tests/api_tests/test_api_link_coverage.cpp` now takes the address of every public member function of
the 15 changed classes (172 functions), so an undefined declaration fails the build.

- **Implemented**, each with behavioural tests in `tests/api_tests/test_graph_operations_api.cpp`:
  - `GraphBuilder`: the constructor (it was undefined, so the class could not be used),
    `buildGraph` for `GRID` and `RANDOM` (with an optional `seed`), `addNodesFromCoordinates`,
    `connectNearestNeighbors`, `connectWithinRadius`, `generateMazeLayout`. `buildGraph` and
    `generateMazeLayout` require an empty graph and reject missing, unknown or out-of-range parameters
    with `std::invalid_argument`.
  - `NodeManager::moveNode`, `renameNode`.
  - `EdgeManager::getMinimumSpanningTree` (Kruskal; a spanning forest for a disconnected graph).
  - `ConnectivityAnalyzer::getBridgeEdges`; its return type changes from `std::vector<int>`, which
    cannot name an edge, to `std::vector<std::pair<int, int>>`.
  - `CostCalculator::saveCostModel`, `loadCostModel` (a line-oriented text format; a failed load leaves
    the model unchanged).
  - `PathSmoother::preserveCriticalWaypoints`.
- **Removed**, because no contract follows from the class's state or the declaration duplicated an
  existing member:
  - `LocalPathPlanner`, the whole class: no member was ever defined, and its methods (dynamic window,
    potential fields, pure pursuit) need a vehicle model the library does not have. The unused
    `NavigationCore` member that held one is gone.
  - `GraphBuilder`: `loadFromFile`, `saveToFile`, `generateWarehouseLayout`, `generateCityRoadNetwork`,
    `loadFromJSON`, `saveToJSON`, `importFromOSM`, `validateGraphStructure`, `optimizeGraphLayout`,
    `generateGraphStatistics`; and the `GraphType` values `DELAUNAY`, `VISIBILITY`, `ROADMAP`, `CUSTOM`,
    which numeric parameters cannot describe.
  - `NodeManager`: `optimizeNodePositions`, `mergeNodes`, `redistributeNodes`.
  - `EdgeManager`: `findRedundantEdges`, `removeRedundantEdges`, `addMissingConnections`,
    `optimizeEdgeWeights`, `createSpanningTree`, `ensureConnectivity`.
  - `ConnectivityAnalyzer`: `findBottleneckEdges`, `calculateConnectivityRobustness`,
    `suggestAdditionalConnections`, `optimizeConnectivity`.
  - `MapUpdater`: `updateMap`, `rollbackUpdate`, `optimizeMapStructure`, and `processSensorData`, which
    was defined but called `updateMap`, so it could not link either. Its source,
    `sensor_data_integration.cpp`, and the unreachable `incremental_map_update.cpp` were deleted.
  - `ObstacleDetector`: `enableContinuousDetection`, `hasObstaclesChanged`.
  - `MultiGoalPlanner`: `setPriorityGoals`, `setTimeLimit`, `enableApproximation`.
  - `PathOptimizer`: `enableIterativeOptimization`, `setOptimizationIterations`.
  - `PathSmoother`: `enableCornerSmoothing`, `setMinimumSegmentLength`.
  - `PathVisualizer`: `highlightNodes`, `showAlgorithmExploration`, `animatePathExecution`. They name no
    output format, and exploration order is available exactly through `SearchObserver`.
  - `PerformanceProfiler`: `setBenchmarkBaseline`, `enableContinuousProfiling`, `setMemoryTrackingPrecision`.
  - `ConfigManager`: `setDefaultValues`.
  - Private declarations with no definition in `BFS` (6), `DFS` (6), `PathSmoother` (3) and
    `MultiGoalPlanner` (2).
- **Unit change.** The random-graph generator behind `GraphBuilder::buildGraph(RANDOM)` now weights edges
  by straight-line coordinate distance instead of `Node::euclideanDistance` (0.1 x that), like every
  other `GraphBuilder` method. It was unreachable before, because the constructor was undefined.

### Build

- `find_package(Threads)` links the platform thread library; Linux builds need it.
- `-DBUILD_EVIDENCE_TOOLS=ON` builds `tools/evidence` (off by default).
