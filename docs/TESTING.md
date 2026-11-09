# Tests, toolchains and CI

## Test programs

CTest runs 15 programs: 12 functional programs with 152 cases, one link-coverage program, and 2
benchmark smoke programs.

| Program | Cases | Covers |
|---|---:|---|
| `test_graph` | 6 | Removing two-way edges in both directions, edge counts, the revision counter |
| `test_a_star` | 20 | Optimality when edges cost less than their length, inconsistent heuristics, graph changes between searches, admissibility checks, negative weights, 100 random graphs against Dijkstra, equal-cost routes whose sums round apart (a five-node graph, and 1,339 queries on a racked grid against Dijkstra) |
| `test_dijkstra` | 17 | Shortest paths and distances, distances to all nodes, unknown and disconnected nodes, zero-weight edges |
| `test_bfs` | 15 | Fewest-edge paths, reachability, nodes at a given depth, all paths, trees |
| `test_bellman_ford` | 13 | Negative weights, negative-cycle detection, all shortest paths, unknown and disconnected nodes |
| `test_jump_point_search` | 16 | Three regression maps from the old implementation's failures; rejection of every graph outside the supported domain (a corner-cutting diagonal, nodes off the grid or sharing a cell, unequal or wrong weights, one-way or missing moves, diagonals when they are disabled); a non-unit step weight; marked cells; 7,200 seeded queries on 600 grids against Dijkstra |
| `test_search_observer` | 9 | Attaching a `SearchObserver` never changes a route; expansions arrive in frontier order; every push names an already-expanded parent; Dijkstra settles each reachable node once; BFS never dequeues the goal; detaching stops events |
| `test_full_navigation` | 16 | Planning and stepping, waypoints, obstacle replanning, validation and optimisation, simulated execution, multi-goal planning, routes that revisit nodes, blocked goals |
| `test_navigation_states` | 12 | `NavigationState` transitions: completion, a blocked goal, a cut-off goal, refusal to advance, pause, resume and emergency stop while `BLOCKED`, repeated failures, recovery by retry and by a new start, a failed start, an obstacle reported while `PAUSED`, `stopNavigation`, a successful replan |
| `test_environment_switching` | 11 | Switching graphs, navigation during a switch, saving and restoring, validating on a different graph |
| `test_emergency_stop` | 2 | An emergency stop whose error callback calls back into the executor |
| `test_graph_operations_api` | 15 | The members implemented in the API cleanup, against references: `GraphBuilder` grids, seeded random graphs, coordinate lists, radius and k-nearest connections and mazes; `NodeManager::moveNode` and `renameNode`; the minimum spanning forest (cycle property on 200 random graphs); bridges (brute force on 300 random graphs); cost-model files (exact round trip, malformed input); critical waypoints in smoothing |
| `test_api_link_coverage` | – | Takes the address of all 172 public member functions and constructors of the 15 classes whose declarations were audited, so a declared but undefined member fails the link. At run time it only prints the count |
| `algorithm_benchmarks`, `scability_tests` | – | Smoke runs that print timings; they fail only if they throw |

- **Failures reach CTest.** Each functional program exits non-zero when any of its cases fails.
- **No unit test enforces speed.** The few that time themselves fail only above one second. That
  catches a pathological slowdown without failing on a slower, shared CI machine.
- **The regression tests were checked against unfixed code.** The tests for the bugs fixed in
  November 2026 were run against the unfixed code first: 17 of their cases failed there with both
  compilers, and all of them pass now.
- **So were the two A\* tie-bug tests.** Before the
  [open-list fix](TECHNICAL_NOTES.md#a-tie-bug-found-by-the-traces), with both compilers, the
  five-node graph returned no route, and 34 of the racked grid's 1,339 queries came back costlier
  than Dijkstra's. With the fix, both pass: the grid reports 0 of 1,339.
- **The Jump Point Search regression maps** are the three smallest maps on which the previous
  implementation returned no route or a costlier one
  ([probe](TECHNICAL_NOTES.md#jump-point-search-domain-rules-and-validation)). The old class had a
  different interface, so the new test file cannot be compiled against it.

Most of the test time goes to two programs:

- **`test_full_navigation`** waits on simulated execution.
- **`scability_tests`** seeds its graphs from the clock, so its run time varies between runs.

## Verified for this revision

On 8 November 2026, the library and tests as committed at `361f788`, the benchmark run's source, were
built and tested on Windows 11 Pro 10.0.26200 (Intel Core Ultra 7 165H), Release, with CMake 4.3.1 and
Ninja 1.13.2. They are unchanged at `27da527`, the source state in `docs/data/manifest.json`: the
commits in between touch only Python evidence scripts.

| Compiler | CTest programs | Total time |
|---|---|---:|
| MSVC 19.51.36260 (Visual Studio 2026) | 15 / 15 passed | 42.3 s |
| GCC 14.2.0 (MSYS2 UCRT64) | 15 / 15 passed | 33.8 s |

Each functional program also reported its own count when run on its own: 152 of 152 cases passed, and
`test_api_link_coverage` linked 172 member functions of 15 classes. The machine was shared with other
work at the time, so these totals say nothing about speed; `scability_tests`, whose graphs are seeded
from the clock, takes most of the time and varies most between runs.

- **One earlier MSVC run failed.** In an earlier session one MSVC run failed `test_bfs`'s one-second
  guard (100 BFS searches took 2.2 s), while every MSVC test ran 20–100× slower than usual. A rerun
  passed.
- **The cause was a slow machine, not a defect.** The machine reported a 1.4 GHz clock at the time.
  The guard did its job, but a heavily throttled CI runner could trip it in the same way.

## Build commands

You need CMake 3.20 or newer and a C++17 compiler. Pass `-DBUILD_TESTS=OFF` to skip the tests, and
`-DBUILD_EVIDENCE_TOOLS=ON` to build `tools/evidence` (the README's figures).

**MSVC.** From the *x64 Native Tools Command Prompt for VS* in the Visual Studio 2026 Start-menu folder.
That prompt provides `cmake`, `ninja` and `ctest`.

```bat
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=cl
cmake --build build
ctest --test-dir build
build\AutonomousNavigationEngine.exe
```

**GCC (MSYS2 UCRT64).** From a prompt where `cmake` and `ninja` are on `PATH` and
`C:\msys64\ucrt64\bin` comes first. The tested setup used the CMake and Ninja that ship with Visual
Studio.

```bat
cmake -S . -B build-gcc -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++
cmake --build build-gcc
ctest --test-dir build-gcc
```

**Linux (GCC or Clang).** These are the CI's commands on Ubuntu 24.04:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++   # or clang++
cmake --build build --parallel
ctest --test-dir build
./build/AutonomousNavigationEngine
```

## Continuous integration

`.github/workflows/ci.yml` runs on every push and pull request. Its actions are pinned to commit
hashes. Seven jobs: five for the library and its evidence, two for the ROS 2 integration
([ros2/README.md](../ros2/README.md#tests-and-checks)).

- **Windows, MSVC** (vswhere, vcvars and Ninja), **Windows, GCC** (MSYS2 UCRT64), and **Ubuntu
  24.04, GCC** and **Clang**. Each builds the library, the demo and every test, runs CTest, then runs
  the demo.
- **Evidence checks** (Ubuntu 24.04). It installs nothing: the checkers need only Python's standard
  library, and the runner already has CMake and GCC.
  1. `check_outputs.py` recomputes every exported route from the exported edge lists, checks the
     workload fingerprints, the raw benchmark results (A\* at Dijkstra's cost on every query and
     replan), the run history and its selection rule, and every number and link in the README.
  2. `test_check_outputs.py` copies the evidence, corrupts it six ways (a route step that is not an
     edge, a wrong README number, a broken README link, an A\* result costlier than Dijkstra's, an
     expansion count that disagrees with its events, a run missing from the history) and requires
     `check_outputs.py` to fail on each.
  3. It builds `evidence_export` and `evidence_bench` without the tests.
  4. `compare_traces.py` runs `evidence_export` on the committed scenarios and requires the four
     committed traces back exactly, apart from the fields that name the compiler and the scenario path.

  It never runs the benchmark: CI runners are shared machines, and the README's timings come only from
  the machine documented in [EVIDENCE.md](EVIDENCE.md#measurement-protocol).
- **ROS 2 adapter core and simulation evidence** (Ubuntu 24.04, no ROS). Builds the ROS-independent
  adapter core (`ros2/ane_navigation/core`) and runs its unit tests and its closed-loop kinematic test;
  checks that the Gazebo world and the roadmap match the warehouse layout; runs `check_sim_evidence.py`,
  which replays every engine call recorded in the published simulation campaign through this
  checkout's engine and re-derives every acceptance verdict, and its mutation test.
- **ROS 2 Jazzy / Gazebo Harmonic** (Ubuntu 24.04, Docker). Builds the simulation image from
  `ros2/docker/Dockerfile`, builds and tests the four ROS 2 packages in it (`colcon test`: adapter-core
  tests, URDF validity, world/roadmap freshness, SDFormat validity), then runs scenario A once in
  headless Gazebo and requires its acceptance criteria and consistency checks. This is the only
  simulator-dependent job; its criteria are in simulation time, so the runner's speed does not matter.

The README badge shows the latest result.
