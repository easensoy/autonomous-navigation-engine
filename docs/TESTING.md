# Tests, toolchains and CI

## Test programs

CTest runs 12 programs: 10 functional programs with 112 cases, and 2 benchmark smoke programs.

| Program | Cases | Covers |
|---|---:|---|
| `test_graph` | 6 | Removing two-way edges in both directions, edge counts, the revision counter |
| `test_a_star` | 20 | Optimality when edges cost less than their length, inconsistent heuristics, graph changes between searches, admissibility checks, negative weights, 100 random graphs against Dijkstra, equal-cost routes whose sums round apart (a five-node graph, and 1,339 queries on a racked grid against Dijkstra) |
| `test_dijkstra` | 17 | Shortest paths and distances, distances to all nodes, unknown and disconnected nodes, zero-weight edges |
| `test_bfs` | 15 | Fewest-edge paths, reachability, nodes at a given depth, all paths, trees |
| `test_bellman_ford` | 13 | Negative weights, negative-cycle detection, all shortest paths, unknown and disconnected nodes |
| `test_jump_point_search` | 3 | Routes made only of graph edges, empty routes for a graph without edges and for unknown nodes |
| `test_search_observer` | 9 | Attaching a `SearchObserver` never changes a route; expansions arrive in frontier order; every push names an already-expanded parent; Dijkstra settles each reachable node once; BFS never dequeues the goal; detaching stops events |
| `test_full_navigation` | 16 | Planning and stepping, waypoints, obstacle replanning, validation and optimisation, simulated execution, multi-goal planning, routes that revisit nodes, blocked goals |
| `test_environment_switching` | 11 | Switching graphs, navigation during a switch, saving and restoring, validating on a different graph |
| `test_emergency_stop` | 2 | An emergency stop whose error callback calls back into the executor |
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

Most of the test time goes to two programs:

- **`test_full_navigation`** waits on simulated execution.
- **`scability_tests`** seeds its graphs from the clock, so its run time varies between runs.

## Verified for this revision

On 8 November 2026, the library and tests as committed at `d732311` (the source state recorded in
`docs/data/manifest.json`) were built and tested on Windows 11 Pro 10.0.26200 (Intel Core Ultra 7
165H), Release, with CMake 4.3.1 and Ninja 1.13.2:

| Compiler | CTest programs | Total time |
|---|---|---:|
| MSVC 19.51.36260 (Visual Studio 2026) | 12 / 12 passed | 42.2 s |
| GCC 14.2.0 (MSYS2 UCRT64) | 12 / 12 passed | 33.2 s |

`scability_tests` took 32 s and 23 s of those, against 15 s and 14 s in the run before the A\* fix.
Its graphs are seeded from the clock, so its run time varies between runs; every other program took
the same time in both runs.

- **One earlier MSVC run failed.** During this session one MSVC run failed `test_bfs`'s one-second
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

`.github/workflows/ci.yml` runs on every push and pull request:

- **Windows, MSVC:** vswhere, vcvars and Ninja.
- **Windows, GCC:** MSYS2 UCRT64.
- **Ubuntu 24.04:** GCC and Clang.

Each job builds the library, the demo and every test, runs CTest, then runs the demo. The actions are
pinned to commit hashes.

**Status:** the workflow was published together with the README's evidence package. Its first run is
the first Linux build of this code, and the README badge shows the latest result. Before publishing,
the MSVC job's setup lines (`vswhere`, `vcvars64`) were checked locally in a `cmd` batch file, and
both pinned action commits were confirmed to match their release tags.
