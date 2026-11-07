# Autonomous Navigation Engine

Classical graph planners in C++17: A\*, Dijkstra, breadth-first and depth-first search, Bellman–Ford and Jump Point Search. Around them sit route following, obstacle-aware replanning, path validation and console/SVG reporting. It builds and passes all 101 test cases with both MSVC and GCC on Windows.

The engine plans on graphs you build: nodes with 2-D coordinates, joined by weighted one-way or two-way edges. It does not model vehicle dynamics, time or perception. Routes are sequences of nodes, obstacles are node IDs that the caller reports, and execution is simulated one node per step. [Known limitations](#known-limitations) lists what is missing or unfinished.

- **Language:** C++17, built with CMake.
- **Size:** 33 public headers and 129 source files, about 45,600 lines, plus 11 test programs.
- **Tested with:** MSVC 19.51 (Visual Studio 2026) and GCC 14.2 (MSYS2 UCRT64) on Windows 11, using CMake 4.3 and Ninja 1.13.
- **Tests:** 9 functional test programs with 101 cases and 2 benchmark smoke programs, run by CTest. All pass with both compilers.
- **Not yet:** Linux and macOS builds, continuous integration.

## Algorithms

| Algorithm | Finds | Guarantee | Tests |
|---|---|---|---|
| A\* | Least-cost route | Optimal when edge weights are non-negative; a negative weight throws. The default heuristic is straight-line distance scaled to the graph's edge costs, so it never overestimates. Nodes are reopened when a cheaper route reaches them, so a custom heuristic only has to be admissible. | 18 cases, including agreement with Dijkstra on 100 random graphs |
| Dijkstra | Least-cost route, and distances to every node | Optimal for non-negative weights; a negative weight throws | 17 cases |
| Breadth-first search | Route with the fewest edges | Fewest edges, not least cost | 15 cases |
| Depth-first search | Some route | None: often far from the shortest | None yet |
| Bellman–Ford | Least-cost route when weights can be negative | Detects negative cycles and returns no route when it finds one | 13 cases |
| Jump Point Search | Route on an 8-connected grid graph | Every route it returns uses only graph edges, but routes are not always the shortest (see [Known limitations](#known-limitations)) | 3 cases |

When the start or goal is not in the graph, or the goal cannot be reached, every search returns an empty route except depth-first search. Depth-first search still throws `std::invalid_argument` for an unknown start or goal.

## Navigation and path operations

- **`NavigationCore`** plans a route, through waypoints if you give them. It follows the route one node per step and replans around obstacles reported on the way, keeping the part already travelled.
- **`DynamicReplanner`** keeps the current route while it stays clear. Otherwise it runs A\* on a copy of the graph with the blocked nodes removed. A blocked goal has no route.
- **`GlobalPathPlanner`** uses A\*, or Bellman–Ford when the graph has a negative weight. You can also ask for Dijkstra or Bellman–Ford explicitly.
- **`MultiGoalPlanner`** orders the goals by nearest neighbour, then improves the order with 2-opt. The tour is good but not guaranteed to be the best.
- **`PathValidator`** checks that every node exists and every step follows an edge. Its detailed check adds feasibility, safety and constraint reports.
- **`PathOptimizer`** removes loops and takes cheaper shortcuts. **`PathSmoother`** cuts corners, using only existing edges.
- **`PathExecutor`** simulates execution: one waypoint per step with a configurable delay, pause and resume, and an emergency stop.
- **`EnvironmentManager`** switches between graphs, keeps a short history, and saves environments to files and restores them.
- **Reporting** includes a console path printer, an ASCII renderer, SVG export, a logger and a timer.

## What a run looks like

The demo, `AutonomousNavigationEngine`, builds a 5 × 5 four-connected grid with 10-unit spacing. It finds a corner-to-corner route with A\*, Dijkstra and breadth-first search, follows that route, and validates and optimises it. Then it exercises environments, graph edits, multi-goal planning, algorithm selection, configuration and the console printer. This excerpt is from one run:

```text
=== Creating Sample Graph ===
…
Created graph with 25 nodes and 40 edges

=== Pathfinding Algorithm Comparison ===
A*: Found path with 9 nodes in 22 μs
Dijkstra: Found path with 9 nodes in 12 μs
…
BFS: Found path with 9 nodes in 236 μs

=== Navigation System Demo ===
Navigation started successfully
Navigation step 0 completed
…
Navigation step 7 completed
Navigation status: COMPLETED | mode GLOBAL_ONLY | at node 24 (9/9) | remaining 0
Current path length: 9 nodes

=== Path Operations Demo ===
…
Path validation: VALID
Optimized path length: 9 nodes
Smoothed path length: 9 nodes
Path cost: 80
…
Total distance: 8.00 units
…
Path Statistics:
Nodes: 9
Segments: 8
Edge-weight cost: 80.00
Geometric length (coordinate units): 80.00
Straight-line distance (coordinate units): 56.57
Efficiency: 70.71%
Turns: 5
Valid (every step follows an edge): yes
```

How to read the excerpt:

- **It is trimmed.** It comes from the 342 lines the demo prints. "…" marks removed lines, most of them per-step logging, and colour codes are stripped.
- **The timings are not a benchmark.** They come from one Release build on an Intel Core Ultra 7 165H and include console logging, which breadth-first search does at every step.
- **The two distance lines disagree by a factor of 10.** "Total distance: 8.00 units" and "Edge-weight cost: 80.00" describe the same route; [Known limitations](#known-limitations) explains why.

## Tests

```text
> ctest --test-dir build
…
 1/11 Test  #1: test_graph .......................   Passed    0.03 sec
 2/11 Test  #2: test_a_star ......................   Passed    0.07 sec
 3/11 Test  #3: test_dijkstra ....................   Passed    0.04 sec
 4/11 Test  #4: test_bfs .........................   Passed    0.05 sec
 5/11 Test  #5: test_bellman_ford ................   Passed    0.03 sec
 6/11 Test  #6: test_jump_point_search ...........   Passed    0.05 sec
 7/11 Test  #7: test_full_navigation .............   Passed    8.22 sec
 8/11 Test  #8: test_environment_switching .......   Passed    0.08 sec
 9/11 Test  #9: test_emergency_stop ..............   Passed    0.03 sec
10/11 Test #10: algorithm_benchmarks .............   Passed    0.42 sec
11/11 Test #11: scability_tests ..................   Passed   62.67 sec

100% tests passed, 0 tests failed out of 11

Total Test time (real) =  72.97 sec
```

This is one MSVC run on a fresh clone, with the `Start` lines omitted.

| Program | Cases | Covers |
|---|---|---|
| `test_graph` | 6 | Removing two-way edges in both directions, edge counts, the revision counter |
| `test_a_star` | 18 | Optimality when edges cost less than their length, inconsistent heuristics, graph changes between searches, admissibility checks, negative weights, 100 random graphs against Dijkstra |
| `test_dijkstra` | 17 | Shortest paths and distances, distances to all nodes, unknown and disconnected nodes, zero-weight edges |
| `test_bfs` | 15 | Fewest-edge paths, reachability, nodes at a given depth, all paths, trees |
| `test_bellman_ford` | 13 | Negative weights, negative-cycle detection, all shortest paths, unknown and disconnected nodes |
| `test_jump_point_search` | 3 | Routes made only of graph edges, empty routes for a graph without edges and for unknown nodes |
| `test_full_navigation` | 16 | Planning and stepping, waypoints, obstacle replanning, validation and optimisation, simulated execution, multi-goal planning, routes that revisit nodes, blocked goals |
| `test_environment_switching` | 11 | Switching graphs, navigation during a switch, saving and restoring, validating on a different graph |
| `test_emergency_stop` | 2 | An emergency stop whose error callback calls back into the executor |
| `algorithm_benchmarks`, `scability_tests` | – | Smoke runs that print timings; they fail only if they throw |

Each functional program exits non-zero when any of its cases fails, so CTest reports the failure. The regression tests for the bugs fixed in November 2026 were run against the unfixed code first. 17 of their cases failed there with both compilers, and all of them pass now.

Most of the test time goes to two programs:

- **`test_full_navigation`** waits on simulated execution.
- **`scability_tests`** took between 34 s and 63 s across three runs of the same code on this machine, partly because it seeds its graphs from the clock.

## Build and run

You need CMake 3.20 or newer and a C++17 compiler. These steps have only been tried on Windows 11.

**MSVC.** From the *x64 Native Tools Command Prompt for VS* in the Visual Studio 2026 Start-menu folder. That prompt provides `cmake`, `ninja` and `ctest`.

```bat
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=cl
cmake --build build
ctest --test-dir build
build\AutonomousNavigationEngine.exe
```

**GCC (MSYS2 UCRT64).** From a prompt where `cmake` and `ninja` are on `PATH` and `C:\msys64\ucrt64\bin` comes first. The tested setup used the CMake and Ninja that ship with Visual Studio.

```bat
cmake -S . -B build-gcc -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++
cmake --build build-gcc
ctest --test-dir build-gcc
```

The build produces the static library `NavigationEngine`, the demo and the 11 test programs. Pass `-DBUILD_TESTS=OFF` to skip the tests.

## Design notes

- **Why the default A\* heuristic is safe.** Let *k* be the smallest cost-per-length ratio over all edges, and let *h*(*n*) = *k* · |*n* − goal|. For any edge *u* → *v*, the triangle inequality gives *h*(*u*) ≤ *k* · (|*uv*| + |*v* − goal|) ≤ cost(*u*, *v*) + *h*(*v*). So *h* is consistent, and therefore admissible, whatever the non-negative edge costs are.
- **Caching against a revision counter.** Every change to a `Graph` (nodes, edges, or mutable node access) advances `Graph::getRevision()`. A\* computes *k* in one pass over the edges, caches it, and recomputes it only when the revision moves.
- **Failure contract.** The searches report "no route" as an empty vector. That covers an unknown start or goal, an unreachable goal, and, in Bellman–Ford, a negative cycle. Exceptions mark inputs an algorithm cannot handle, such as a negative weight in A\* or Dijkstra. Depth-first search does not follow this yet: it throws for an unknown start or goal.

## Known limitations

- **No vehicle model.** Planning happens on graphs only: there is no time, kinematics or trajectory. Obstacles are node IDs that the caller reports; there is no perception.
- **Mixed distance units.** `Node::euclideanDistance` multiplies coordinate distances by 0.1, so straight-line distances and edge weights can be in different units. The demo shows it: the console printer reports 8.00 units for a route whose edge-weight cost is 80.
- **Jump Point Search is not always shortest.** Every route it returns is valid. But on 300 random 8-connected grids with obstacles, its route was longer than Dijkstra's in 79 cases (by up to 2.4×), and it found no route in 4 cases where one existed. Its straight and diagonal jump rules come from different variants of the algorithm. Use A\* when the shortest route matters.
- **Experimental code.** Many source files compile into the library, but nothing in the public API, the demo or the tests reaches them. Examples are `DStarLite`, `DynamicWindowApproach` and `TravelingSalesmanSolver` in `src/navigation_strategies/`.
- **Declared but undefined methods.** These fail at link time if called:
    - the `GraphBuilder` and `LocalPathPlanner` constructors;
    - `GraphBuilder::buildGraph`;
    - `NodeManager::moveNode`;
    - `EdgeManager::getMinimumSpanningTree`;
    - `ConnectivityAnalyzer::getBridgeEdges`;
    - `MapUpdater::rollback`.
- **Placeholder scores.** The performance-based scoring in `AlgorithmSelector` uses simulated, random metrics, not measured ones.
- **Process-wide state.** 18 components keep a file-level singleton. One of them is the emergency-stop state, which the public API cannot reset.
- **Benchmarks are smoke tests.** `algorithm_benchmarks` and `scability_tests` seed their random graphs from the clock, build a new solver for every query, and do not measure memory. Their timings are therefore not comparable between runs.
- **Verbose output.** Most components print straight to standard output rather than through the logger, so their output cannot be filtered.
- **Compiler warnings.** A GCC Debug build adds `-Wall -Wextra -Wpedantic` and reports 76 warnings:
    - 56 unused parameters;
    - 14 unused variables;
    - 3 out-of-order member initialisers;
    - one each of unused-but-set-variable, sign-compare and maybe-uninitialized.

## Project layout

```text
include/                   public headers, one folder per module
src/
  core/                    Node, Edge, Graph
  pathfinding_algorithms/  A*, Dijkstra, BFS, DFS, Bellman–Ford, Jump Point Search
  navigation_strategies/   global planning, replanning and multi-goal planning, plus experiments
  autonomous_navigator/    NavigationCore, AlgorithmSelector, PathExecutor
  path_operations/         validation, optimisation, smoothing, cost calculation
  environment/             EnvironmentManager, MapUpdater, ObstacleDetector
  graph_operations/        node and edge managers, connectivity analysis, graph generators
  utilities/               configuration, maths helpers, priority queues
  reporting/               logging, timing, console, ASCII and SVG output
  main.cpp                 the demo
tests/
  core_tests/  algorithm_tests/  integration_tests/  performance_tests/
```

## Roadmap

Planned, not yet implemented:

1. Continuous integration with MSVC and GCC, and a `.gitignore` for build output.
2. Benchmarks with fixed seeds, solvers built once per graph, warm-up runs, percentiles and saved workloads.
3. A visual generated by the C++ code: a warehouse map, a corridor that becomes blocked, and the replanned route, labelled with real costs and timings.
4. Timed trajectories with a simple vehicle model, and evaluation of whole scenarios in closed loop.
5. A small learned scorer for candidate routes, compared against these planners as baselines.
