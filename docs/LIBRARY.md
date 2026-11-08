# What is in the library

The inventory that used to open the README. For what the library does on a concrete scenario, see the
[README](../README.md); for known gaps, [LIMITATIONS.md](LIMITATIONS.md).

The engine plans on graphs you build: nodes with 2-D coordinates, joined by weighted one-way or two-way
edges. It has no model of vehicle dynamics, time or perception:

- routes are sequences of nodes;
- obstacles are node ids the caller reports;
- execution is simulated, one node per step.

## Searches

| Algorithm | Finds | Guarantee | Tests |
|---|---|---|---|
| A\* | Least-cost route | Edge weights must be non-negative; a negative weight throws. The default heuristic is straight-line distance scaled to the graph's edge costs, so it never overestimates ([why](TECHNICAL_NOTES.md#why-the-default-a-heuristic-is-safe)). The open list keeps one entry per improvement and the search skips stale ones, so equal-cost routes whose sums differ by one rounding step cannot hide a node ([fixed tie bug](TECHNICAL_NOTES.md#a-tie-bug-found-by-the-traces)). | 20 cases, including agreement with Dijkstra on 100 random graphs and on 1,339 queries full of equal-cost ties |
| Dijkstra | Least-cost route, and distances to every node | Optimal for non-negative weights; a negative weight throws. Settles every reachable node, with no early exit at the goal. | 17 cases |
| Breadth-first search | Route with the fewest edges | Fewest edges, not least cost | 15 cases |
| Depth-first search | Some route | None: often far from the shortest | none yet |
| Bellman–Ford | Least-cost route when weights can be negative | Detects negative cycles and returns no route when it finds one | 13 cases |
| Jump Point Search | Route on an 8-connected grid graph | Every route it returns uses only graph edges, but routes are not always the shortest ([LIMITATIONS.md](LIMITATIONS.md)) | 3 cases |

A\*, Dijkstra and BFS accept an optional, read-only `SearchObserver` that receives their expansions and
frontier pushes in execution order ([TECHNICAL_NOTES.md](TECHNICAL_NOTES.md#search-observer); 9 test
cases).

When the start or goal is not in the graph, or the goal cannot be reached, every search returns an empty
route, except depth-first search, which throws `std::invalid_argument` for an unknown start or goal.

## Navigation and path operations

- **`NavigationCore`** plans a route, through waypoints if given. It follows the route one node per
  step and replans around obstacles reported on the way, keeping the part already travelled.
- **`DynamicReplanner`** keeps the current route while it stays clear. Otherwise it runs A\* on a copy
  of the graph with the blocked nodes removed ([cost of that](TECHNICAL_NOTES.md#where-a-replans-time-goes)).
  A blocked goal has no route.
- **`GlobalPathPlanner`** uses A\*, or Bellman–Ford when the graph has a negative weight. Dijkstra or
  Bellman–Ford can also be requested explicitly.
- **`MultiGoalPlanner`** orders the goals by nearest neighbour, then improves the order with 2-opt.
  The tour is good but not guaranteed to be the best.
- **`PathValidator`** checks that every node exists and every step follows an edge. Its detailed
  check adds feasibility, safety and constraint reports.
- **`PathOptimizer`** removes loops and takes cheaper shortcuts. **`PathSmoother`** cuts corners,
  using only existing edges.
- **`PathExecutor`** simulates execution: one waypoint per step with a configurable delay, pause and
  resume, and an emergency stop.
- **`EnvironmentManager`** switches between graphs, keeps a short history, and saves environments to
  files and restores them.
- **Reporting:** a console path printer, an ASCII renderer, SVG export, a logger and a timer.

## The demo program

`AutonomousNavigationEngine` (`src/main.cpp`) works on a 5 × 5 four-connected grid with 10-unit spacing:

1. It finds a corner-to-corner route with A\*, Dijkstra and breadth-first search.
2. It follows that route.
3. It validates and optimises the route.
4. It exercises environments, graph edits, multi-goal planning, algorithm selection, configuration and
   the console printer.

An excerpt from one run:

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

The excerpt is trimmed from the 342 lines the demo prints. "…" marks removed lines, mostly per-step
logging, and colour codes are stripped.

- **The demo's timings are not a benchmark.** They include console logging, which BFS does at every
  step. For measurements, see [EVIDENCE.md](EVIDENCE.md).
- **"Total distance: 8.00 units" and "Edge-weight cost: 80.00" describe the same route.** The
  difference is the library's 0.1 distance scale ([Units](TECHNICAL_NOTES.md#units)).

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
tools/evidence/            scenario files, exporter, benchmark and figure scripts for the README
docs/                      these notes; data/ and assets/ hold the README's measured data and figures
```
