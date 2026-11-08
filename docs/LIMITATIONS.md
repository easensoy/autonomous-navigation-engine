# Limitations

What the library does not do, what is known to be wrong, and what is unfinished. The README keeps a
short version; this is the full list.

## Scope

- **Graph planning only.** There is no time, kinematics, trajectory or vehicle model. A navigation step
  moves to the next node of the route. Obstacles are node ids that the caller reports; there is no
  perception.
- **Costs are given, not learned or measured.** The README's picking-zone multiplier (cost ×3) is an
  assumption of the scenario, not measured traffic.

## Known bugs and sharp edges

- **Replanning copies the whole graph.** `DynamicReplanner` copies the graph, then deletes each
  blocked node with a full adjacency scan, then builds a new A\* that rescans every edge. The search is
  14–16% of a replan's time ([measured](TECHNICAL_NOTES.md#where-a-replans-time-goes)).
- **The navigator's failure state is easy to misread.** When no route is left it reports `IDLE`, the
  same status as before a start, while `getCurrentPath()` still returns the old route through the
  blocked nodes ([details](TECHNICAL_NOTES.md#what-the-navigator-reports-when-no-route-is-left)).
- **Mixed distance units.** `Node::euclideanDistance` and `manhattanDistance` multiply coordinate
  distances by 0.1, so straight-line distances and edge weights can be in different units
  ([Units](TECHNICAL_NOTES.md#units)).
- **Jump Point Search is not always shortest.** Every route it returns is valid. In the
  November 2026 audit, on 300 random 8-connected grids with obstacles, its route was longer than
  Dijkstra's in 79 cases (by up to 2.4×) and it found no route in 4 cases where one existed. That
  audit was not repeated for this page. Its straight and diagonal jump rules come from different
  variants of the algorithm. Use A\* when the shortest route matters.
- **Depth-first search throws** `std::invalid_argument` for an unknown start or goal, unlike the other
  searches, which return an empty route.

## Unfinished code

- **Experimental code.** Many source files compile into the library but nothing in the public API, the
  demo or the tests reaches them. Examples: `DStarLite`, `DynamicWindowApproach` and
  `TravelingSalesmanSolver` in `src/navigation_strategies/`.
- **Declared but undefined methods.** These fail at link time if called:
  - the `GraphBuilder` and `LocalPathPlanner` constructors;
  - `GraphBuilder::buildGraph`;
  - `NodeManager::moveNode`;
  - `EdgeManager::getMinimumSpanningTree`;
  - `ConnectivityAnalyzer::getBridgeEdges`;
  - `MapUpdater::rollback`.

  The evidence tools build graphs through `Graph::addNode` and `addEdge` for this reason.
- **Placeholder scores.** The performance-based scoring in `AlgorithmSelector` uses simulated, random
  metrics, not measured ones.
- **Process-wide state.** 18 components keep a file-level singleton. One of them is the emergency-stop
  state, which the public API cannot reset.
- **Verbose output.** Most components print straight to standard output rather than through the
  logger, so their output cannot be filtered. BFS prints every node it visits.
- **Compiler warnings.** A GCC Debug build adds `-Wall -Wextra -Wpedantic` and reports 76 warnings:
  - 56 unused parameters;
  - 14 unused variables;
  - 3 out-of-order member initialisers;
  - one each of unused-but-set-variable, sign-compare and maybe-uninitialized.

  This was counted in November 2026, before this work.

## Measurements

- **One machine, one compiler.** The README's timings come from one laptop (Windows 11, MSVC 19.51,
  *Balanced* power plan). Other machines will give other absolute numbers.
  - The comparisons that matter are between planners on the same machine: paired per query, rotated
    in order.
  - [EVIDENCE.md](EVIDENCE.md) records the protocol, and the manifest records the machine.
- **BFS's timings include its print statements**, which cannot be switched off. Their formatting is
  skipped because `std::cout` has no buffer during benchmarks, but the calls remain.
- **No memory measurements.** Nothing on the README reports memory.
- **The test programs' benchmarks are smoke tests.** `algorithm_benchmarks` and `scability_tests` seed
  their random graphs from the clock and build a new solver for every query. Their timings are not
  comparable between runs; use `tools/evidence` for measurements.

## Not yet

- **macOS builds** have not been tried.
- **Linux:** only the CI workflow builds and tests on Linux (Ubuntu 24.04, GCC and Clang); its latest
  result is the README badge ([TESTING.md](TESTING.md)).
- Possible next steps, not commitments:
  - replan without copying the graph;
  - give the navigator a distinct "no route" state;
  - timed trajectories with a simple vehicle model, evaluated in closed loop;
  - a small learned scorer for candidate routes, compared against these planners as baselines.
