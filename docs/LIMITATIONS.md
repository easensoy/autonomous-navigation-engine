# Limitations

What the library does not do, what is known to be wrong, and what is unfinished. The README keeps a
short version; this is the full list. A limitation leaves this page only after its fix is implemented
and tested ([CHANGELOG.md](../CHANGELOG.md) lists those changes).

## Scope

- **Graph planning only.** There is no time, kinematics, trajectory or vehicle model. A navigation step
  moves a simulated position to the next node of the route. Obstacles are node ids that the caller
  reports; there is no perception. `NavigationState::BLOCKED` reports that no route is left; it is not a
  safety mechanism.
- **Costs are given, not learned or measured.** The README's picking-zone multiplier (cost ×3) is an
  assumption of the scenario, not measured traffic.
- **Jump Point Search has a narrow domain.** It accepts only uniform-cost grid graphs without corner
  cutting and rejects every other graph; weighted cells need A\* or Dijkstra
  ([domain](TECHNICAL_NOTES.md#jump-point-search-domain-rules-and-validation)). Its correctness on that
  domain rests on the canonical-ordering argument and a seeded differential test against Dijkstra, not
  on a proof.

## Known defects and sharp edges

- **Replanning copies the whole graph.** `DynamicReplanner` copies the graph, deletes each blocked node
  with a full adjacency scan, then builds a new A\* that rescans every edge. The search itself is a
  small share of a replan's time ([measured](TECHNICAL_NOTES.md#where-a-replans-time-goes)).
- **Mixed distance units.** `Node::euclideanDistance` and `manhattanDistance` multiply coordinate
  distances by 0.1, so straight-line distances and edge weights can be in different units
  ([Units](TECHNICAL_NOTES.md#units)). `GraphBuilder` and `NodeManager` use plain coordinate distances.
- **Depth-first search throws** `std::invalid_argument` for an unknown start or goal, unlike the other
  searches, which return an empty route.

## Unfinished code

- **Unreachable classes.** Many source files define classes that nothing constructs, so they compile to
  no code and are not part of the public API. Examples: `DStarLite`, `DynamicWindowApproach` and
  `TravelingSalesmanSolver` in `src/navigation_strategies/`, and `MinimumSpanningTreeCalculator` in
  `src/graph_operations/`.
- **Placeholder scores.** The performance-based scoring in `AlgorithmSelector` uses simulated, random
  metrics, not measured ones. When it selects Jump Point Search, execution falls back to A\*.
- **Process-wide state.** Several components keep a file-level singleton, among them the
  emergency-stop state, which the public API cannot reset, and the console and ASCII renderers, which
  stay bound to the first graph they draw.
- **Verbose output.** Most components print straight to standard output rather than through the
  logger, so their output cannot be filtered. BFS prints every node it visits.
- **Compiler warnings.** A GCC Debug build with `-Wall -Wextra -Wpedantic` reported 76 warnings in
  November 2026, before this work: 56 unused parameters, 14 unused variables, 3 out-of-order member
  initialisers, and one each of unused-but-set-variable, sign-compare and maybe-uninitialized.

## Measurements

- **One machine, one compiler, shared with other work.** The README's timings come from one laptop
  (Windows 11, MSVC 19.51, *Balanced* power plan). No controlled environment (an idle machine with
  fixed clock frequencies) was available, and other work was not stopped. The published run waited
  the full 15 minutes without reaching its 10% load gate; other processes then used 16–24% of the
  machine's CPU capacity while it ran, and a query's timed repeats spread by 10–48% of their median
  (median over queries). Dijkstra's median latency, whose code never changed, moved by up to 45%
  between the recorded runs. The comparisons that matter are within a run: paired per query, in
  rotated order ([protocol](EVIDENCE.md#measurement-protocol)).
- **BFS's timings include its print statements**, which cannot be switched off. Their formatting is
  skipped because `std::cout` has no buffer during benchmarks, but the calls remain.
- **No memory measurements.** Nothing on the README reports memory.
- **The test programs' benchmarks are smoke tests.** `algorithm_benchmarks` and `scability_tests` seed
  their random graphs from the clock and build a new solver for every query. Their timings are not
  comparable between runs; use `tools/evidence` for measurements.

## Not yet

- **macOS builds** have not been tried. Linux is built and tested only by the CI workflow (Ubuntu
  24.04, GCC and Clang); its latest result is the README badge ([TESTING.md](TESTING.md)).
- **Proposed extensions, not commitments:**
  - replan without copying the graph: skip blocked nodes inside the search and reuse the original
    A\* heuristic scale, which stays admissible when nodes are removed;
  - timed trajectories with a simple vehicle model, evaluated in closed loop;
  - a small learned scorer for candidate routes, evaluated against these planners as baselines.
