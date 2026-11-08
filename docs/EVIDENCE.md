# How the figures and numbers are produced

Every figure in the [README](../README.md) is drawn from files that the C++ library's own runs wrote.
This page lists what was run, how it was timed, what was checked, and which data each figure comes
from. Reproduction commands are at the [end](#reproduce).

## Pipeline

```text
tools/evidence/scenarios/*.txt ─┐
                                ├─ evidence_export (C++, trace mode) ──> docs/data/{hero,search,failure_*}.json
docs/data/workloads/*.txt ──────┤
                                └─ evidence_bench  (C++, measurement mode) ──> docs/data/bench/*.csv, *.json
                                          │
            summarize.py  ── docs/data/summary.json ──> render/*.py ──> docs/assets/*.png, *.gif
            provenance.py ── docs/data/manifest.json
            check_outputs.py ── re-checks routes, workload fingerprints and the README's numbers
```

`python tools/evidence/regenerate.py` runs these stages in that order (`tools/evidence/README.md`).

- **The library does all the planning.** Both C++ tools link the `NavigationEngine` library and build
  every graph through `Graph::addNode` and `Graph::addEdge`. Routes come from `NavigationCore`,
  `DynamicReplanner`, `AStar`, `Dijkstra` and `BFS`. The Python scripts compute percentiles and draw;
  they never search a graph.
- **Two modes, two programs.** `evidence_export` attaches search observers and times nothing.
  `evidence_bench` times and attaches no observer. The source of `evidence_bench` never calls
  `setSearchObserver`.

## Trace mode: ordered search events

`AStar`, `Dijkstra` and `BFS` accept an optional `SearchObserver`
([include/pathfinding_algorithms/SearchObserver.hpp](../include/pathfinding_algorithms/SearchObserver.hpp)).
It is called where each algorithm does the work:

| Event | A* | Dijkstra | BFS |
|---|---|---|---|
| `onExpand(node, cost, priority)` | a node is settled (taken off the open list and not stale); includes the goal, then the search stops | a node is settled; the search does not stop at the goal | a node is dequeued; the search stops when the goal is first *seen*, so the goal is never dequeued |
| `onFrontierPush(node, parent, cost, priority)` | a node gets a new best g; priority = g + h | a node gets a new best distance; priority = g | a node is discovered; priority = depth |

With no observer attached (the default), each hook is one null-pointer check. The checks that
tracing changes nothing:

- `evidence_export search` runs every planner twice, with and without an observer, and requires the
  same route. It also requires A*'s observer count to equal `AStar::getLastSearchNodesExplored()`,
  and expansions to arrive in nondecreasing priority.
- `tests/algorithm_tests/test_search_observer.cpp` (9 cases, part of CTest) checks the same on 40
  random queries on a weighted grid. It also checks that every push names an already-expanded
  parent, that Dijkstra settles each reachable node once at its final distance, and that BFS
  dequeues in nondecreasing depth.

"Expansions" in the figures always means `onExpand` events. The definitions differ by at most one
node between planners (the goal), as the table says.

## Measurement mode: what is timed

| Quantity | Timed region | Excluded |
|---|---|---|
| Query latency | one call to `AStar::findPath`, `Dijkstra::findShortestPath` or `BFS::findPath` | graph construction, workload loading, route validation, file output |
| A* first query | the first `findPath` on a fresh `AStar`; it includes the one-pass scan of all edges that sets the heuristic scale (cached per graph revision afterwards). Reported separately as `astar_first_query_ns` in `*.query_run.json` | — |
| Replan, end to end | one `NavigationCore::handleObstacles` call on a navigator that has just planned the trip | navigator construction and the initial plan |
| Replan parts | `Graph` copy constructor; `Graph::removeNode` for each blocked node; `AStar` constructor + first `findPath` on the copy; a second `findPath` on the same object. These are the operations `DynamicReplanner::planAvoidingObstacles` performs, timed one at a time. | as above |
| Hero plan / replan | `NavigationCore::startNavigation` on a new navigator; `handleObstacles` after 9 steps | the 9 steps |

- **Clock.** `std::chrono::steady_clock`, read immediately before and after the timed call. Its step
  on the test machine is 100 ns (`clock_step_ns` in every run record).
- **Repeats.** Each query runs once as warm-up, then 5 timed times. A query's latency is the median of
  its 5. Percentiles are taken across queries, by nearest rank. p99 is reported only when there are at
  least 1,000 queries, so it is not reported for the large warehouse (500 queries). Each replan event
  runs once as warm-up, then 5 times. The hero plan and replan are timed 1,000 times after 5 warm-ups.
- **Pairing and order.** For each query, the four planners run back to back, in an order rotated by
  query and repetition, so drift and cache effects spread evenly. Ratios such as "Dijkstra ÷ A*" are
  per query (paired), then summarised.
- **Solver reuse.** One solver object per planner is reused for all queries on a graph, as an
  application would. A*'s scale pass is paid before timing starts and reported on its own.
- **Logging.** The library prints to `std::cout` in many places. Every benchmark runs with `std::cout`
  holding no stream buffer, so each print returns after a stream-state check. `BFS::findPath` has
  per-node print statements that the API cannot switch off, so they stay in its timed loop. Their
  formatting is skipped but the calls are not, and **BFS's latency includes that residue**.
- **Machine.** One run on one laptop, recorded in `docs/data/manifest.json`. Intel Core Ultra 7
  165H (Dell Precision 5690), Windows 11 Pro 10.0.26200, AC power, *Balanced* power plan (left as
  the user had it). The benchmark thread is pinned to one logical processor of the fastest core class
  and runs at raised priority (`pinning` in each run record). Timings from other machines, compilers
  or power settings will differ. The comparisons between planners on the same machine are the point.
  Background processes were not stopped. During the published run, file sync, antivirus scanning and
  other processes kept several cores busy.
- **Build.** CMake Release with MSVC 19.51.36260: `/O2 /Ob2 /DNDEBUG /EHsc /GR /MD -std:c++17`.
  The exact compile commands are in the manifest.
- **Noise.** Per query, the spread of the 5 timed runs is (max − min) ÷ median. Its median over
  queries is 21–40% for most planner/size pairs in the published run, and 44–48% for A\* on the small
  and medium warehouses, whose sub-millisecond queries are the most exposed to scheduling noise
  (`repeat_spread_median` in `summary.json`). A quieter run of the same protocol earlier that day
  had 6–17% for most pairs. Taking each query's median of 5 damps this, and the README's comparisons
  are paired per query.
- **What moved between runs.** Three full runs were made that day; the published one is the last,
  at the committed source.
  - Absolute times moved with machine load. Dijkstra's and BFS's per-query times, whose code never
    changed, were 16–43% higher in the published run than in the first (median of paired ratios).
    The large warehouse's median replan went from 30.4 ms to 37.5 ms to 44.9 ms.
  - The search share of a replan stayed within 12–18%.
  - Dijkstra ÷ A\* went from 9.4–10.6× (first run, before the A\* fix) to 10.0–13.0×. Part of that
    is the fix, which made A\* 11–17% faster
    ([interleaved timing](TECHNICAL_NOTES.md#a-tie-bug-found-by-the-traces)), and part is load.

Sample sizes: 1,000 / 1,000 / 500 queries for the small, medium and large warehouses; 300 / 300 / 200
replan events. Failures are reported with denominators: every run record counts invalid routes, and
`summary.json` lists found, valid and optimal routes per planner.

## Validation runs (not part of the regeneration)

**What the hooks cost with no observer attached.** Measured before the A\* open-list fix, so the two
builds differ only by the hooks. The library as committed at HEAD (`57c9805`, without the hooks) was
built with the same, unchanged `evidence_bench` source and the same flags. It was timed against the
working tree on the same workloads, alternating the two binaries. Per-query latency of the working
tree ÷ HEAD (paired, `tools/evidence/compare_bench.py`):

| Workload | Runs | A\* | A\*, h = 0 | Dijkstra | BFS |
|---|---|---:|---:|---:|---:|
| small, 1,000 queries | 2 alternating pairs | 0.988 | 0.990 | 0.981 | 0.990 |
| medium, 1,000 queries | 1 pair | 0.963 | 0.968 | 0.970 | 0.986 |

Medians of the per-query ratios; the 5th–95th percentile spans are 0.73–1.21. The hooks add no
measurable cost, and the small differences in favour of the working tree are within run-to-run
noise. The HEAD build also returns non-optimal A\* routes on the same 15 small-warehouse queries, so
the [tie bug](TECHNICAL_NOTES.md#a-tie-bug-found-by-the-traces) predates this work.

**Same results with another compiler and standard library.** After the A\* fix, everything was
rebuilt with GCC 14.2.0 (libstdc++) and all four exports were rerun. Every exporter check passed, and
the output matches the MSVC 19.51 run in every respect apart from the compiler's name:

- the graph fingerprint;
- every route, node for node;
- every expansion count, and the expansion *order*, event for event, for all four planners;
- both failure states, string for string.

Before the fix the two builds matched in the same way, including the node A\* dropped in the hero
query and the 8 it dropped in the failure case. The two A\* regression tests failed with GCC on the
pre-fix code, as with MSVC, and pass with the fix.

**The replan finding with GCC.** `evidence_bench replans` and `hero` were rerun with the GCC build
after the fix (same protocol, not published as the main data):

| Warehouse | Replan p50 | Search alone p50 | Search share of a replan |
|---|---:|---:|---:|
| README map, 744 nodes | 0.45 ms | 0.07 ms | 16% |
| small, 1,107 nodes | 0.48 ms | 0.08 ms | 17% |
| medium, 8,943 nodes | 3.74 ms | 0.61 ms | 17% |
| large, 33,786 nodes | 23.5 ms | 3.18 ms | 15% |

- The finding holds: the search is 15–17% of a replan, against 14–16% with MSVC.
- Every replan returns the optimal route cost: 300, 300 and 200 events.
- The absolute times cannot be compared with MSVC's. The runs were made at different times, on a
  machine whose background load varied.

**The A\* fix's effect on latency.** Timed in alternation with the pre-fix build, A\*'s median query
time fell by 11–17%, while the unchanged planners moved by at most 6%
([details](TECHNICAL_NOTES.md#a-tie-bug-found-by-the-traces)). This is separate from the route-cost
result: the fix's purpose was optimal routes, and the speed change is a side effect.

## Workloads

`docs/data/workloads/` holds the saved benchmark inputs. They were written by `evidence_export
workloads` from fixed seeds with splitmix64, which gives the same numbers on every compiler (unlike
`std::uniform_int_distribution`).

| Name | Racks | Lattice | Nodes | Edges | Picking zones |
|---|---|---|---|---|---|
| `warehouse_s` | 4 × 5 | 63 × 29 | 1,107 | 3,214 | 4 |
| `warehouse_m` | 12 × 17 | 183 × 89 | 8,943 | 25,370 | 50 |
| `warehouse_l` | 24 × 34 | 363 × 174 | 33,786 | 95,183 | 189 |

- Each aisle stretch between two cross aisles is a picking zone (cost ×3) with probability 0.25.
- Queries are uniform pairs of distinct nodes.
- A replan event is a trip of at least a quarter of the warehouse diagonal. A 3 × 3 square around the
  middle node of A*'s route is blocked, and the navigator replans from the start of the trip.
- Every run record stores a fingerprint (FNV-1a over node coordinates and edge weights).
  `check_outputs.py` requires it to match `workloads.json`, so a timing cannot come from a different
  graph than the one documented.

## Checks

- **`evidence_export`** exits non-zero if any check fails. Its checks:
  - every route starts and ends where it should and follows graph edges;
  - no blocked node is entered after the blockage;
  - the replanned remainder starts at the navigator's node and keeps the travelled part unchanged;
  - `getRemainingDistance()` equals the summed edge weights ahead at every step;
  - A* and Dijkstra costs equal Dijkstra's optimum on the same reduced graph;
  - A\* and Dijkstra expand every node they pushed with priority below the optimum;
  - BFS's edge count equals the fewest possible (Dijkstra on unit weights);
  - failure cases agree with an independent reachability test, and when the goal is cut off, A\*
    expands the whole reachable component before giving up.

  The results are stored in each JSON file under `checks`.
- **`evidence_bench`** validates every route it times. It also counts A\* routes that are valid but
  not optimal (`astar_not_optimal`, `replans_not_optimal`) and leaves the deciding to
  `check_outputs.py`.
- **`check_outputs.py`** re-checks the exported routes with plain arithmetic over the exported edge
  list. It also checks:
  - the workload fingerprints and sample counts;
  - from the raw events, that no node pushed below the optimum was left unexpanded;
  - that A\* returns Dijkstra's cost on every benchmark query and replan: by the benchmark's counts,
    and recomputed from the raw results files;
  - that each A\* miss recorded before the
    [open-list fix](TECHNICAL_NOTES.md#a-tie-bug-found-by-the-traces) (`astar_fix_before.json`) is
    the same query as now, answered at Dijkstra's cost;
  - that every number the README quotes equals the value in `summary.json`, and that every README
    link target exists.

  Run against the data from before the fix, the new checks fail exactly where the bug showed: the
  dropped node in the hero query, the failure case, and all 50 queries and 23 replans.

## Artifact map

| Asset (light / dark) | Drawn by | From | Produced by |
|---|---|---|---|
| `hero-{light,dark}.gif`, `hero-static-{light,dark}.png` | `render/hero.py` | `hero.json` (routes, per-step timeline), `search.json` (shortest-by-distance reference), `summary.json` (plan and replan p50) | `evidence_export hero` and `search` on `scenarios/hero_replan.txt`; `evidence_bench hero` |
| `replan-cost-*.png` | `render/perf.py` | `summary.json`, from `bench/*.replan_times.csv` and `bench/hero.times.csv` | `evidence_bench replans` and `hero` |
| `search-comparison-*.png` | `render/search.py` | `search.json` (`expand_events` in recorded order, returned routes) | `evidence_export search` |
| `perf-latency-*.png` | `render/perf.py` | `summary.json` and `bench/*.query_results.csv`, from `bench/*.query_times.csv` | `evidence_bench queries` on `workloads/` |
| `failure-panel-*.png` | `render/failure.py` | `failure_goal_blocked.json`, `failure_disconnected.json` | `evidence_export failure` |
| `how-it-fits-*.png` | `render/diagram.py` | `hero.json` (node, blocked-node and step counts); the call path is `NavigationCore` → `DynamicReplanner` as written in `src/` | — |

Data files in `docs/data/`:

| File | Contents |
|---|---|
| `hero.json` | Scenario, full graph (nodes, edges with weights), routes before and after the replan, per-step timeline (step, node, route version, remaining cost), status strings, checks |
| `search.json` | The replanner's query; for each planner: route, cost, length, edges, ordered expansion and push events; the shortest-by-distance reference route; checks |
| `failure_*.json` | Navigator state before and after the blockage, the console message, the replanner's result, reachability, A*'s expansion count, checks |
| `workloads/` | Benchmark scenarios, queries, replan events and `workloads.json` (sizes, seeds, fingerprints) |
| `bench/*.query_times.csv` | One row per timed call: query, planner, repetition, nanoseconds |
| `bench/*.query_results.csv` | One row per query and planner: found, valid, cost, edges, length, A* expansions, cost ÷ Dijkstra's |
| `bench/*.replan_times.csv`, `*.replan_results.csv` | Replan phases per event and repetition; outcome per event |
| `bench/hero.times.csv` | The hero's plan, replan and replan parts, 1,000 runs |
| `bench/*_run.json`, `hero.run.json` | Run records: compiler, clock step, pinning, repeats, graph fingerprint, sample and invalid counts |
| `astar_fix_before.json` | Frozen reference, not regenerated: every A\* query and replan that missed the optimum before the open-list fix, with its start, goal and costs, and the source state of that run |
| `summary.json` | Every number the README quotes, computed from the files above |
| `manifest.json` | Git revision, dirty-state id and modified files, CMake cache and compile commands, machine, Python package versions, SHA-256 of every data and asset file |

## Units

- **Coordinates and lengths** are raw coordinate units, the x/y stored on each `Node`, written `u`.
  The library's `Node::euclideanDistance` multiplies distances by 0.1 (a leftover
  `COORDINATE_TO_METER_SCALE`). The evidence tools never use it for anything they report, and nothing
  here is in metres.
- **Costs** are edge weights: length × zone multiplier.
- **Steps** are calls to `NavigationCore::executeNavigationStep`, one graph node each. The animation
  plays steps at a fixed rate that has nothing to do with computation time. Measured times are
  printed separately in the figure.

## Reproduce

From a clean checkout, in a shell with CMake, Ninja and a C++17 compiler on `PATH`:

```bash
python -m pip install -r tools/evidence/requirements.txt
python tools/evidence/regenerate.py --cxx cl       # MSVC, from the x64 Native Tools prompt
python tools/evidence/regenerate.py --cxx g++      # or GCC / Clang
```

On the test machine a full run takes 9–12 minutes:
- about 20 s to build from scratch;
- 7.5–10 minutes of benchmark, depending on background load;
- 50–90 s to render.

Useful options:

- `--skip-bench` reuses the saved timings and re-renders in one to two minutes.
- `--quick` takes one timed repeat with no warm-up. It is for checking the pipeline, not for
  publishing.
- `--regenerate-workloads` rewrites the workload files from their seeds. The fingerprints in
  `workloads.json` show whether anything changed.

Equal-cost ties could resolve differently between standard libraries, because `std::priority_queue`
leaves the order of equal elements unspecified. A route might then differ on another compiler while
its cost stays the same. The exporter's checks therefore compare costs, not node lists. (MSVC and GCC
happen to agree node for node on these scenarios; see the validation runs above.)
