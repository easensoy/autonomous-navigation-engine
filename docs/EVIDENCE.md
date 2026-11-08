# How the figures and numbers are produced

Every figure in the [README](../README.md) is drawn from files that the C++ library's own runs wrote.
This page lists what was run, how it was timed, what was checked, and which data each figure comes
from. Reproduction commands are at the [end](#reproduce).

## Pipeline

```text
tools/evidence/scenarios/*.txt ─┐
                                ├─ evidence_export (C++, trace mode) ──> docs/data/{hero,search,failure_*}.json
docs/data/workloads/*.txt ──────┤
                                └─ load gate, then evidence_bench (C++, measurement mode) ──> docs/data/bench/
                                          │
            summarize.py  ── docs/data/summary.json ──> render/*.py ──> docs/assets/*.png, *.gif
            provenance.py ── docs/data/manifest.json
            history.py    ── docs/data/history/ (every benchmark run, kept)
            check_outputs.py ── re-checks routes, raw results, the history and the README's numbers
```

`python tools/evidence/regenerate.py` runs these stages in that order
([tools/evidence/README.md](../tools/evidence/README.md)).

- **The library does all the planning.** Both C++ tools link the `NavigationEngine` library and build
  every graph through `Graph::addNode` and `Graph::addEdge`. Routes come from `NavigationCore`,
  `DynamicReplanner`, `AStar`, `Dijkstra` and `BFS`. The Python scripts compute percentiles, draw and
  check; they never search a graph.
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

## Measurement protocol

### What is timed

| Quantity | Timed region | Excluded |
|---|---|---|
| Query latency | one call to `AStar::findPath`, `Dijkstra::findShortestPath` or `BFS::findPath` | graph construction, workload loading, route validation, file output |
| A* first query | the first `findPath` on a fresh `AStar`; it includes the one-pass scan of all edges that sets the heuristic scale (cached per graph revision afterwards). Reported separately as `astar_first_query_ns` in `*.query_run.json` | — |
| Replan, end to end | one `NavigationCore::handleObstacles` call on a navigator that has just planned the trip | navigator construction and the initial plan |
| Replan parts | `Graph` copy constructor; `Graph::removeNode` for each blocked node; `AStar` constructor + first `findPath` on the copy; a second `findPath` on the same object. These are the operations `DynamicReplanner::planAvoidingObstacles` performs, timed one at a time. | as above |
| Hero plan / replan | `NavigationCore::startNavigation` on a new navigator; `handleObstacles` after 9 steps | the 9 steps |

### Sample structure

| Workload | Queries | Timed query calls (4 planners × 5 repeats) | Replan events | Timed replans (× 5 repeats) |
|---|---:|---:|---:|---:|
| `warehouse_s`, 1,107 nodes | 1,000 | 20,000 | 300 | 1,500 |
| `warehouse_m`, 8,943 nodes | 1,000 | 20,000 | 300 | 1,500 |
| `warehouse_l`, 33,786 nodes | 500 | 10,000 | 200 | 1,000 |
| hero map, 744 nodes | — | — | 1 | 1,000 |

- **Repeats.** Each query runs once per planner as a warm-up, then 5 timed times. A query's latency is
  the median of its 5. Each replan event runs once as a warm-up, then 5 times; each repetition times
  the end-to-end replan and then each of its parts. The hero's plan and replan are timed 1,000 times
  after 5 warm-ups.
- **Statistics.** Percentiles are taken across queries (or replan events), by nearest rank, from the
  per-query medians. p99 is reported only when there are at least 1,000 values, so not for the large
  warehouse's queries.
- **Fixed workloads.** The queries and replan events are saved files generated from fixed seeds
  ([Workloads](#workloads)); every run times the same inputs, and `check_outputs.py` requires each run
  record's graph fingerprint to match the saved workload.

### Controls

- **Clock.** `std::chrono::steady_clock`, read immediately before and after the timed call. Its step
  on the test machine is 100 ns (`clock_step_ns` in every run record).
- **Pairing and order.** For each query, the four planners run back to back, in an order rotated by
  query and repetition, so drift and cache effects spread evenly over the planners. Ratios such as
  "Dijkstra ÷ A\*" are computed per query from the per-query medians (paired), then summarised.
- **Setup outside the timed region.** Graph construction, workload loading, route validation and file
  output happen outside it. One solver object per planner is reused for all queries on a graph, as an
  application would; A\*'s scale pass is paid before timing starts and reported on its own. The
  replan parts are timed separately from the end-to-end replan.
- **Logging.** The library prints to `std::cout` in many places. Every benchmark runs with `std::cout`
  holding no stream buffer, so each print returns after a stream-state check. `BFS::findPath` has
  per-node print statements that the API cannot switch off, so they stay in its timed loop. Their
  formatting is skipped but the calls are not, and **BFS's latency includes that residue**.
- **Pinning.** The benchmark thread is pinned to one logical processor of the fastest core class and
  runs at raised priority (`pinning` in each run record).
- **Build.** CMake Release with MSVC 19.51.36260: `/O2 /Ob2 /DNDEBUG /EHsc /GR /MD -std:c++17`. The
  exact compile commands are in the manifest.

### Machine and background load

One laptop, recorded in `docs/data/manifest.json`: Intel Core Ultra 7 165H (Dell Precision 5690),
22 logical processors, Windows 11 Pro 10.0.26200, AC power, *Balanced* power plan (left as the owner
had it). Timings from other machines, compilers or power settings will differ; the comparisons between
planners on the same machine are the point.

The machine is not dedicated to benchmarking, and nothing running on it was stopped. Two measures make
the load visible instead:

- **Load gate.** Before timing, `regenerate.py` samples the machine's CPU load (the CPU time of all
  processes as a share of all logical processors) in 10-second windows. It starts the benchmark once
  two windows in a row are at or below a threshold (10% by default), or when a time limit (900 s by
  default) runs out. The benchmark runs either way; `docs/data/bench/environment.json` keeps every
  window and whether the gate was met.
- **Background load per command.** Each run record stores `background_cpu_load`: the CPU time of all
  other processes while that command ran, divided by its wall time × the number of logical processors.
  It is recorded, not subtracted from any timing.

**In the published run** the gate was not met: in 900 s, no two consecutive windows were at or below
10%; the 90 windows measured 20–51%. The benchmark then ran from 19:39 to 19:48 UTC on 8 November 2026.
During its commands other processes used 16–24% of the machine's capacity: 18%, 24% and 24% during the
small, medium and large query runs; 16%, 23% and 21% during the replan runs; 18% during the hero
timing. A sample of the busiest processes taken during the run showed Docker Desktop with its WSL 2
virtual machine, OneDrive, local development servers and other analysis jobs, none of them part of this
benchmark.

### Variability

Two kinds of variation are reported separately, per planner and warehouse (`summary.json`):

- **Measurement variation:** how much repeated timings of the same query differ. Per query, the spread
  of its 5 timed repeats is (max − min) ÷ median; `repeat_spread_median` and `repeat_spread_p95` are
  its median and 95th percentile over queries. For replans, `end_to_end_repeat_spread_median` is the
  same median for the end-to-end time.
- **Workload variation:** how much queries differ from each other. `workload_p95_over_p50` is the 95th
  percentile of the per-query medians divided by their median.

In the published run:

| Warehouse | Planner | p50 latency | p95 latency | Repeat spread, median | Repeat spread, p95 | p95 ÷ p50 across queries |
|---|---|---:|---:|---:|---:|---:|
| small | A\* | 0.045 ms | 0.187 ms | 15% | 112% | 4.13 |
| small | A\*, *h* = 0 | 0.191 ms | 0.360 ms | 12% | 122% | 1.88 |
| small | Dijkstra | 0.485 ms | 0.530 ms | 13% | 99% | 1.09 |
| small | BFS | 0.358 ms | 0.693 ms | 10% | 127% | 1.94 |
| medium | A\* | 0.612 ms | 2.810 ms | 48% | 115% | 4.59 |
| medium | A\*, *h* = 0 | 2.191 ms | 5.241 ms | 35% | 90% | 2.39 |
| medium | Dijkstra | 5.795 ms | 8.092 ms | 22% | 64% | 1.40 |
| medium | BFS | 3.267 ms | 7.239 ms | 26% | 87% | 2.22 |
| large | A\* | 3.646 ms | 21.786 ms | 34% | 103% | 5.98 |
| large | A\*, *h* = 0 | 17.723 ms | 45.770 ms | 31% | 71% | 2.58 |
| large | Dijkstra | 47.592 ms | 57.643 ms | 23% | 42% | 1.21 |
| large | BFS | 19.902 ms | 45.066 ms | 36% | 74% | 2.26 |

- **Measurement variation is large** under this load: the median query's repeats spread by 10–48% of
  their median, and one query in twenty by 42–127%, a single repeat slowed by preemption or frequency
  changes. Taking each query's median of 5 removes single outliers; it does not remove a slowdown that
  lasts through all 5 repeats. Replans spread by 17%, 22% and 33% (median, end to end).
- **Workload variation is a property of the planner**, not noise: Dijkstra settles every reachable node
  whatever the query, so its p95 is within 1.1–1.4× its median, while A\*'s work grows with the
  distance to the goal (4.1–6.0×).
- **Paired ratios are less exposed** than absolute times, because a slowdown that lasts through one
  query's back-to-back calls affects all four planners. They are not immune: a slowdown that starts or
  stops between two planners' calls does not cancel.

### Run history and selection rule

Every full run of the benchmark is kept in [`docs/data/history`](data/history): `index.json` lists each
run with its source state, compiler, machine, load-gate result, background load and headline numbers,
and each run's folder keeps its `summary.json`, `manifest.json` and run records. A later run never
replaces an earlier one.

**Selection rule**, stated in `index.json`: the README, `docs/data/summary.json` and the figures use the
newest complete benchmark run made at the published source revision; every run stays in the history,
whatever its results. `check_outputs.py` requires the published run records to be the newest complete
run in the history. The rule was declared before the fourth run was made.

| Run (UTC) | Source | Load gate | Background load | Large replan p50 | Hero replan p50 | Search share of a replan | Dijkstra ÷ A\*, paired p50 | A\* at Dijkstra's cost |
|---|---|---|---:|---:|---:|---:|---:|---|
| 2026-11-08 04:30 | `57c9805` + uncommitted | not used | not measured | 30.4 ms | 0.44 ms | 12–18% | 9.4–10.6× | 985/1,000, 976/1,000, 489/500 |
| 2026-11-08 11:13 (partial) | `57c9805` + uncommitted | not used | not measured | 37.5 ms | 0.50 ms | 13–17% | 10.4–12.4× | 1,000/1,000, 1,000/1,000, 500/500 |
| 2026-11-08 11:46 | `d732311` | not used | not measured | 44.9 ms | 0.42 ms | 14–16% | 10.0–13.0× | 1,000/1,000, 1,000/1,000, 500/500 |
| 2026-11-08 19:49 (published) | `361f788` | not met in 900 s | 16–24% | 36.7 ms | 0.35 ms | 13–16% | 9.8–12.1× | 1,000/1,000, 1,000/1,000, 500/500 |

- **Run 1 predates the A\* open-list fix**, which explains its A\* misses. Run 2 is a partial record: its
  files were overwritten by run 3 before the history existed, and only the headline values printed at
  the time remain. Runs 1–3 predate the load gate and the background-load records.
- **Code that changed between runs.** Run 1 → 2: the A\* open-list fix. Run 3 → 4: `NavigationCore`
  (explicit states), which is inside the end-to-end replan and the hero timings. The query planners,
  `Graph` and the replan parts are the same code in runs 2, 3 and 4.
- **What moved.** Dijkstra's median query time, whose code never changed, was 0.47, 0.68 and 0.49 ms on
  the small warehouse and 34.5, 48.3 and 47.6 ms on the large one in runs 1, 3 and 4: absolute times
  moved by up to 45% between runs on the same machine. The search share of a replan stayed within
  12–18%, and Dijkstra ÷ A\* within 9.4–13.0×.
- **Figures redrawn after the run.** After the run at `361f788`, six Python evidence scripts changed:
  two drawing fixes (the failure panel's console text, `66fd15a`; the spacing of the latency figure's
  labels, `4d7687a`), the mutation test's copy of the evidence (`3accd2f`, `0a8f807`), and hashes that
  do not depend on line endings for the history check and the manifest (`ff7a16a`, `27da527`). The
  figures and the manifest were then regenerated with `--skip-bench` from the same run records, so
  `docs/data/manifest.json` names `27da527`, while the history keeps the run's own manifest. The
  library, the tests and the benchmark programs are the same at both commits.

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

**Same traces with another compiler and standard library.** After the A\* fix (at `d732311`),
everything was rebuilt with GCC 14.2.0 (libstdc++) and all four exports were rerun. Every exporter
check passed, and the output matched the MSVC 19.51 run in every respect apart from the compiler's
name:

- the graph fingerprint;
- every route, node for node;
- every expansion count, and the expansion *order*, event for event, for all four planners;
- both failure states, string for string.

Before the fix the two builds matched in the same way, including the node A\* dropped in the hero
query and the 8 it dropped in the failure case. The two A\* regression tests failed with GCC on the
pre-fix code, as with MSVC, and pass with the fix. `compare_traces.py` now repeats this comparison
for the current code ([Checks](#checks)).

**The replan finding with GCC.** `evidence_bench replans` and `hero` were rerun with the GCC build
after the fix (at `d732311`, same protocol, not published as the main data):

| Warehouse | Replan p50 | Search alone p50 | Search share of a replan |
|---|---:|---:|---:|
| README map, 744 nodes | 0.45 ms | 0.07 ms | 16% |
| small, 1,107 nodes | 0.48 ms | 0.08 ms | 17% |
| medium, 8,943 nodes | 3.74 ms | 0.61 ms | 17% |
| large, 33,786 nodes | 23.5 ms | 3.18 ms | 15% |

- The finding holds: the search is 15–17% of a replan, against 13–16% with MSVC in the published run.
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
    expands the whole reachable component before giving up;
  - after a failed replan the navigator is `BLOCKED`, stays on its node, refuses to step, returns only
    the travelled part from `getCurrentPath()` and the abandoned route from `getInvalidatedRoute()`,
    reports an infinite remaining distance, and fails `retryNavigation()` while the blockage stands.

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
  - the failure states listed above, from the exported values;
  - the run history: a stated selection rule, runs in time order, a summary and manifest kept for
    every complete run, and the published run records as the newest complete run;
  - that every number the README quotes from `summary.json` and the history is stated as computed, and
    that every README link target exists.

  Run against the data from before the A\* fix, these checks fail exactly where the bug showed: the
  dropped node in the hero query, the failure case, and all 50 queries and 23 replans.
- **In CI**, the *Evidence checks* job runs `check_outputs.py`; then `test_check_outputs.py`, which
  corrupts a copy of the evidence six ways and requires the checker to fail on each; then it builds
  `evidence_export` on Ubuntu with GCC, and `compare_traces.py` requires the four committed traces
  back exactly, apart from the fields naming the compiler and the scenario path. CI never runs the
  benchmark.

## Artifact map

| Asset (light / dark) | Drawn by | From | Produced by |
|---|---|---|---|
| `hero-{light,dark}.gif`, `hero-static-{light,dark}.png` | `render/hero.py` | `hero.json` (routes, per-step timeline), `search.json` (shortest-by-distance reference), `summary.json` (plan and replan p50) | `evidence_export hero` and `search` on `scenarios/hero_replan.txt`; `evidence_bench hero` |
| `replan-cost-*.png` | `render/perf.py` | `summary.json`, from `bench/*.replan_times.csv` and `bench/hero.times.csv` | `evidence_bench replans` and `hero` |
| `search-comparison-*.png` | `render/search.py` | `search.json` (`expand_events` in recorded order, returned routes) | `evidence_export search` |
| `perf-latency-*.png` | `render/perf.py` | `summary.json` and `bench/*.query_results.csv`, from `bench/*.query_times.csv` | `evidence_bench queries` on `workloads/` |
| `failure-panel-*.png` | `render/failure.py` | `failure_goal_blocked.json`, `failure_disconnected.json` (state, travelled and invalidated routes, console message, status, retry result) | `evidence_export failure` |
| `how-it-fits-*.png` | `render/diagram.py` | `hero.json` (node, blocked-node and step counts); the call path is `NavigationCore` → `DynamicReplanner` as written in `src/` | — |

Data files in `docs/data/`:

| File | Contents |
|---|---|
| `hero.json` | Scenario, full graph (nodes, edges with weights), routes before and after the replan, per-step timeline (step, node, route version, remaining cost), status strings, checks |
| `search.json` | The replanner's query; for each planner: route, cost, length, edges, ordered expansion and push events; the shortest-by-distance reference route; checks |
| `failure_*.json` | Navigator state before and after the blockage, the console message, the travelled and invalidated routes, the refused step, the retry result, the replanner's result, reachability, A*'s expansion count, checks |
| `workloads/` | Benchmark scenarios, queries, replan events and `workloads.json` (sizes, seeds, fingerprints) |
| `bench/*.query_times.csv` | One row per timed call: query, planner, repetition, nanoseconds |
| `bench/*.query_results.csv` | One row per query and planner: found, valid, cost, edges, length, A* expansions, cost ÷ Dijkstra's |
| `bench/*.replan_times.csv`, `*.replan_results.csv` | Replan phases per event and repetition; outcome per event |
| `bench/hero.times.csv` | The hero's plan, replan and replan parts, 1,000 runs |
| `bench/*_run.json`, `hero.run.json` | Run records: compiler, clock step, pinning, repeats, graph fingerprint, sample and invalid counts, logical processors, background CPU load |
| `bench/environment.json` | The load gate's threshold, windows, wait and result, and the benchmark's start and end times |
| `history/` | `index.json` (every benchmark run, its environment and headline numbers, and the selection rule) and one folder per complete run with its summary, manifest and run records |
| `astar_fix_before.json` | Frozen reference, not regenerated: every A\* query and replan that missed the optimum before the open-list fix, with its start, goal and costs, and the source state of that run |
| `summary.json` | Every number the README quotes from this run, computed from the files above |
| `manifest.json` | Git revision, dirty-state id and modified files, CMake cache and compile commands, machine, Python package versions, SHA-256 of every data and asset file (text files with LF line endings, as git stores them) |

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

On the test machine the published run took about 11 minutes after the load gate:

- about 12 s to build the evidence tools;
- 9 minutes 12 s of benchmark;
- about 90 s to summarise, render, record and check.

Before the benchmark, the load gate waits up to 15 minutes for a quiet machine.

Useful options:

- `--skip-bench` reuses the saved timings and re-renders in one to two minutes.
- `--load-gate 0.05`, `--load-gate-timeout 1800` and `--no-load-gate` change or skip the wait.
- `--history-note "..."` stores a note with the run in `docs/data/history/index.json`.
- `--quick` takes one timed repeat with no warm-up. It is for checking the pipeline, not for
  publishing, and is not recorded in the history.
- `--regenerate-workloads` rewrites the workload files from their seeds. The fingerprints in
  `workloads.json` show whether anything changed.

The checks alone, with Python's standard library and no build:

```bash
python tools/evidence/check_outputs.py
python tools/evidence/test_check_outputs.py
```

Equal-cost ties could resolve differently between standard libraries, because `std::priority_queue`
leaves the order of equal elements unspecified. A route might then differ on another compiler while
its cost stays the same. The exporter's checks therefore compare costs, not node lists, and
`compare_traces.py`, which compares node lists and event order, is run with a single toolchain in CI.
MSVC and GCC happen to agree node for node on these scenarios (see the validation runs above).
