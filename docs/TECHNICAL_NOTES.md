# Technical notes

Design decisions, the measured findings behind the README, and what they imply. Numbers come from
`docs/data/summary.json` unless stated otherwise ([how they were measured](EVIDENCE.md)).

- [Why the default A\* heuristic is safe](#why-the-default-a-heuristic-is-safe)
- [A\* tie bug found by the traces](#a-tie-bug-found-by-the-traces)
- [Where a replan's time goes](#where-a-replans-time-goes)
- [What the navigator reports when no route is left](#what-the-navigator-reports-when-no-route-is-left)
- [Search observer](#search-observer)
- [Units](#units)
- [Failure contract of the searches](#failure-contract-of-the-searches)

## Why the default A\* heuristic is safe

Let *k* be the smallest cost-per-length ratio over all edges, and let *h*(*n*) = *k* · |*n* − goal|.
For any edge *u* → *v*, the triangle inequality gives
*h*(*u*) ≤ *k* · (|*uv*| + |*v* − goal|) ≤ cost(*u*, *v*) + *h*(*v*).
So *h* is consistent, and therefore admissible, whatever the non-negative edge costs are.

Every change to a `Graph` advances `Graph::getRevision()`: nodes, edges, or mutable node access.
`AStar` computes *k* in one pass over the edges, caches it against the revision, and recomputes it only
when the revision moves.

There is a price. *k* is a global minimum, so one cheap edge anywhere weakens the heuristic everywhere.
In the README's warehouse every edge costs at least its length, so *k* = 1 and *h* is the straight-line
distance.

## A\* tie bug found by the traces

Found by the ordered search traces and fixed in the open list. The numbers before the fix are frozen
in [`docs/data/astar_fix_before.json`](data/astar_fix_before.json), taken from the first published
run of this pipeline: `57c9805` with uncommitted state `999180972288abbe`.

**Symptom, before the fix.** With its default heuristic, `AStar::findPath` could leave a node on the
frontier forever. The replanner's failure case first showed it: when the goal was cut off, A\* stopped
after 720 expansions, though 728 nodes were reachable. All 728 had been pushed, and 8 were never
expanded. With *h* = 0 the same search expanded all 728.

**Mechanism.** The printout showed it directly:

- Two routes of equal cost to a node can sum to values one rounding step apart (20.656854249492383
  and 20.65685424949238).
- Once *h* (about 22) is added, both open-list entries have exactly the same *f*.
- On equal *f* the open list prefers the larger *g* (`AStar::AStarNode::operator>`), so it popped the
  stale entry first.
- `OpenListManager::pop` then removed the node from its open set, and `findPath` discarded the entry
  as stale.
- When the fresh entry reached the top, `pop` skipped it, because the node was no longer in the open
  set.

**Consequences, measured before the fix.**

- On the saved warehouses, A\* returned a costlier route than Dijkstra on 15 of 1,000 (small),
  24 of 1,000 (medium) and 11 of 500 (large) queries.
- Each miss cost 0.51–1.41 more than the optimum: usually 0.586 (2 − √2), and at most 7.8% of the
  route's cost. No query lost its route entirely.
- With *h* = 0, where no rounding step is absorbed, A\* matched Dijkstra on every query.
- Replans inherited the bug through `DynamicReplanner`'s A\*: 5 of 300, 8 of 300 and 10 of 200 replan
  events returned a costlier route than the optimum.
- In the README's hero replan query, A\* dropped node 307 and expanded 157 nodes. In the failure case
  it dropped eight.
- The library as committed at `57c9805`, without this work's hooks, gave the same 15 misses on the
  small warehouse, so the bug predates this work.

**Minimal counterexample.** On five nodes, A\* returned *no route* although one exists:

```text
S(0,0) -0.1-> P(1,0) -0.2--> X(1,1) -1000-> G(1000,1)
S(0,0) -0.15> Q(0,1) -0.15-> X(1,1)

A* (default heuristic) events, before the fix:
  push   0 from -1 g=0 f=100.00004999998751
  expand 0 g=0 f=100.00004999998751
  push   1 from 0 g=0.10000000000000001 f=100.00005005003752
  push   2 from 0 g=0.14999999999999999 f=100.15000000000001
  expand 1 g=0.10000000000000001 f=100.00005005003752
  push   3 from 1 g=0.30000000000000004 f=100.2
  expand 2 g=0.14999999999999999 f=100.15000000000001
  push   3 from 2 g=0.29999999999999999 f=100.2
A* route length: 0 (expanded 3)
Dijkstra route: 0 2 3 4  cost 1000.3
```

After the fix the same eight events come first. Then the stale entry for X is popped and skipped,
which emits no event, and the live one is expanded:

```text
  expand 3 g=0.29999999999999999 f=100.2
  push   4 from 3 g=1000.3 f=1000.3
  expand 4 g=1000.3 f=1000.3
A* route length: 4 (expanded 5)
```

**The fix.** The open list no longer decides which of its entries are live:

- `OpenListManager` is a plain priority queue. `pop()` returns the top entry, `empty()` is the
  queue's emptiness, and `size()` counts entries. The open set has gone, along with the two unused
  private helpers that read it (`isInOpenList`, `removeFromOpenList`).
- `findPath` already skipped an entry whose *g* is above the best known *g* for its node, or whose node
  was expanded at no greater cost. That check now decides on its own.
- The check is exact. A node is pushed only when its *g* strictly improves, so each of its entries has
  a different *g*, and the entry with the best one is never skipped. The tie-break on equal *f*
  (larger *g* first) is unchanged; it now affects only the order of work, not which nodes are expanded.

**Verified after the fix.**

- **Regression tests.** Two cases in `tests/algorithm_tests/test_a_star.cpp`: the five-node graph,
  and 1,339 queries on a 22 × 17 racked grid with unit and diagonal costs, checked against Dijkstra.
  Run on the unfixed code, both failed with MSVC and with GCC: no route, and 34 queries costlier. Both
  pass now.
- **Benchmark.** On the same workloads, A\* returns Dijkstra's cost on all 2,500 queries and all 800
  replans. `check_outputs.py` recomputes this from the raw results. It also looks up each of the 50
  queries and 23 replans listed in `astar_fix_before.json` and requires the same start and goal, the
  same optimum, and now an optimal route.
- **Traces.** In the hero's replan query, A\* now expands 158 nodes. The only addition is node 307,
  and the route is unchanged. In the failure case it expands all 728 reachable nodes before giving up.
  Dijkstra's, A\* (*h* = 0)'s and BFS's traces are identical to before, event for event.
- **Exporter.** It now fails if A\* or Dijkstra leaves unexpanded any node pushed with priority below
  the optimum. On the pre-fix data, the new checks fail exactly where the bug showed.
- **History.** Commit `9509e4b` is the tree just before the fix: the two new tests fail there. Its
  sources are identical to the pre-fix build used in the timing run below, which reproduces the
  recorded 15 and 24 misses on the small and medium warehouses.

**Latency, measured separately.** Route cost and speed are different questions, so the fix's effect on
speed was timed on its own. The pre-fix binary (the tree at `9509e4b`) and the fixed one were run
alternately on the small and medium workloads, two pairs each, with the same flags and protocol.
Paired per query, the median ratio of fixed ÷ pre-fix latency was:

| Workload | A\* | A\*, *h* = 0 | Dijkstra (unchanged) | BFS (unchanged) |
|---|---:|---:|---:|---:|
| small, 2 × 1,000 queries | 0.89 | 0.88 | 1.03 | 1.02 |
| medium, 2 × 1,000 queries | 0.83 | 0.83 | 0.94 | 0.95 |

A\*'s median query time fell by 11–17%. The unchanged planners moved by up to 6% between the same
runs, which bounds the noise. A drop of this size fits what the fix removed: a hash-set insert on
every push, and a lookup and erase on every pop. These raw runs are kept locally, not in `docs/data`.

## Where a replan's time goes

`DynamicReplanner::planAvoidingObstacles` runs whenever a reported obstacle lies on the route ahead.
It does four things:

1. Copies the whole graph: `Graph(*environment)`, which allocates every node again.
2. Removes each blocked node with `Graph::removeNode`, which scans every adjacency list once per
   removed node: O(*m* · (*V* + *E*)) for *m* blocked nodes.
3. Constructs a new `AStar` on the copy. Its heuristic scale is not cached for the copy, so the first
   query scans every edge again.
4. Searches.

Timed one by one, the search is 14–16% of a replan on the three benchmark warehouses, from 1,107
to 33,786 nodes, and 16% for the README's own replan. On the benchmark warehouses:

- copying is 28–41%;
- removal is 14–18% (33% for the README's 14-node blockage);
- the scale pass is 22–31%.

On the large warehouse that is 44.9 ms per replan at p50, against 5.94 ms for the search alone.
(Numbers from the run in `docs/data`; [EVIDENCE.md](EVIDENCE.md) explains the split. The parts are
timed separately, so they add up to 86–98% of the end-to-end time. The remainder is navigator
bookkeeping and cache effects; nothing on this path prints.) Absolute times moved with machine load
across three full runs that day (30.4 ms, 37.5 ms and 44.9 ms on the large warehouse), but the search
share stayed within 12–18%.

**What would remove most of it, not implemented.**

- Search the original graph and skip blocked nodes in the neighbour loop: a blocked-set lookup per
  edge instead of a copy.
- Reuse the original `AStar`'s scale. Removing nodes only removes edges, which can only raise the
  minimum cost-per-length ratio, so the old *k* stays admissible and consistent on the reduced graph.

## What the navigator reports when no route is left

`NavigationCore::handleObstacles` asks the replanner for a route from the current node. When none
exists, it:

- prints `[NAVIGATION] No route around the reported obstacles; navigation halted`;
- sets itself inactive.

From then on:

- `executeNavigationStep()` returns false, and the position does not change. The navigator never
  steps into a blocked node.
- `getNavigationStatus()` reports `IDLE`, the same word as before navigation starts. There is no
  distinct "no route" state.
- `getCurrentPath()` still returns the old route, through the blocked nodes, and
  `getRemainingDistance()` still sums it.

A caller has to read the status, not the path, to learn that navigation stopped. Both behaviours are
in the README's failure panel, captured from the engine. A clearer contract (a `BLOCKED` status,
clearing the path) would be an API change and is not made here.

## Search observer

`SearchObserver` ([header](../include/pathfinding_algorithms/SearchObserver.hpp)) is one of the two
changes this work made to the library; the other is the [A\* open-list fix](#a-tie-bug-found-by-the-traces),
which the observer's traces made necessary. The observer is opt-in and read-only:

- `AStar`, `Dijkstra` and `BFS` gain `setSearchObserver(SearchObserver*)`.
- They call `onExpand` and `onFrontierPush` where they do the work.
- Nothing else changes: with no observer, each hook is one pointer test.

A* computes its start priority once and its push priority into a local before pushing. These are
the same floating-point expressions as before, so routes are bit-identical. The trace test and the
exporter both run every query with and without an observer and require identical routes.

Why it was needed: `AStar::getLastSearchExploredNodes()` returns the closed set's contents in hash
order, so it cannot show *how* a search proceeded. Animating that set would invent an order.

## Units

Coordinates are raw coordinate units, the x/y stored on each `Node`. `Node::euclideanDistance` and
`manhattanDistance` multiply by 0.1 (`COORDINATE_TO_METER_SCALE`), so the library mixes two scales:

- the demo's console printer reports 8.00 "units" for a route whose edge-weight cost is 80;
- the A\* scale *k* is computed with the same 0.1 factor on both sides, so it cancels and the heuristic
  is unaffected.

The README and its figures report lengths in raw coordinate units (`u`) and costs as edge weights,
never in metres.

## Failure contract of the searches

The searches report "no route" as an empty vector. That covers:

- an unknown start or goal;
- an unreachable goal;
- in Bellman–Ford, a negative cycle.

Exceptions mark inputs an algorithm cannot handle, such as a negative weight in A\* or Dijkstra.
Depth-first search does not follow this yet: it throws for an unknown start or goal.
