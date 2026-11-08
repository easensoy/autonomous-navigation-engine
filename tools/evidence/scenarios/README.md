# Scenario files

Plain-text descriptions of the warehouse graphs the evidence tools build. `evidence_export` and
`evidence_bench` read them (`tools/evidence/evidence_common.cpp`, `loadScenario`) and build the
graph through the library's public API, `Graph::addNode` and `Graph::addEdge`.

One directive per line; `#` starts a comment.

| Directive | Meaning |
|---|---|
| `include <file>` | Read another scenario file first (path relative to this one). |
| `name <word>` / `title <text>` | Identification only. |
| `lattice <xmin> <ymin> <xmax> <ymax>` | Integer bounds, inclusive. One node per integer point not covered by a shelf. |
| `connectivity 4\|8` | Neighbours joined by two-way edges; 8 adds diagonals. |
| `shelf <x0> <y0> <x1> <y1> [label]` | Closed rectangle. Nodes inside it are not created, and no edge segment may touch it, so diagonals never cut a shelf corner. |
| `zone <x0> <y0> <x1> <y1> <multiplier> [label]` | Cost zone. A node inside takes the multiplier (the largest, if zones overlap; 1 elsewhere). |
| `start <x> <y>` / `goal <x> <y>` | Lattice points of the trip. |
| `block <x0> <y0> <x1> <y1> [label]` | Nodes inside are reported to `NavigationCore::handleObstacles` as obstacles... |
| `block_step <k>` | ...after `k` navigation steps. |
| `point <name> <x> <y>` | A named lattice point. |

**Edge cost** = straight-line length in raw coordinate units × the mean of the two endpoint
multipliers. An edge between a normal node and a picking-zone node (×3) therefore costs 2× its
length; an edge inside the zone costs 3×.

**Node ids** are `(y - ymin) * (xmax - xmin + 1) + (x - xmin)`.

**Units.** Coordinates and lengths are raw coordinate units (`Node::getX`, `Node::getY`), written `u`
in the figures. They are not metres: the library's `Node::euclideanDistance` multiplies by 0.1, and the
evidence tools do not use it for anything they report. Costs are edge weights.

Files here:

- `warehouse_layout.txt`: the 47 × 24 warehouse used by the README (744 nodes, 2,249 edges).
- `hero_replan.txt`: the hero trip; aisle B is blocked at step 9.
- `failure_goal_blocked.txt`, `failure_disconnected.txt`: the failure panel's two cases.

The benchmark warehouses in `docs/data/workloads/` use the same format; `evidence_export workloads`
writes them from fixed seeds.
