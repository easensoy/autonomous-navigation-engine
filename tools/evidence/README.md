# tools/evidence

Everything that turns the library's behaviour into the README's figures and numbers. Nothing here
plans a route: the C++ tools call the library, and the Python scripts only summarise and draw what
the C++ tools wrote.

```bash
python -m pip install -r tools/evidence/requirements.txt
python tools/evidence/regenerate.py              # build, export, benchmark, summarise, render, check
python tools/evidence/regenerate.py --skip-bench # reuse the saved timings; about a minute
```

Run it from a shell with CMake, a generator (Ninja by default) and a C++17 compiler on `PATH`; on
Windows, the *x64 Native Tools Command Prompt for VS* (add `--cxx cl`) or a shell with MSYS2 UCRT64
`g++` first on `PATH` (add `--cxx g++`).

| File | Role |
|---|---|
| `evidence_export.cpp` | Trace mode. Runs the hero trip, the four-planner comparison and the failure cases; attaches `SearchObserver`; checks every route; writes `docs/data/*.json`. Also writes the seeded workloads. Times nothing. |
| `evidence_bench.cpp` | Measurement mode. Times planner calls and replans on the saved workloads; no observer; `std::cout` silenced; writes raw CSV to `docs/data/bench/`. |
| `evidence_common.*` | Scenario files, graph construction through `Graph::addNode`/`addEdge`, route checks, JSON writer. |
| `scenarios/` | The README's scenario files and their [format](scenarios/README.md). |
| `summarize.py` | Percentiles and headline numbers from the raw files, to `docs/data/summary.json`. |
| `render/` | matplotlib and Pillow drawing code, light and dark variants, to `docs/assets/`. |
| `provenance.py` | `docs/data/manifest.json`: source revision and dirty-state id, compiler and flags, machine, output hashes. |
| `check_outputs.py` | Re-checks routes against the exported graph by plain arithmetic, the workload fingerprints, A\*'s cost against Dijkstra's on every benchmark query and replan, each A\* miss recorded before the open-list fix (`docs/data/astar_fix_before.json`), and every number the README quotes. |
| `regenerate.py` | Runs the stages in order. |

How the measurements are taken, what each file contains and which figure comes from which data:
[docs/EVIDENCE.md](../../docs/EVIDENCE.md).
