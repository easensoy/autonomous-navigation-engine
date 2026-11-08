# tools/evidence

Everything that turns the library's behaviour into the README's figures and numbers. Nothing here
plans a route: the C++ tools call the library, and the Python scripts only summarise, draw and check
what the C++ tools wrote.

```bash
python -m pip install -r tools/evidence/requirements.txt
python tools/evidence/regenerate.py              # build, export, benchmark, summarise, render, record, check
python tools/evidence/regenerate.py --skip-bench # reuse the saved timings; about two minutes
```

Run it from a shell with CMake, a generator (Ninja by default) and a C++17 compiler on `PATH`; on
Windows, the *x64 Native Tools Command Prompt for VS* (add `--cxx cl`) or a shell with MSYS2 UCRT64
`g++` first on `PATH` (add `--cxx g++`). Before timing, the benchmark waits up to 15 minutes for the
machine's CPU load to fall to 10% (`--load-gate`, `--load-gate-timeout`, `--no-load-gate`); it records
whether that happened.

| File | Role |
|---|---|
| `evidence_export.cpp` | Trace mode. Runs the hero trip, the four-planner comparison and the failure cases; attaches `SearchObserver`; checks every route and the navigator's `BLOCKED` contract; writes `docs/data/*.json`. Also writes the seeded workloads. Times nothing. |
| `evidence_bench.cpp` | Measurement mode. Times planner calls and replans on the saved workloads; no observer; `std::cout` silenced; records other processes' CPU load per command; writes raw CSV to `docs/data/bench/`. |
| `evidence_common.*` | Scenario files, graph construction through `Graph::addNode`/`addEdge`, route checks, JSON writer. |
| `scenarios/` | The README's scenario files and their [format](scenarios/README.md). |
| `summarize.py` | Percentiles, variability and headline numbers from the raw files, to `docs/data/summary.json`. |
| `render/` | matplotlib and Pillow drawing code, light and dark variants, to `docs/assets/`. |
| `provenance.py` | `docs/data/manifest.json`: source revision and dirty-state id, compiler and flags, machine, output hashes. |
| `history.py` | The load gate, and the run history in `docs/data/history/` with its selection rule. |
| `check_outputs.py` | Re-checks routes against the exported graph by plain arithmetic, the workload fingerprints, A\*'s cost against Dijkstra's on every benchmark query and replan, each A\* miss recorded before the open-list fix (`docs/data/astar_fix_before.json`), the failure states, the run history, and every number and link in the README. Standard library only. |
| `test_check_outputs.py` | Mutation test: corrupts a copy of the evidence six ways and requires `check_outputs.py` to fail on each. |
| `compare_traces.py` | Reruns `evidence_export` and requires the committed traces back exactly. |
| `compare_bench.py` | Paired per-query latency ratios between two benchmark runs (used for the validation runs in `docs/EVIDENCE.md`). |
| `preview_readme.py` | Renders `README.md` to a local page styled like GitHub's, for checking layout and light/dark images. |
| `regenerate.py` | Runs the stages in order. |

CI runs `check_outputs.py`, `test_check_outputs.py` and `compare_traces.py` on every push, and never
the benchmark. How the measurements are taken, what each file contains and which figure comes from
which data: [docs/EVIDENCE.md](../../docs/EVIDENCE.md).
