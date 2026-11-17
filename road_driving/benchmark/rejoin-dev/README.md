# Pose corrections in the stack: development reruns

The stack change that re-joins the path after a pose correction (`rejoinpolicy` in
[`src/stack.hpp`](../../src/stack.hpp)) was developed after the third evaluation. It was checked by
rerunning the closed loops of networks R3 and R4 at commit `f674183`, with each network's earlier compiler.
Both networks had been used before, so these are development results, not a held-out evaluation.

| Folder | Network | Split | Build | Compared with |
|---|---|---|---|---|
| [`r3/`](r3) | R3 | `r3dev` | GCC 14.2 | [`../r3-dev/`](../r3-dev) (GCC 14.2 at `b7ee7f2`) |
| [`r4/`](r4) | R4 | `r4` | MSVC 19.51 | [`../heldout-r4/`](../heldout-r4) (MSVC 19.51 at `87436de`) |

`r4/closed_loop.md` keeps the `r4` split's title, "held-out evaluation", because the file is written by
the same campaign code. The held-out evaluation itself is [`../heldout-r4/`](../heldout-r4), unchanged.

Each closed loop ran as one process per scenario, merged with
[`tools/merge_closed_loop.py`](../../tools/merge_closed_loop.py):

```bash
road_drive bench-closed --split r4 --seeds 10 --scenario R4-A --out out/rejoin/r4-R4-A
```

The open loops are not rerun: they drive on simulator truth, and the change does not touch the estimators.
The comparison is in [LOCALISATION.md](../../LOCALISATION.md#after-the-third-evaluation-pose-corrections-in-the-stack-development).
