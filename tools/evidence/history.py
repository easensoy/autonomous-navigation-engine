"""Measurement environment and run history for the benchmark.

Two jobs:
  1. Before the benchmark, wait for a quiet machine (load gate) and record what was measured.
  2. After a benchmark run, keep its summary, manifest and run records under docs/data/history, so a
     later run never replaces an earlier one. docs/data/history/index.json lists every run with its
     source state, environment and headline numbers.

Selection rule (also in index.json and docs/EVIDENCE.md): the README, docs/data/summary.json and the
figures use the newest complete run made at the published source revision. Earlier runs stay in the
history whatever their results.
"""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import time
from pathlib import Path

SELECTION_RULE = ("The README, docs/data/summary.json and the figures use the newest complete benchmark run made "
                  "at the published source revision. Every run stays in this history, whatever its results; "
                  "none is replaced or dropped.")
WORKLOADS = ("warehouse_s", "warehouse_m", "warehouse_l")
VARIANTS = ("astar", "astar_zero", "dijkstra", "bfs")


# --- system load ---------------------------------------------------------------------------------

def _cpu_times():
    """(busy, total) CPU seconds summed over all logical processors, or None if unavailable."""
    if os.name == "nt":
        import ctypes
        from ctypes import wintypes

        class FILETIME(ctypes.Structure):
            _fields_ = [("low", wintypes.DWORD), ("high", wintypes.DWORD)]

        idle, kernel, user = FILETIME(), FILETIME(), FILETIME()
        if not ctypes.windll.kernel32.GetSystemTimes(ctypes.byref(idle), ctypes.byref(kernel), ctypes.byref(user)):
            return None
        value = lambda t: ((t.high << 32) | t.low) * 1e-7  # noqa: E731
        total = value(kernel) + value(user)  # kernel time includes idle time
        return total - value(idle), total
    try:
        fields = Path("/proc/stat").read_text().splitlines()[0].split()[1:]
    except OSError:
        return None
    ticks = [int(f) for f in fields[:8]]
    user, nice, system, idle, iowait, irq, softirq, steal = ticks
    busy = user + nice + system + irq + softirq + steal
    return float(busy), float(busy + idle + iowait)


def busy_fraction(seconds: float):
    """Share of the machine's CPU capacity in use over the next `seconds`, or None."""
    a = _cpu_times()
    time.sleep(seconds)
    b = _cpu_times()
    if a is None or b is None or b[1] <= a[1]:
        return None
    return max(0.0, min(1.0, (b[0] - a[0]) / (b[1] - a[1])))


def wait_for_quiet(threshold: float, window: float = 10.0, consecutive: int = 2, timeout: float = 900.0) -> dict:
    """Waits until `consecutive` windows in a row stay at or below `threshold`, or until `timeout`.
    The benchmark starts either way; the result records whether the gate was met."""
    started = time.time()
    samples, quiet = [], 0
    while True:
        load = busy_fraction(window)
        samples.append(None if load is None else round(load, 4))
        if load is None:
            break
        quiet = quiet + 1 if load <= threshold else 0
        if quiet >= consecutive or time.time() - started >= timeout:
            break
    return {"threshold": threshold, "window_s": window, "consecutive_windows": consecutive, "timeout_s": timeout,
            "waited_s": round(time.time() - started, 1), "window_loads": samples,
            "met": bool(samples) and samples[-1] is not None and quiet >= consecutive,
            "definition": "share of all logical processors' time in use by any process, per window, sampled "
                          "before the benchmark starts"}


# --- history -------------------------------------------------------------------------------------

def headline(summary: dict) -> dict:
    """The numbers a reader compares across runs, taken from a summary.json."""
    h, q, rp = summary["hero"], summary["queries"], summary["replans"]
    return {
        "hero_plan_ms_p50": h["timing_ms"]["plan"]["p50"],
        "hero_replan_ms_p50": h["timing_ms"]["replan"]["p50"],
        "replan_search_share": {**{n: rp[n]["share_of_end_to_end_median"]["search"] for n in WORKLOADS},
                                "hero": h["replan_share_median"]["search"]},
        "replan_ms_p50": {n: rp[n]["phases_ms"]["end_to_end"]["p50"] for n in WORKLOADS},
        "astar_query_ms_p50": {n: q[n]["variants"]["astar"]["latency_ms"]["p50"] for n in WORKLOADS},
        "dijkstra_over_astar_paired_p50": {n: q[n]["dijkstra_over_astar_paired"]["p50"] for n in WORKLOADS},
        "astar_zero_over_astar_paired_p50": {n: q[n]["astar_zero_over_astar_paired"]["p50"] for n in WORKLOADS},
        "astar_optimal": {n: f"{q[n]['variants']['astar']['optimal']}/{q[n]['queries']}" for n in WORKLOADS},
        "repeat_spread_median": {n: {v: q[n]["variants"][v]["repeat_spread_median"] for v in VARIANTS}
                                 for n in WORKLOADS},
    }


def _run_records(bench: Path) -> list[Path]:
    return sorted(bench.glob("*_run.json")) + sorted(bench.glob("*.run.json"))


def records_digest(bench: Path) -> str:
    h = hashlib.sha256()
    for path in _run_records(bench):
        h.update(path.name.encode() + b"\0" + path.read_bytes())
    return h.hexdigest()


def _background_loads(bench: Path) -> dict:
    loads = {}
    for path in _run_records(bench):
        record = json.loads(path.read_text(encoding="utf-8"))
        if "background_cpu_load" in record:
            loads[path.name.replace("_run.json", "").replace(".run.json", "")] = record["background_cpu_load"]
    return loads


def load_index(history: Path) -> dict:
    path = history / "index.json"
    if path.exists():
        return json.loads(path.read_text(encoding="utf-8"))
    return {"schema": "ane-evidence/history/1", "selection_rule": SELECTION_RULE, "runs": []}


def save_index(history: Path, index: dict) -> None:
    history.mkdir(parents=True, exist_ok=True)
    (history / "index.json").write_text(json.dumps(index, indent=1) + "\n", encoding="utf-8")


def record_run(data: Path, note: str = "", history: Path | None = None) -> str:
    """Copies a run (data/summary.json, data/manifest.json, data/bench run records and environment)
    into the history (default data/history). Returns the run's folder name, or an empty string if
    this run is already recorded."""
    bench = data / "bench"
    history = history or data / "history"
    index = load_index(history)
    digest = records_digest(bench)
    if any(run.get("bench_records_sha256") == digest for run in index["runs"]):
        return ""
    manifest = json.loads((data / "manifest.json").read_text(encoding="utf-8"))
    summary = json.loads((data / "summary.json").read_text(encoding="utf-8"))
    source = manifest["source"]
    stamp = manifest["generated_utc"][:16].replace(":", "-")
    state = source["head"][:7] + (f"-{source['dirty_state_id']}" if source.get("dirty") else "")
    folder = f"{stamp}_{state}"
    target = history / folder
    (target / "bench").mkdir(parents=True, exist_ok=True)
    shutil.copy2(data / "summary.json", target / "summary.json")
    shutil.copy2(data / "manifest.json", target / "manifest.json")
    for path in _run_records(bench) + [p for p in [bench / "environment.json"] if p.exists()]:
        shutil.copy2(path, target / "bench" / path.name)
    environment = {}
    if (bench / "environment.json").exists():
        environment = json.loads((bench / "environment.json").read_text(encoding="utf-8"))
    index["runs"].append({
        "folder": folder,
        "generated_utc": manifest["generated_utc"],
        "source": {"head": source["head"], "dirty": source.get("dirty"), "dirty_state_id": source.get("dirty_state_id")},
        "compiler": manifest.get("build", {}).get("cmake_cache", {}).get("CMAKE_CXX_COMPILER"),
        "machine": manifest.get("machine"),
        "load_gate": environment.get("load_gate"),
        "background_cpu_load": _background_loads(bench),
        "bench_records_sha256": digest,
        "headline": headline(summary),
        "note": note,
    })
    save_index(history, index)
    return folder
