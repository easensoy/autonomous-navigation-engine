#!/usr/bin/env python3
"""Regenerate every README figure and number from the C++ library's own output.

    python tools/evidence/regenerate.py            # build, export, benchmark, summarise, render, check
    python tools/evidence/regenerate.py --quick    # fewer timing repeats (figures only, not for publishing)
    python tools/evidence/regenerate.py --skip-bench --skip-build   # re-render from saved data

Stages (each writes only under docs/data, docs/assets or build/):
  build     configure + build tools/evidence in build/evidence (CMake, Release, -DBUILD_EVIDENCE_TOOLS=ON)
  export    evidence_export: hero, search comparison, failure cases  -> docs/data/*.json
  bench     evidence_bench: queries, replans, hero timings            -> docs/data/bench/*.csv, *.json
  summary   percentiles and headline numbers from the raw files      -> docs/data/summary.json
  render    figures (light and dark)                                 -> docs/assets/*.png, *.gif
  manifest  provenance: git revision and dirty state, compiler, flags, hardware, output hashes
                                                                     -> docs/data/manifest.json
  check     re-validates exported routes and the README's numbers against the data

Run it from a shell where CMake, Ninja (or another generator) and a C++17 compiler are on PATH: on
Windows the "x64 Native Tools Command Prompt for VS" for MSVC, or a shell with MSYS2 UCRT64 g++.
Python needs numpy, matplotlib and pillow (see tools/evidence/requirements.txt).
"""

from __future__ import annotations

import argparse
import os
import platform
import shutil
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
TOOLS = REPO / "tools" / "evidence"
SCENARIOS = TOOLS / "scenarios"
DATA = REPO / "docs" / "data"
BENCH = DATA / "bench"
WORKLOADS = DATA / "workloads"
ASSETS = REPO / "docs" / "assets"

sys.path.insert(0, str(TOOLS))


def run(cmd, **kwargs):
    printable = " ".join(f'"{c}"' if " " in str(c) else str(c) for c in cmd)
    print(f"$ {printable}", flush=True)
    started = time.perf_counter()
    subprocess.run([str(c) for c in cmd], check=True, **kwargs)
    print(f"  ({time.perf_counter() - started:.1f} s)", flush=True)


def exe(build_dir: Path, name: str) -> Path:
    suffix = ".exe" if os.name == "nt" else ""
    path = build_dir / "tools" / "evidence" / (name + suffix)
    if not path.exists():
        raise SystemExit(f"{path} not found; run the build stage first")
    return path


def stage_build(args):
    cmd = ["cmake", "-S", REPO, "-B", args.build_dir, "-DCMAKE_BUILD_TYPE=Release", "-DBUILD_EVIDENCE_TOOLS=ON",
           "-DBUILD_TESTS=OFF", "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"]
    if args.generator:
        cmd += ["-G", args.generator]
    if args.cxx:
        cmd += [f"-DCMAKE_CXX_COMPILER={args.cxx}"]
    run(cmd)
    run(["cmake", "--build", args.build_dir, "--target", "evidence_export", "evidence_bench", "--parallel"])


def stage_export(args):
    DATA.mkdir(parents=True, exist_ok=True)
    tool = exe(args.build_dir, "evidence_export")
    run([tool, "hero", SCENARIOS / "hero_replan.txt", DATA / "hero.json"])
    run([tool, "search", SCENARIOS / "hero_replan.txt", DATA / "search.json"])
    run([tool, "failure", SCENARIOS / "failure_goal_blocked.txt", DATA / "failure_goal_blocked.json"])
    run([tool, "failure", SCENARIOS / "failure_disconnected.txt", DATA / "failure_disconnected.json"])
    if args.regenerate_workloads or not (WORKLOADS / "workloads.json").exists():
        WORKLOADS.mkdir(parents=True, exist_ok=True)
        run([tool, "workloads", WORKLOADS])


def stage_bench(args):
    BENCH.mkdir(parents=True, exist_ok=True)
    tool = exe(args.build_dir, "evidence_bench")
    repeats = "1" if args.quick else "5"
    warmup = "0" if args.quick else "1"
    for name in ("warehouse_s", "warehouse_m", "warehouse_l"):
        run([tool, "queries", WORKLOADS, name, BENCH, repeats, warmup])
        run([tool, "replans", WORKLOADS, name, BENCH, repeats])
    run([tool, "hero", SCENARIOS / "hero_replan.txt", BENCH, "100" if args.quick else "1000"])


def stage_manifest(args):
    import provenance

    provenance.write_manifest(REPO, args.build_dir, DATA / "manifest.json", quick=args.quick)


def stage_summary(args):
    import summarize

    summarize.write_summary(DATA, DATA / "summary.json")


def stage_render(args):
    sys.path.insert(0, str(TOOLS / "render"))
    import render_all

    render_all.main(themes=args.themes, gif=not args.no_gif)


def stage_check(args):
    import check_outputs

    if not check_outputs.main(REPO):
        raise SystemExit("check stage failed")


STAGES = {
    "build": stage_build,
    "export": stage_export,
    "bench": stage_bench,
    "summary": stage_summary,
    "render": stage_render,
    "manifest": stage_manifest,
    "check": stage_check,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--build-dir", type=Path, default=REPO / "build" / "evidence")
    parser.add_argument("--generator", default="Ninja" if shutil.which("ninja") else None)
    parser.add_argument("--cxx", default=None, help="C++ compiler for the build stage (default: CMake's choice)")
    parser.add_argument("--quick", action="store_true", help="1 timing repeat, no warm-up (not for publishing)")
    parser.add_argument("--regenerate-workloads", action="store_true",
                        help="rewrite docs/data/workloads from the seeds (they are checked in)")
    parser.add_argument("--skip-build", action="store_true")
    parser.add_argument("--skip-bench", action="store_true")
    parser.add_argument("--only", nargs="+", choices=list(STAGES), help="run only these stages, in order")
    parser.add_argument("--themes", nargs="+", default=["light", "dark"])
    parser.add_argument("--no-gif", action="store_true", help="skip the animation (fast re-render)")
    args = parser.parse_args()
    args.build_dir = args.build_dir.resolve()

    stages = args.only or [s for s in STAGES
                           if not (s == "build" and args.skip_build) and not (s == "bench" and args.skip_bench)]
    print(f"Python {platform.python_version()} on {platform.platform()}; stages: {', '.join(stages)}")
    for name in stages:
        print(f"\n== {name} ==", flush=True)
        STAGES[name](args)
    print("\nDone.")


if __name__ == "__main__":
    main()
