"""Provenance manifest for the generated data: what code, built how, ran where, produced which files."""

from __future__ import annotations

import ctypes
import datetime as dt
import hashlib
import json
import os
import platform
import re
import subprocess
from pathlib import Path

# Everything that can change what the tools compute. Markdown is excluded: it never feeds the build,
# and an untracked note (such as include/README.md) should not mark the source state dirty.
BUILD_INPUTS = ("CMakeLists.txt", "include", "src", "tests", "tools", ":(exclude,glob)**/*.md")


def git(repo: Path, *args: str, binary=False):
    out = subprocess.run(["git", "-C", str(repo), *args], check=True, capture_output=True)
    return out.stdout if binary else out.stdout.decode("utf-8", "replace").strip()


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def status_entries(repo: Path):
    """(two-letter status, path) for each changed build input, from NUL-separated porcelain output.
    Not stripped: the status of the first entry can start with a space (" M path")."""
    fields = git(repo, "status", "--porcelain", "-z", "--untracked-files=all", "--", *BUILD_INPUTS,
                 binary=True).decode("utf-8", "replace").split("\0")
    entries, i = [], 0
    while i < len(fields):
        field = fields[i]
        i += 1
        if not field:
            continue
        entries.append((field[:2], field[3:]))
        if field[0] in "RC":
            i += 1  # a rename or copy is followed by its source path
    return entries


def source_state(repo: Path) -> dict:
    """HEAD plus an identifier for uncommitted changes to anything that feeds the build."""
    head = git(repo, "rev-parse", "HEAD")
    entries = status_entries(repo)
    modified = [path for code, path in entries if code != "??"]
    untracked = sorted(path for code, path in entries if code == "??")
    h = hashlib.sha256()
    h.update(git(repo, "diff", "HEAD", "--binary", "--", *BUILD_INPUTS, binary=True))
    for rel in untracked:
        p = repo / rel
        if p.is_file():
            # Hash with LF line endings so the identifier matches across checkouts with autocrlf.
            h.update(rel.encode() + b"\0" + p.read_bytes().replace(b"\r\n", b"\n") + b"\0")
    return {
        "head": head,
        "head_subject": git(repo, "log", "-1", "--format=%s"),
        "dirty": bool(modified or untracked),
        "dirty_state_id": h.hexdigest()[:16] if (modified or untracked) else None,
        "dirty_state_id_definition": "first 16 hex digits of sha256(git diff HEAD --binary over the build inputs "
                                     "(CMakeLists.txt, include, src, tests, tools; Markdown excluded), then each "
                                     "untracked build-input file's path and LF-normalised content)",
        "modified_files": modified,
        "untracked_build_inputs": untracked,
    }


def cpu_name() -> str:
    if os.name == "nt":
        try:
            import winreg

            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DESCRIPTION\System\CentralProcessor\0")
            return winreg.QueryValueEx(key, "ProcessorNameString")[0].strip()
        except OSError:
            pass
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor() or "unknown"


def memory_bytes():
    if os.name == "nt":
        class MemoryStatus(ctypes.Structure):
            _fields_ = [("dwLength", ctypes.c_ulong), ("dwMemoryLoad", ctypes.c_ulong),
                        ("ullTotalPhys", ctypes.c_ulonglong), ("ullAvailPhys", ctypes.c_ulonglong),
                        ("ullTotalPageFile", ctypes.c_ulonglong), ("ullAvailPageFile", ctypes.c_ulonglong),
                        ("ullTotalVirtual", ctypes.c_ulonglong), ("ullAvailVirtual", ctypes.c_ulonglong),
                        ("ullAvailExtendedVirtual", ctypes.c_ulonglong)]
        status = MemoryStatus()
        status.dwLength = ctypes.sizeof(MemoryStatus)
        if ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(status)):
            return status.ullTotalPhys
    try:
        for line in Path("/proc/meminfo").read_text().splitlines():
            if line.startswith("MemTotal"):
                return int(line.split()[1]) * 1024
    except OSError:
        pass
    return None


def power_state():
    state = {}
    if os.name == "nt":
        class PowerStatus(ctypes.Structure):
            _fields_ = [("ACLineStatus", ctypes.c_ubyte), ("BatteryFlag", ctypes.c_ubyte),
                        ("BatteryLifePercent", ctypes.c_ubyte), ("SystemStatusFlag", ctypes.c_ubyte),
                        ("BatteryLifeTime", ctypes.c_ulong), ("BatteryFullLifeTime", ctypes.c_ulong)]
        status = PowerStatus()
        if ctypes.windll.kernel32.GetSystemPowerStatus(ctypes.byref(status)):
            state["ac_power"] = {0: False, 1: True}.get(status.ACLineStatus)
        try:
            out = subprocess.run(["powercfg", "/getactivescheme"], capture_output=True, text=True).stdout
            match = re.search(r"\(([^)]+)\)", out)
            state["power_plan"] = match.group(1) if match else out.strip()
        except OSError:
            pass
    return state


def compile_flags(build_dir: Path) -> dict:
    path = build_dir / "compile_commands.json"
    if not path.exists():
        return {}
    entries = json.loads(path.read_text())
    wanted = {"evidence_bench.cpp": "benchmark", "a_star_core.cpp": "library (A*)"}
    flags = {}
    for entry in entries:
        name = Path(entry["file"]).name
        if name in wanted:
            command = entry.get("command") or " ".join(entry.get("arguments", []))
            # Drop the source/object paths; keep the compiler and flags.
            command = re.sub(r'\s(/Fo|-o\s)\S+', "", command)
            command = re.sub(r'\s(/Fd)\S+', "", command)
            flags[wanted[name]] = command.replace(str(build_dir.parent.parent), "<repo>")
    return flags


def cmake_cache(build_dir: Path) -> dict:
    keys = ("CMAKE_BUILD_TYPE", "CMAKE_CXX_COMPILER", "CMAKE_GENERATOR", "CMAKE_CXX_FLAGS",
            "CMAKE_CXX_FLAGS_RELEASE")
    out = {}
    cache = build_dir / "CMakeCache.txt"
    if cache.exists():
        for line in cache.read_text(errors="replace").splitlines():
            for key in keys:
                if line.startswith(key + ":"):
                    out[key] = line.split("=", 1)[1]
    return out


def bench_records(data: Path) -> list:
    records = []
    for path in sorted((data / "bench").glob("*.json")):
        record = json.loads(path.read_text())
        records.append({k: record.get(k) for k in (
            "command", "workload", "compiler", "clock_step_ns", "pinning", "repeats", "warmup", "graph_nodes",
            "graph_edges", "graph_fingerprint", "samples", "invalid_routes", "logging", "observers", "timer")}
                       | {"file": path.name})
    return records


def tool_hashes(repo: Path, build_dir: Path) -> dict:
    """SHA-256 of the evidence tools' sources and of the binaries that produced the data."""
    out = {}
    for name in ("evidence_common.cpp", "evidence_common.hpp", "evidence_export.cpp", "evidence_bench.cpp"):
        path = repo / "tools" / "evidence" / name
        if path.exists():
            out[f"tools/evidence/{name}"] = sha256(path.read_bytes().replace(b"\r\n", b"\n"))
    for name in ("evidence_export", "evidence_bench"):
        for suffix in (".exe", ""):
            path = build_dir / "tools" / "evidence" / (name + suffix)
            if path.is_file():
                out[f"{name}{suffix} (binary)"] = file_sha256(path)
                break
    return out


def output_hashes(repo: Path) -> dict:
    hashes = {}
    for folder in ("docs/data", "docs/assets"):
        for path in sorted((repo / folder).rglob("*")):
            if path.is_file() and path.name != "manifest.json":
                hashes[path.relative_to(repo).as_posix()] = file_sha256(path)
    return hashes


def write_manifest(repo: Path, build_dir: Path, out: Path, *, quick: bool):
    import matplotlib
    import numpy
    import PIL

    data = out.parent
    manifest = {
        "schema": "ane-evidence/manifest/1",
        "generated_utc": dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat(),
        "regenerate": "python tools/evidence/regenerate.py",
        "quick_mode": quick,
        "source": source_state(repo),
        "build": {"directory": build_dir.relative_to(repo).as_posix() if build_dir.is_relative_to(repo) else str(build_dir),
                  "cmake_cache": cmake_cache(build_dir), "compile_commands": compile_flags(build_dir)},
        "machine": {
            "cpu": cpu_name(),
            "logical_processors": os.cpu_count(),
            "memory_bytes": memory_bytes(),
            "os": platform.platform(),
            **power_state(),
        },
        "python": {"version": platform.python_version(), "numpy": numpy.__version__,
                   "matplotlib": matplotlib.__version__, "pillow": PIL.__version__},
        "timing_policy": {
            "timer": "std::chrono::steady_clock read immediately before and after each planner call",
            "excluded_from_timing": "graph construction, workload loading, route validation, CSV/JSON writing, "
                                    "tracing (no observer is attached in evidence_bench), rendering",
            "logging": "std::cout has no stream buffer during benchmarks; BFS::findPath still executes its "
                       "per-node print statements, which then return after a stream-state check",
            "statistic": "per query: median of the timed repeats; percentiles are across queries (or replan events)",
        },
        "bench_runs": bench_records(data),
        "tools_sha256": tool_hashes(repo, build_dir),
        "outputs_sha256": output_hashes(repo),
    }
    out.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {out.relative_to(repo)}")
