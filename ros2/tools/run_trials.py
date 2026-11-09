#!/usr/bin/env python3
"""Runs the scenario trials, one fresh ane-sim container per trial, and records every attempt.

  python ros2/tools/run_trials.py --campaign <name> [--scenarios A_nominal B_aisle_blockage ...] [--trials N]

For each scenario and trial k it derives the seed (scenario seed + k) and the initial pose (seeded
jitter around the start station), runs ros2/tools/container_trial.sh in a new container, and writes

  ros2/evidence/trials/<campaign>/<scenario>/trial_<k>/manifest.json   inputs of the attempt
  ... runner_result.json, metrics.json, trajectory.json                (from the container)
  ros2/evidence/trials/<campaign>/index.json                           every attempt and its status
  build/ros2-trials/<campaign>/<scenario>/trial_<k>/                   bag and logs (not in Git)

A clock check before and after the campaign measures the container's monotonic clock against the
host's (Docker Desktop VMs drift; see ros2/README.md, "Clocks"). Standard library only; needs Docker.
"""

import argparse
import hashlib
import json
import platform
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "ros2" / "ane_sim" / "scripts"))
from scenario_lib import integration_tree_sha256, load_roadmap, load_scenario  # noqa: E402

SCENARIOS = ["A_nominal", "B_aisle_blockage", "C_disconnection", "D_feedback_dropout", "E_feedback_loss", "F_drive_stall"]


def sh(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def host_description():
    info = {"os": platform.platform(), "python": platform.python_version()}
    if platform.system() == "Windows":
        ps = ("$c=Get-CimInstance Win32_Processor|Select -First 1; $s=Get-CimInstance Win32_ComputerSystem; "
              "\"$($c.Name)|$($c.NumberOfCores)|$($c.NumberOfLogicalProcessors)|$($s.TotalPhysicalMemory)|$($s.Model)\"")
        out = sh(["powershell", "-NoProfile", "-Command", ps]).stdout.strip().split("|")
        if len(out) == 5:
            info.update({"cpu": out[0].strip(), "cores": int(out[1]), "logical_processors": int(out[2]),
                         "ram_gib": round(int(out[3]) / 2**30, 1), "model": out[4].strip()})
    gpu = sh(["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"])
    info["gpu"] = gpu.stdout.strip() if gpu.returncode == 0 else "unknown"
    info["rendering"] = "software (Mesa llvmpipe in the container); the GPU is not used"
    docker = sh(["docker", "info", "--format", "{{.ServerVersion}}|{{.NCPU}}|{{.MemTotal}}|{{.OperatingSystem}}|{{.KernelVersion}}"])
    v = docker.stdout.strip().split("|")
    if len(v) == 5:
        info["docker"] = {"server": v[0], "cpus": int(v[1]), "memory_gib": round(int(v[2]) / 2**30, 1), "os": v[3], "kernel": v[4]}
    return info


def clock_check(image, seconds=60):
    """Rate of the container's monotonic clock against the host's, and steps of its system clock.

    The container prints a line when it starts timing and another when `seconds` of its monotonic clock
    have passed; the host times the interval between the two lines with its own monotonic clock.
    """
    code = ("import sys, time\nm0=time.monotonic(); r0=time.time(); prev=r0; steps=[]\n"
            "print('start', flush=True)\n"
            f"while time.monotonic()-m0 < {seconds}:\n"
            "    time.sleep(0.005); r=time.time()\n"
            "    if r-prev < -0.05 or r-prev > 0.2: steps.append(round(r-prev, 3))\n"
            "    prev=r\n"
            "print('end', round(time.monotonic()-m0, 4), round(time.time()-r0, 4), steps, flush=True)\n")
    proc = subprocess.Popen(["docker", "run", "--rm", image, "python3", "-u", "-c", code], stdout=subprocess.PIPE, text=True)
    first = proc.stdout.readline()
    t0 = time.monotonic()
    last = proc.stdout.readline()
    t1 = time.monotonic()
    proc.wait()
    if not first.startswith("start") or not last.startswith("end"):
        return {"error": f"clock check failed: {first!r} {last!r}"}
    _, mono, real, steps = last.split(maxsplit=3)
    host = t1 - t0
    return {"container_monotonic_s": float(mono), "container_realtime_s": float(real),
            "container_realtime_steps_s": json.loads(steps.replace("'", '"')),
            "host_monotonic_s": round(host, 4),
            "container_monotonic_rate": round(float(mono) / host, 5),
            "definition": "container_monotonic_rate = container monotonic seconds per host second over the window"}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--campaign", required=True)
    parser.add_argument("--scenarios", nargs="+", default=SCENARIOS)
    parser.add_argument("--trials", type=int, help="override every scenario's trial count")
    parser.add_argument("--image", default="ane-sim:jazzy")
    parser.add_argument("--volume", default="ane-ws")
    parser.add_argument("--note", default="")
    parser.add_argument("--capture", action="store_true",
                        help="run each trial with the Gazebo GUI and RViz2 on a virtual display and record the screen "
                             "(ros2/tools/container_capture.sh); frames go to the build directory")
    args = parser.parse_args()

    out_root = REPO / "ros2" / "evidence" / "trials" / args.campaign
    bag_root = REPO / "build" / "ros2-trials" / args.campaign
    out_root.mkdir(parents=True, exist_ok=True)
    roadmap_file = REPO / "ros2" / "ane_sim" / "maps" / "warehouse_roadmap.txt"
    roadmap = load_roadmap(roadmap_file)
    head = sh(["git", "-C", str(REPO), "rev-parse", "HEAD"]).stdout.strip()
    engine_dirty = sh(["git", "-C", str(REPO), "status", "--porcelain", "--", "src", "include", "CMakeLists.txt"]).stdout.strip()
    integration_dirty = sh(["git", "-C", str(REPO), "status", "--porcelain", "--", "ros2", ":!ros2/evidence"]).stdout.strip()
    image_id = sh(["docker", "image", "inspect", args.image, "--format", "{{.Id}}"]).stdout.strip()
    versions = sh(["docker", "run", "--rm", args.image, "cat", "/opt/ane/versions.txt"]).stdout
    campaign = {
        "schema": "ane-sim/campaign/1",
        "campaign": args.campaign,
        "note": args.note,
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "engine_commit": head,
        "engine_sources_modified": bool(engine_dirty),
        "engine_source_changes": engine_dirty.splitlines(),
        "integration_tree_sha256": integration_tree_sha256(REPO / "ros2"),
        "integration_tree_definition": "scenario_lib.integration_tree_sha256: every file under ros2/ except evidence/ and *.md, LF line endings",
        "integration_sources_uncommitted": integration_dirty.splitlines(),
        "image": {"tag": args.image, "id": image_id, "versions": versions.strip().splitlines()},
        "host": host_description(),
        "timing": {"physics_step_s": 0.001, "target_real_time_factor": 1.0, "control_rate_hz": 20,
                   "ground_truth_rate_hz": 50, "lidar_rate_hz": 10, "gate_rate_hz_wall": 20},
        "files_sha256": {"roadmap": sha256(roadmap_file),
                         "world": sha256(REPO / "ros2/ane_sim/worlds/warehouse.sdf"),
                         "navigator_params": sha256(REPO / "ros2/ane_navigation/config/navigator.yaml"),
                         "robot_xacro": sha256(REPO / "ros2/ane_description/urdf/ane_amr.urdf.xacro")},
        "clock_check_before": clock_check(args.image),
    }
    (out_root / "campaign.json").write_text(json.dumps(campaign, indent=1) + "\n", encoding="utf-8")
    index_path = out_root / "index.json"
    index = json.loads(index_path.read_text()) if index_path.exists() else {"schema": "ane-sim/trial-index/1", "attempts": []}

    for name in args.scenarios:
        scenario = load_scenario(REPO / "ros2" / "ane_sim" / "scenarios" / f"{name}.txt")
        start_xy = roadmap["nodes"][roadmap["stations"][scenario.start]]
        for k in range(args.trials or scenario.trials):
            seed = scenario.seed + k
            x, y, yaw = scenario.initial_pose(start_xy, k)
            out_dir = out_root / name / f"trial_{k}"
            bag_dir = bag_root / name / f"trial_{k}"
            out_dir.mkdir(parents=True, exist_ok=True)
            manifest = {"schema": "ane-sim/trial-manifest/1", "campaign": args.campaign, "scenario": name,
                        "scenario_file": f"ros2/ane_sim/scenarios/{name}.txt", "scenario_sha256": scenario.sha256(),
                        "trial": k, "seed": seed, "initial_pose": {"x": x, "y": y, "yaw": yaw},
                        "engine_commit": head, "image_id": image_id,
                        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
            (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=1) + "\n", encoding="utf-8")
            container_out = f"/ane/ros2/evidence/trials/{args.campaign}/{name}/trial_{k}"
            if args.capture:
                script, last = "container_capture.sh", f"/ane/build/ros2-trials/{args.campaign}/{name}/trial_{k}"
            else:
                script, last = "container_trial.sh", f"/ane/build/ros2-trials/{args.campaign}/{name}/trial_{k}/bag"
            manifest["mode"] = "capture (Gazebo GUI and RViz2 on Xvfb, screen recorded)" if args.capture else "headless"
            (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=1) + "\n", encoding="utf-8")
            command = ["docker", "run", "--rm", "--name", f"ane-trial-{name.lower()}-{k}",
                       "-v", f"{REPO}:/ane", "-v", f"{args.volume}:/ws", args.image,
                       "bash", "-c", f"tr -d '\\r' < /ane/ros2/tools/{script} > /tmp/trial.sh && bash /tmp/trial.sh "
                       + " ".join([f"/ane/ros2/ane_sim/scenarios/{name}.txt", str(k), str(seed), f"{x:.6f}", f"{y:.6f}",
                                   f"{yaw:.6f}", container_out, last])]
            began = time.monotonic()
            try:
                result = sh(command, timeout=scenario.timeout * 3 + 180)
                status = "completed" if result.returncode == 0 else f"failed (exit {result.returncode})"
                output = result.stdout + result.stderr
            except subprocess.TimeoutExpired as error:
                sh(["docker", "rm", "-f", f"ane-trial-{name.lower()}-{k}"])
                status, output = "timed out (host)", (error.stdout or b"").decode(errors="replace") if isinstance(error.stdout, bytes) else str(error.stdout)
            elapsed = time.monotonic() - began
            bag_dir.mkdir(parents=True, exist_ok=True)
            (bag_dir / "container_output.txt").write_text(output, encoding="utf-8")
            metrics_file = out_dir / "metrics.json"
            metrics = json.loads(metrics_file.read_text()) if metrics_file.exists() else None
            attempt = {"scenario": name, "trial": k, "seed": seed, "status": status, "host_elapsed_s": round(elapsed, 1),
                       "outcome": metrics["metrics"]["outcome"] if metrics else None,
                       "accepted": metrics["accepted"] if metrics else None,
                       "consistent": metrics["consistent"] if metrics else None,
                       "finished_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
            index["attempts"].append(attempt)
            index_path.write_text(json.dumps(index, indent=1) + "\n", encoding="utf-8")
            print(f"{name} trial {k} seed {seed}: {status}; outcome {attempt['outcome']}; accepted {attempt['accepted']}; "
                  f"consistent {attempt['consistent']}; {elapsed:.0f} s", flush=True)

    campaign["clock_check_after"] = clock_check(args.image)
    campaign["finished_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    (out_root / "campaign.json").write_text(json.dumps(campaign, indent=1) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
