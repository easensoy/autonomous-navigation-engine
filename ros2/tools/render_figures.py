#!/usr/bin/env python3
"""Plots a campaign's recorded trials: one top-down panel per scenario and a combined grid, in light
and dark variants.

  python ros2/tools/render_figures.py --campaign-dir ros2/evidence/trials/<NAME> --out <dir>

Every line comes from recorded data: ground-truth trajectories (trajectory.json, from the simulator's
/ground_truth/odom), the engine's routes and the blockages (navigator events). The map, roadmap and
cost zone are drawn from the roadmap file the navigator loaded. Needs matplotlib and numpy.
"""

import argparse
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

PKG = Path(__file__).resolve().parents[1] / "ane_sim"
sys.path.insert(0, str(PKG / "scripts"))
from scenario_lib import load_roadmap, load_scenario  # noqa: E402
ORDER = ["A_nominal", "B_aisle_blockage", "C_disconnection", "D_feedback_dropout", "E_feedback_loss", "F_drive_stall"]
THEMES = {
    "light": {"bg": "#ffffff", "fg": "#1f2328", "muted": "#59636e", "obstacle": "#c9ced6", "wall": "#8c959f",
              "edge": "#d0d7de", "node": "#afb8c1", "route": "#1a7f37", "trajectory": "#0969da", "sensor": "#cf222e",
              "operator": "#8250df", "pallet": "#bc4c00", "fault": "#bf8700", "zone": "#fb8f44"},
    "dark": {"bg": "#0d1117", "fg": "#e6edf3", "muted": "#9198a1", "obstacle": "#30363d", "wall": "#6e7681",
             "edge": "#30363d", "node": "#484f58", "route": "#3fb950", "trajectory": "#58a6ff", "sensor": "#ff7b72",
             "operator": "#d2a8ff", "pallet": "#f0883e", "fault": "#e3b341", "zone": "#f0883e"},
}
TERMINAL_MARK = {"COMPLETED": "o", "BLOCKED": "s", "FAILED_LOCALIZATION": "D", "FAILED_CONTROLLER": "D", "STOPPED": "s"}


def load_trials(campaign_dir, name):
    trials = []
    for folder in sorted((campaign_dir / name).glob("trial_*")):
        traj, metrics = folder / "trajectory.json", folder / "metrics.json"
        if traj.exists() and metrics.exists():
            trials.append((json.loads(traj.read_text()), json.loads(metrics.read_text())))
    return trials


def draw(ax, roadmap, name, trials, theme, compact=False):
    c = THEMES[theme]
    ax.set_facecolor(c["bg"])
    nodes = roadmap["nodes"]
    hall = (-8.0, -5.0, 8.0, 5.0)
    for o in roadmap["obstacles"]:
        x0, y0, x1, y1 = o["rect"]
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color=c["wall"] if o["kind"] == "wall" else c["obstacle"], lw=0, zorder=1))
    for z in roadmap["zones"]:
        x0, y0, x1, y1 = z["rect"]
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color=c["zone"], alpha=0.18, lw=0, zorder=1))
    for a, b, _ in roadmap["edges"]:
        ax.plot([nodes[a][0], nodes[b][0]], [nodes[a][1], nodes[b][1]], color=c["edge"], lw=1.0, zorder=2)
    for station, node in roadmap["stations"].items():
        x, y = nodes[node]
        ax.annotate(station, (x, y), xytext=(0, 9 if station == "dispatch" else -14), textcoords="offset points",
                    ha="center", fontsize=7 if compact else 8, color=c["muted"], zorder=6)
    if trials:
        first = trials[0][0]
        for o in first["obstacles"]:
            x0, y0, x1, y1 = o["rect"]
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color=c["pallet"], alpha=0.85, lw=0, zorder=3))
        planned = [r for r in first["routes"] if r["type"] == "ROUTE_PLANNED"]
        replaced = [r for r in first["routes"] if r["type"] == "ROUTE_REPLACED"]
        for r in planned:
            pts = [nodes[n] for n in r["nodes"]]
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=c["route"], lw=2.0, ls=(0, (4, 3)), zorder=4,
                    label="engine route at start")
        for r in replaced:
            pts = [nodes[n] for n in r["nodes"]]
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=c["route"], lw=2.4, zorder=4, label="engine replacement route")
        sensor = {n for b in first["blockages"] if b["source"] == "SENSOR" for n in b["nodes"]}
        operator = {n for b in first["blockages"] if b["source"] != "SENSOR" for n in b["nodes"]}
        if operator:
            ax.scatter([nodes[n][0] for n in operator], [nodes[n][1] for n in operator], marker="x", s=22, color=c["operator"],
                       lw=1.4, zorder=5, label="closed by operator")
        if sensor:
            ax.scatter([nodes[n][0] for n in sensor], [nodes[n][1] for n in sensor], marker="x", s=30, color=c["sensor"],
                       lw=1.8, zorder=5, label="blocked from lidar")
    for k, (traj, metrics) in enumerate(trials):
        gt = traj["ground_truth"]
        xs, ys = [p[1] for p in gt], [p[2] for p in gt]
        ax.plot(xs, ys, color=c["trajectory"], lw=1.3 if k == 0 else 0.9, alpha=0.95 if k == 0 else 0.55, zorder=6,
                label="robot trajectory (ground truth)" if k == 0 else None)
        start = next((f["t"] for f in traj["faults"] if f["type"] == "FAULT_STARTED"), None)
        end = next((f["t"] for f in traj["faults"] if f["type"] == "FAULT_ENDED"), traj.get("terminal"))
        if start is not None:
            seg = [p for p in gt if start <= p[0] <= (end or start)]
            ax.plot([p[1] for p in seg], [p[2] for p in seg], color=c["fault"], lw=3.0, alpha=0.9, zorder=7,
                    solid_capstyle="round", label="during the injected fault" if k == 0 else None)
        outcome = metrics["metrics"]["outcome"]
        ax.scatter([xs[-1]], [ys[-1]], marker=TERMINAL_MARK.get(outcome, "o"), s=16, color=c["fg"], zorder=8)
    ax.set_xlim(hall[0] - 0.35, hall[2] + 0.35)
    ax.set_ylim(hall[1] - 0.35, hall[3] + 0.35)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_visible(False)
    scenario = load_scenario(PKG / "scenarios" / f"{name}.txt")
    outcomes = {}
    for _, m in trials:
        outcomes[m["metrics"]["outcome"]] = outcomes.get(m["metrics"]["outcome"], 0) + 1
    accepted = sum(1 for _, m in trials if m["accepted"])
    summary = ", ".join(f"{v}× {k}" for k, v in outcomes.items())
    ax.set_title(f"{name[0]}  {scenario.title.split(':')[0]}\n{summary}; {accepted}/{len(trials)} met their criteria",
                 fontsize=8.5 if compact else 10, color=c["fg"], loc="left")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--campaign-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    roadmap = load_roadmap(PKG / "maps" / "warehouse_roadmap.txt")
    args.out.mkdir(parents=True, exist_ok=True)
    data = {name: load_trials(args.campaign_dir, name) for name in ORDER}
    for theme in THEMES:
        c = THEMES[theme]
        fig, axes = plt.subplots(3, 2, figsize=(10.0, 10.6), facecolor=c["bg"])
        for ax, name in zip(axes.flat, ORDER):
            draw(ax, roadmap, name, data[name], theme, compact=True)
        handles, labels = {}, {}
        for ax in axes.flat:
            for h, label in zip(*ax.get_legend_handles_labels()):
                handles.setdefault(label, h)
        legend = fig.legend(handles.values(), handles.keys(), loc="lower center", ncol=4, fontsize=8, frameon=False,
                            labelcolor=c["fg"])
        fig.text(0.5, 0.012, "Gazebo Harmonic simulation, 5 trials per scenario; trajectories are the simulator's ground truth, "
                 "routes and blockages the navigator's recorded events. Not a physical experiment.",
                 ha="center", fontsize=7.5, color=c["muted"])
        fig.subplots_adjust(left=0.01, right=0.99, top=0.96, bottom=0.075, wspace=0.03, hspace=0.18)
        legend.set_bbox_to_anchor((0.5, 0.025))
        fig.savefig(args.out / f"simulation-trials-{theme}.png", dpi=110, facecolor=c["bg"])
        plt.close(fig)
        for name in ORDER:
            fig, ax = plt.subplots(figsize=(8.0, 5.4), facecolor=c["bg"])
            draw(ax, roadmap, name, data[name], theme)
            ax.legend(loc="lower center", bbox_to_anchor=(0.5, -0.13), ncol=3, fontsize=8, frameon=False, labelcolor=c["fg"])
            fig.subplots_adjust(left=0.01, right=0.99, top=0.9, bottom=0.12)
            fig.savefig(args.out / f"{name}-{theme}.png", dpi=110, facecolor=c["bg"])
            plt.close(fig)
    print(f"wrote figures to {args.out}")


if __name__ == "__main__":
    main()
