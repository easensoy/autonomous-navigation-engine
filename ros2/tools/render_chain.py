#!/usr/bin/env python3
"""Draws the executed software chain of one simulation trial (light and dark PNG).

  python ros2/tools/render_chain.py --out-dir ros2/evidence/media

Every box is a process or library that runs in a trial (ros2/README.md, "Execution chain"); every arrow
is a ROS topic, service or C++ call that the code makes, labelled with its name and rate. Needs
matplotlib.
"""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch  # noqa: E402

THEMES = {
    "light": {"bg": "#ffffff", "ink": "#1f2328", "muted": "#59636e", "edge": "#d0d7de",
              "engine": "#fff1e5", "engine_edge": "#d9822b", "adapter": "#e8f1fd", "adapter_edge": "#3b78d8",
              "sim": "#ecf6ee", "sim_edge": "#3f9a57", "harness": "#f4edfb", "harness_edge": "#8a5cc7",
              "arrow": "#57606a"},
    "dark": {"bg": "#0d1117", "ink": "#e6edf3", "muted": "#9198a1", "edge": "#3d444d",
             "engine": "#3a2614", "engine_edge": "#e3964a", "adapter": "#13233b", "adapter_edge": "#5b9cf0",
             "sim": "#13281a", "sim_edge": "#56b46f", "harness": "#2a1d3a", "harness_edge": "#a985e0",
             "arrow": "#9198a1"},
}

# (key, layer, x, y, width, height, title, lines)
BOXES = [
    ("runner", "harness", 0.2, 3.9, 2.7, 1.8, "scenario runner", ["requests the mission,", "spawns obstacles, sends",
                                                                   "closures and faults"]),
    ("nav", "adapter", 4.1, 3.0, 4.2, 2.7, "graph_navigator (adapter node)",
     ["pose → nearest roadmap node", "mission executive: legs, node steps", "segment controller: turn, pure pursuit",
      "lidar returns → blocked nodes", "watchdogs: feedback, progress, tracking"]),
    ("engine", "engine", 10.0, 3.3, 3.3, 2.4, "NavigationCore (this engine)",
     ["A* via DynamicReplanner", "states, route ownership", "BLOCKED when no route is left"]),
    ("gate", "adapter", 0.2, 0.35, 2.55, 1.45, "cmd_vel_gate", ["stop and stale-command", "guard, steady clock"]),
    ("fault", "harness", 3.55, 0.35, 2.7, 1.45, "fault_injector", ["test instrument, passes", "through unless asked"]),
    ("gazebo", "sim", 7.35, 0.35, 5.95, 1.45, "Gazebo Harmonic", ["DART physics · DiffDrive · gpu_lidar, 360 beams",
                                                                 "ground-truth pose · topics via ros_gz_bridge"]),
]

# (from, to, label, (start side), (end side), label placement); sides are fractions of the box size,
# chosen so that horizontal arrows are horizontal.
ARROWS = [
    ("runner", "nav", "start_mission,\nblockage_reports", (1.0, 0.639), (0.0, 0.759), "above"),
    ("nav", "engine", "startNavigation,\nhandleObstacles,\nexecuteNavigationStep", (1.0, 0.759), (0.0, 0.729), "above"),
    ("engine", "nav", "node route, state", (0.0, 0.2917), (1.0, 0.370), "below"),
    ("nav", "gate", "cmd_vel_nav · 20 Hz", (0.05, 0.0), (0.8, 1.0), "left"),
    ("gate", "fault", "cmd_vel", (1.0, 0.586), (0.0, 0.586), "above"),
    ("fault", "gazebo", "cmd_vel_robot", (1.0, 0.676), (0.0, 0.676), "above"),
    ("gazebo", "fault", "ground truth", (0.0, 0.262), (1.0, 0.262), "above"),
    ("fault", "nav", "odom", (0.55, 1.0), (0.2226, 0.0), "right"),
    ("gazebo", "nav", "scan · 10 Hz", (0.12, 1.0), (0.944, 0.0), "right"),
]


def anchor(box, side):
    _, _, x, y, w, h, _, _ = box
    return x + side[0] * w, y + side[1] * h


def render(theme_name, out):
    t = THEMES[theme_name]
    fig = plt.figure(figsize=(13.6, 6.6), dpi=170)
    fig.patch.set_facecolor(t["bg"])
    ax = fig.add_axes([0.01, 0.10, 0.98, 0.80])
    ax.set_xlim(0, 13.5)
    ax.set_ylim(0, 5.9)
    ax.axis("off")
    fig.text(0.015, 0.955, "Executed software chain of one simulation trial", fontsize=17, fontweight="bold",
             color=t["ink"], va="center")
    fig.text(0.015, 0.915, "One Docker container per trial: ROS 2 Jazzy, Gazebo Harmonic, this engine linked into the "
             "adapter. Simulation only; the pose is simulator ground truth.", fontsize=11.5, color=t["muted"], va="center")
    boxes = {b[0]: b for b in BOXES}
    for key, layer, x, y, w, h, title, lines in BOXES:
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.12", linewidth=1.8,
                                    facecolor=t[layer], edgecolor=t[layer + "_edge"]))
        ax.text(x + 0.16, y + h - 0.2, title, fontsize=13, fontweight="bold", color=t["ink"], va="top")
        for i, line in enumerate(lines):
            ax.text(x + 0.16, y + h - 0.62 - 0.37 * i, line, fontsize=11.2, color=t["ink"], va="top")
    for start, end, label, s_side, e_side, where in ARROWS:
        p0, p1 = anchor(boxes[start], s_side), anchor(boxes[end], e_side)
        ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="-|>", mutation_scale=16, linewidth=1.6, color=t["arrow"],
                                     shrinkA=1, shrinkB=1))
        mx, my = (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2
        place = {"above": (mx, my + 0.07, "center", "bottom"), "below": (mx, my - 0.07, "center", "top"),
                 "left": (mx - 0.12, my, "right", "center"), "right": (mx + 0.12, my, "left", "center")}[where]
        ax.text(place[0], place[1], label, fontsize=10.2, color=t["muted"], ha=place[2], va=place[3],
                linespacing=1.15)
    legend = [("engine", "this library"), ("adapter", "adapter (ros2/ane_navigation)"),
              ("sim", "simulator"), ("harness", "test harness (scripted events, faults)")]
    x = 0.015
    for layer, text in legend:
        fig.patches.append(FancyBboxPatch((x, 0.035), 0.018, 0.028, boxstyle="round,pad=0.002", transform=fig.transFigure,
                                          facecolor=t[layer], edgecolor=t[layer + "_edge"], linewidth=1.5))
        fig.text(x + 0.024, 0.049, text, fontsize=11, color=t["ink"], va="center")
        x += 0.05 + 0.0068 * len(text)
    fig.savefig(out, facecolor=t["bg"])
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out-dir", type=Path, default=Path(__file__).resolve().parents[1] / "evidence" / "media")
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    for theme in THEMES:
        path = args.out_dir / f"software-chain-{theme}.png"
        render(theme, path)
        print(path)


if __name__ == "__main__":
    main()
