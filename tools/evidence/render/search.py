"""Figure A: Dijkstra, A* with h = 0, A* and BFS on the replanner's own query (docs/data/search.json).

Expanded nodes are coloured by the order the algorithm expanded them, taken from the ordered
events SearchObserver recorded while each search ran. Routes are the ones each planner returned.
"""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle

from common import ASSETS, THEMES, Scene, apply_rc, draw_blocked, draw_floor, load_json, save

ORDER = ("dijkstra", "astar_zero", "astar", "bfs")
NOTES = {
    "dijkstra": "settles every reachable node (no early exit)",
    "astar_zero": "same order as Dijkstra, stops at the goal",
    "astar": "default heuristic: straight-line distance × min cost/length",
    "bfs": "fewest edges; ignores edge costs",
}


def render(theme_name: str):
    t = THEMES[theme_name]
    apply_rc(t)
    data = load_json("search.json")
    scene = Scene(data)
    variants = {v["key"]: v for v in data["variants"]}
    start, goal = data["start"], data["goal"]
    ramp = LinearSegmentedColormap.from_list("order", [t.expand_lo, t.expand_hi])

    fig = plt.figure(figsize=(9.6, 6.5))
    fig.text(0.03, 0.966, "One query, four planners", fontsize=15, fontweight="bold", va="center")
    fig.text(0.03, 0.929, "The replanner's query at step 9 (aisle B blocked): navigator's node to the station. "
                          "Shading: expansion order.", fontsize=10, color=t.muted, va="center")
    boxes = [[0.01, 0.485, 0.485, 0.345], [0.505, 0.485, 0.485, 0.345],
             [0.01, 0.075, 0.485, 0.345], [0.505, 0.075, 0.485, 0.345]]
    for key, box in zip(ORDER, boxes):
        v = variants[key]
        ax = fig.add_axes(box)
        draw_floor(ax, scene, t, lattice_dots=False)
        ax.set_xlim(-0.8, 46.8)
        ax.set_ylim(-0.8, 23.8)
        order = [e[0] for e in v["expand_events"]]
        if order:
            xs = np.array([scene.xy[n][0] for n in order])
            ys = np.array([scene.xy[n][1] for n in order])
            c = np.linspace(0, 1, len(order))
            ax.scatter(xs, ys, c=c, cmap=ramp, marker="s", s=13, linewidths=0, zorder=3)
        draw_blocked(ax, scene, t, data["blocked_nodes"], crosses=True)
        rx, ry = scene.route_xy(v["route"]["nodes"])
        ax.plot(rx, ry, color=t.route, lw=2.6, solid_capstyle="round", solid_joinstyle="round", zorder=7)
        sx, sy = scene.xy[start]
        gx, gy = scene.xy[goal]
        ax.scatter([sx], [sy], s=55, color=t.marker, edgecolors=t.marker_ring, linewidths=1.4, zorder=9)
        ax.scatter([gx], [gy], marker="o", s=95, facecolors="none", edgecolors=t.marker, linewidths=1.8, zorder=9)
        ax.scatter([gx], [gy], marker="o", s=16, color=t.marker, zorder=9)
        r = v["route"]
        top = box[1] + box[3] + 0.006
        fig.text(box[0] + 0.03, top + 0.03, v["label"], fontsize=11.5, fontweight="bold", va="bottom")
        fig.text(box[0] + box[2] - 0.03, top + 0.03, f"{v['expansions']} expanded", fontsize=11.5,
                 fontweight="bold", va="bottom", ha="right", color=t.text)
        fig.text(box[0] + 0.03, top + 0.002, f"route: cost {r['cost']:.1f} · {r['length']:.1f} u · {r['hops']} edges",
                 fontsize=9.5, color=t.muted, va="bottom")

    # Shared key along the bottom.
    cax = fig.add_axes([0.045, 0.028, 0.11, 0.016])
    cax.imshow(np.linspace(0, 1, 256)[None, :], aspect="auto", cmap=ramp)
    cax.set_xticks([])
    cax.set_yticks([])
    for spine in cax.spines.values():
        spine.set_visible(False)
    fig.text(0.04, 0.036, "first", fontsize=8.5, color=t.muted, ha="right", va="center")
    fig.text(0.16, 0.036, "last expanded", fontsize=8.5, color=t.muted, ha="left", va="center")
    handles = [
        Line2D([], [], color=t.route, lw=2.6, label="returned route"),
        Line2D([], [], color=t.blocked, marker="x", lw=0, markersize=6.5, markeredgewidth=1.5, label="blocked"),
        Rectangle((0, 0), 1, 1, facecolor=t.zone, alpha=0.35, edgecolor=t.zone, hatch="////",
                  label="picking zone (cost ×3)"),
        Line2D([], [], color=t.marker, marker="o", lw=0, markersize=7, markeredgecolor=t.marker_ring,
               label="navigator"),
        Line2D([], [], color=t.marker, marker="o", lw=0, markersize=8, markerfacecolor="none",
               markeredgewidth=1.6, label="station"),
    ]
    fig.legend(handles=handles, loc="center left", bbox_to_anchor=(0.25, 0.036), ncol=5, frameon=False,
               fontsize=9, handlelength=1.8, columnspacing=1.3, handletextpad=0.5, labelcolor=t.text)
    path = ASSETS / f"search-comparison-{theme_name}.png"
    save(fig, path)
    return path


def main(themes=("light", "dark")):
    return [render(theme) for theme in themes]


if __name__ == "__main__":
    for p in main():
        print(p)
