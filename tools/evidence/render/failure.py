"""Figure C: what the navigator does when a blockage leaves no route (docs/data/failure_*.json).

The text under each map is the engine's own output for that state: the console line
NavigationCore::handleObstacles printed, getNavigationStatus(), executeNavigationStep(), what
getCurrentPath() and getInvalidatedRoute() return, and the result of retryNavigation().
"""

from __future__ import annotations

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from common import ASSETS, MONO, THEMES, Scene, apply_rc, draw_blocked, draw_floor, load_json, save

CASES = (
    ("failure_goal_blocked", "Goal reported blocked"),
    ("failure_disconnected", "Goal cut off: both ends of its aisle blocked"),
)


def short_status(status: str) -> str:
    # "IDLE | mode GLOBAL_ONLY | at node 527 (10/46) | remaining 37.6569" -> drop the mode field
    parts = [p.strip() for p in status.split("|")]
    return " | ".join(p for p in parts if not p.startswith("mode"))


def render(theme_name: str):
    t = THEMES[theme_name]
    apply_rc(t)
    fig = plt.figure(figsize=(9.6, 4.9))
    fig.text(0.03, 0.958, "When no route is left", fontsize=15, fontweight="bold", va="center")
    fig.text(0.03, 0.912, "Same trip as the hero; at step 9 a different blockage is reported. The navigator's "
                          "state afterwards, as the engine reports it.", fontsize=10, color=t.muted, va="center")
    for col, (name, title) in enumerate(CASES):
        d = load_json(f"{name}.json")
        scene = Scene(d)
        x0 = 0.015 + col * 0.495
        ax = fig.add_axes([x0, 0.36, 0.47, 0.47])
        draw_floor(ax, scene, t, lattice_dots=False)
        ax.set_xlim(-0.8, 46.8)
        ax.set_ylim(-0.8, 23.8)
        if d.get("unreachable_nodes"):
            xs = [scene.xy[n][0] for n in d["unreachable_nodes"]]
            ys = [scene.xy[n][1] for n in d["unreachable_nodes"]]
            ax.scatter(xs, ys, marker="s", s=16, color=t.expand_hi, linewidths=0, zorder=3)
        draw_blocked(ax, scene, t, d["blocked_nodes"])
        travelled = d["current_path_after"]
        invalidated = d["invalidated_route"]
        tx, ty = scene.route_xy(travelled)
        ax.plot(tx, ty, color=t.route, lw=3.0, solid_capstyle="round", zorder=6)
        sx, sy = scene.route_xy(invalidated)
        ax.plot(sx, sy, color=t.previous, lw=1.8, ls=(0, (4, 3)), zorder=5)
        px, py = scene.xy[d["position"]]
        ax.scatter([px], [py], s=70, color=t.marker, edgecolors=t.marker_ring, linewidths=1.6, zorder=9)
        ax.annotate(d["state_after"], (px, py), xytext=(0, -11), textcoords="offset points", ha="center",
                    va="top", fontsize=9, color=t.text, fontweight="bold", zorder=10)
        gx, gy = scene.xy[d["goal"]]
        ax.scatter([gx], [gy], marker="o", s=110, facecolors="none", edgecolors=t.marker, linewidths=1.8, zorder=9)
        ax.scatter([gx], [gy], marker="o", s=18, color=t.marker, zorder=9)
        ax.annotate("goal", (gx, gy), xytext=(0, 9), textcoords="offset points", ha="center", va="bottom",
                    fontsize=9, color=t.text, zorder=10)
        fig.text(x0 + 0.01, 0.855, title, fontsize=11.5, fontweight="bold", va="bottom")

        lines = [
            ("status", short_status(d["status_after"])),
            ("step()", f"{'true' if d['execute_step_after'] else 'false'}: stays at node {d['position']}"),
            ("path", f"getCurrentPath(): the {len(travelled)} travelled nodes only"),
            ("ahead", f"getInvalidatedRoute(): {len(invalidated)} nodes, "
                      f"{d['invalidated_route_blocked_nodes']} blocked"),
            ("retry", f"retryNavigation(): {'true' if d['retry_after'] else 'false'}, "
                      f"state {d['state_after_retry']}"),
        ]
        if d["goal_blocked"]:
            lines.append(("why", "blocked goal: replanner returns [] without a search"))
        elif d["astar_expansions"] == d["reachable_component_nodes"]:
            lines.append(("why", f"no route: all {d['astar_expansions']} reachable nodes expanded"))
        else:
            lines.append(("why", f"no route: A* gives up after {d['astar_expansions']} expansions"))
        y = 0.255
        for label, text in lines:
            fig.text(x0 + 0.01, y, label, fontsize=7.3, color=t.muted, va="top", family=MONO)
            fig.text(x0 + 0.06, y, text, fontsize=7.3, color=t.text, va="top", family=MONO)
            y -= 0.036
        console = d["console_message"]
    fig.text(0.025, 0.305, "handleObstacles() printed, in both cases:", fontsize=8.5, color=t.muted, va="top")
    fig.text(0.285, 0.305, f"\"{console}\"", fontsize=7.8, color=t.text, va="top", family=MONO)

    handles = [
        Line2D([], [], color=t.route, lw=3.0, label="travelled: getCurrentPath()"),
        Line2D([], [], color=t.previous, lw=1.8, ls=(0, (4, 3)), label="invalidated: getInvalidatedRoute()"),
        Line2D([], [], color=t.blocked, marker="x", lw=0, markersize=6.5, markeredgewidth=1.5,
               label="reported blocked"),
        Line2D([], [], color=t.expand_hi, marker="s", lw=0, markersize=5.5, label="cut off from the navigator"),
    ]
    fig.legend(handles=handles, loc="center", bbox_to_anchor=(0.5, 0.04), ncol=4, frameon=False, fontsize=8.8,
               handlelength=1.8, columnspacing=1.4, handletextpad=0.5, labelcolor=t.text)
    path = ASSETS / f"failure-panel-{theme_name}.png"
    save(fig, path)
    return path


def main(themes=("light", "dark")):
    return [render(theme) for theme in themes]


if __name__ == "__main__":
    for p in main():
        print(p)
