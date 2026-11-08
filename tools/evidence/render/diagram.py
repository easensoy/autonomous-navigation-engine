"""Implementation and data-flow diagram for the README: the hero's executed path through the library
(top row) and the evidence pipeline that turns it into figures (bottom row).

The labels name real functions and files; the numbers come from docs/data/hero.json.
"""

from __future__ import annotations

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

from common import ASSETS, MONO, THEMES, apply_rc, load_json, save


def box(ax, t, x, y, w, h, title, body, *, accent=False, mono_title=True):
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle="round,pad=0.006,rounding_size=0.012",
                                facecolor=t.floor, edgecolor=t.route if accent else t.rule,
                                linewidth=1.6 if accent else 1.0, zorder=2))
    ax.text(x, y + h * 0.18, title, ha="center", va="center", fontsize=8.8, color=t.text, zorder=3,
            family=MONO if mono_title else None, fontweight="bold" if not mono_title else "normal")
    ax.text(x, y - h * 0.2, body, ha="center", va="center", fontsize=7.9, color=t.muted, zorder=3,
            linespacing=1.25)


def arrow(ax, t, x0, y0, x1, y1, label=None, *, label_dy=0.035):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle="-|>", mutation_scale=9, color=t.faint,
                                 linewidth=1.0, zorder=1, shrinkA=0, shrinkB=0))
    if label:
        ax.text((x0 + x1) / 2, (y0 + y1) / 2 + label_dy, label, ha="center", va="bottom", fontsize=7.4,
                color=t.faint, family=MONO)


def render(theme_name: str):
    t = THEMES[theme_name]
    apply_rc(t)
    hero = load_json("hero.json")
    nodes = hero["graph"]["node_count"]
    blocked = len(hero["blocked_nodes"])
    k = hero["block_step"]
    steps_after = len(hero["timeline"]) - 1 - k

    fig = plt.figure(figsize=(9.6, 3.3))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")

    def row(widths, left=0.015, right=0.985):
        gap = (right - left - sum(widths)) / (len(widths) - 1)
        centers, x = [], left
        for w in widths:
            centers.append(x + w / 2)
            x += w + gap
        return centers

    h = 0.22
    top = 0.68
    ax.text(0.015, 0.93, "Executed path for the hero, inside the library", fontsize=10.5, fontweight="bold",
            color=t.text, va="center")
    ws = [0.12, 0.115, 0.155, 0.145, 0.18, 0.125]
    xs = row(ws)
    box(ax, t, xs[0], top, ws[0], h, "hero_replan.txt", "scenario file")
    box(ax, t, xs[1], top, ws[1], h, "Graph", f"addNode, addEdge;\n{nodes} nodes", accent=True)
    box(ax, t, xs[2], top, ws[2], h, "startNavigation", "DynamicReplanner →\nGlobalPathPlanner → A*", accent=True)
    box(ax, t, xs[3], top, ws[3], h, f"{k} steps", "one node per\nexecuteNavigationStep", accent=True)
    box(ax, t, xs[4], top, ws[4], h, f"handleObstacles({blocked})",
        f"copy graph, removeNode × {blocked},\nnew AStar, findPath from here", accent=True)
    box(ax, t, xs[5], top, ws[5], h, f"{steps_after} steps", "to the station,\nevery step checked", accent=True)
    for a in range(5):
        arrow(ax, t, xs[a] + ws[a] / 2, top, xs[a + 1] - ws[a + 1] / 2, top)

    bottom = 0.2
    ax.text(0.015, 0.43, "Evidence pipeline (tools/evidence): calls the library and records what it does",
            fontsize=10.5, fontweight="bold", color=t.text, va="center")
    bw = [0.175, 0.17, 0.13, 0.15, 0.2]
    bx = row(bw)
    box(ax, t, bx[0], bottom, bw[0], h, "evidence_export", "trace mode: SearchObserver,\nchecks every route")
    box(ax, t, bx[1], bottom, bw[1], h, "docs/data", "JSON traces; CSV timings\nfrom evidence_bench")
    box(ax, t, bx[2], bottom, bw[2], h, "summarize.py", "percentiles,\nsummary.json")
    box(ax, t, bx[3], bottom, bw[3], h, "render/*.py", "matplotlib, Pillow;\nlight and dark")
    box(ax, t, bx[4], bottom, bw[4], h, "check_outputs.py", "routes, fingerprints and\nevery README number")
    for a in range(4):
        arrow(ax, t, bx[a] + bw[a] / 2, bottom, bx[a + 1] - bw[a + 1] / 2, bottom)
    path = ASSETS / f"how-it-fits-{theme_name}.png"
    save(fig, path)
    return path


def main(themes=("light", "dark")):
    return [render(theme) for theme in themes]


if __name__ == "__main__":
    for p in main():
        print(p)
