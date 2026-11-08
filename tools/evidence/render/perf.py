"""Measured performance figures, from docs/data/summary.json (computed from the raw CSVs).

  replan-cost-*.png      where a replan's time goes, per warehouse size
  perf-latency-*.png     query latency p50/p95 by graph size, and route cost against the optimum
"""

from __future__ import annotations

import csv
import math

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

from common import ASSETS, DATA, SERIES_LABEL, THEMES, apply_rc, load_json, save

WORKLOADS = ("warehouse_s", "warehouse_m", "warehouse_l")
VARIANTS = ("dijkstra", "astar_zero", "bfs", "astar")


def ink_for(background_hex: str) -> str:
    """Dark or white label text, whichever contrasts with the bar (WCAG relative luminance)."""
    channels = [int(background_hex.lstrip("#")[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    linear = [c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in channels]
    luminance = 0.2126 * linear[0] + 0.7152 * linear[1] + 0.0722 * linear[2]
    return "#1f2328" if luminance > 0.18 else "#ffffff"


def _style_axes(ax, t):
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(t.rule)
    ax.tick_params(colors=t.muted, labelsize=9)
    ax.grid(True, color=t.rule, linewidth=0.6, alpha=0.6)
    ax.set_axisbelow(True)


def render_replan(theme_name: str):
    t = THEMES[theme_name]
    apply_rc(t)
    s = load_json("summary.json")
    rows = [("README map", s["hero"]["graph"]["nodes"], s["hero"]["replan_share_median"],
             s["hero"]["timing_ms"]["replan"]["p50"], s["hero"]["timing_ms"]["search_cached"]["p50"],
             s["hero"]["timing_runs"], "runs")]
    for name, label in zip(WORKLOADS, ("small", "medium", "large")):
        r = s["replans"][name]
        rows.append((f"{label} warehouse", s["queries"][name]["nodes"], r["share_of_end_to_end_median"],
                     r["phases_ms"]["end_to_end"]["p50"], r["phases_ms"]["search_cached"]["p50"], r["events"],
                     "events"))
    parts = [("search", "A* search", t.route), ("scale_pass", "heuristic scale pass on the copy", t.route_light),
             ("copy", "copy the graph", t.expand_hi), ("remove", "remove blocked nodes", t.rack)]

    fig = plt.figure(figsize=(9.6, 4.0))
    fig.text(0.03, 0.935, "Where a replan's time goes", fontsize=15, fontweight="bold", va="center")
    fig.text(0.03, 0.855, "One replan (NavigationCore::handleObstacles), split into the operations it performs, "
                          "each timed on its own.", fontsize=10, color=t.muted, va="center")
    ax = fig.add_axes([0.235, 0.2, 0.5, 0.56])
    y = np.arange(len(rows))[::-1]
    for yi, (label, nodes, shares, total, search, n, unit) in zip(y, rows):
        left = 0.0
        for key, _, color in parts:
            w = shares[key]
            ax.barh(yi, w, left=left, height=0.62, color=color, edgecolor=t.background, linewidth=0.8)
            if w >= 0.09:
                ax.text(left + w / 2, yi, f"{w:.0%}", ha="center", va="center", fontsize=8.5, color=ink_for(color))
            left += w
        other = max(0.0, 1.0 - left)
        ax.barh(yi, other, left=left, height=0.62, color="none", edgecolor=t.rule, linewidth=0.8)
        ax.text(-0.02, yi + 0.12, label, ha="right", va="center", fontsize=10, color=t.text)
        ax.text(-0.02, yi - 0.2, f"{nodes:,} nodes", ha="right", va="center", fontsize=8.5, color=t.muted)
        ax.text(1.02, yi + 0.12, f"{total:.2f} ms" if total < 10 else f"{total:.1f} ms", ha="left", va="center",
                fontsize=10, color=t.text, fontweight="bold")
        ax.text(1.02, yi - 0.2, f"search alone {search:.2f} ms · {n:,} {unit}", ha="left", va="center",
                fontsize=8.5, color=t.muted)
    ax.set_xlim(0, 1)
    ax.set_ylim(-0.6, len(rows) - 0.4)
    ax.set_yticks([])
    ax.set_xticks([0, 0.25, 0.5, 0.75, 1.0])
    ax.set_xticklabels(["0", "25%", "50%", "75%", "100%"], fontsize=9, color=t.muted)
    ax.set_xlabel("median share of one replan's time", fontsize=9, color=t.muted, labelpad=3)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color(t.rule)
    ax.tick_params(axis="x", colors=t.muted)
    fig.text(0.745, 0.775, "replan, p50", fontsize=8.5, color=t.muted, va="bottom")
    handles = [Patch(facecolor=c, edgecolor="none", label=lbl) for _, lbl, c in parts]
    handles.append(Patch(facecolor="none", edgecolor=t.rule, label="other (navigator bookkeeping, cache effects)"))
    fig.legend(handles=handles, loc="center", bbox_to_anchor=(0.5, 0.045), ncol=5, frameon=False, fontsize=8.6,
               handlelength=1.3, columnspacing=1.0, handletextpad=0.5, labelcolor=t.text)
    path = ASSETS / f"replan-cost-{theme_name}.png"
    save(fig, path)
    return path


def _ratios(name, variant):
    out = []
    with open(DATA / "bench" / f"{name}.query_results.csv", newline="") as f:
        for row in csv.DictReader(f):
            if row["variant"] == variant and row["cost_ratio_to_dijkstra"]:
                out.append(float(row["cost_ratio_to_dijkstra"]))
    return np.sort(np.array(out))


def render_latency(theme_name: str):
    t = THEMES[theme_name]
    apply_rc(t)
    s = load_json("summary.json")
    q = s["queries"]
    nodes = [q[w]["nodes"] for w in WORKLOADS]

    fig = plt.figure(figsize=(9.6, 4.75))
    fig.text(0.03, 0.948, "Query latency and route cost on saved warehouse workloads", fontsize=15,
             fontweight="bold", va="center")
    reps, warm = q["warehouse_s"]["repeats"], q["warehouse_s"]["warmup"]
    fig.text(0.03, 0.885, f"{q['warehouse_s']['queries']:,} / {q['warehouse_m']['queries']:,} / "
                          f"{q['warehouse_l']['queries']:,} random start-goal pairs (fixed seeds). Per query: median "
                          f"of {reps} timed run{'s' if reps != 1 else ''}"
                          f"{f' after {warm} warm-up' if warm else ', no warm-up'}.",
             fontsize=10, color=t.muted, va="center")

    # Left: p50 and p95 latency against graph size.
    ax = fig.add_axes([0.075, 0.14, 0.4, 0.6])
    _style_axes(ax, t)
    fig.text(0.075, 0.795, "Latency per query", fontsize=11, fontweight="bold", va="bottom")
    offsets = {"dijkstra": -0.045, "astar_zero": -0.015, "bfs": 0.015, "astar": 0.045}
    label_y = {}
    for v in VARIANTS:
        color = t.series[v]
        p50 = [q[w]["variants"][v]["latency_ms"]["p50"] for w in WORKLOADS]
        p95 = [q[w]["variants"][v]["latency_ms"]["p95"] for w in WORKLOADS]
        xs = [n * 10 ** offsets[v] for n in nodes]
        ax.plot(xs, p50, color=color, lw=2.0, marker="o", ms=5.5, zorder=3)
        for x, lo, hi in zip(xs, p50, p95):
            ax.plot([x, x], [lo, hi], color=color, lw=1.4, alpha=0.8, zorder=2)
            ax.plot([x], [hi], marker="_", ms=8, mew=1.6, color=color, zorder=2)
        label_y[v] = math.log10(p50[-1])
    # Direct labels at the right end, nudged apart so none overlap (in decades of the log axis).
    placed = sorted(label_y.items(), key=lambda kv: kv[1])
    for k in range(1, len(placed)):
        if placed[k][1] - placed[k - 1][1] < 0.13:
            placed[k] = (placed[k][0], placed[k - 1][1] + 0.13)
    for v, ly in placed:
        ax.text(nodes[-1] * 1.35, 10 ** ly, SERIES_LABEL[v], color=t.series[v], fontsize=9, va="center",
                ha="left")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlim(nodes[0] / 1.6, nodes[-1] * 4.2)
    ax.set_xticks(nodes)
    ax.set_xticklabels([f"{n:,}" for n in nodes])
    ax.minorticks_off()
    ax.set_xlabel("graph size (nodes)", fontsize=9.5)
    ax.set_ylabel("latency per query (ms, log scale)", fontsize=9.5)
    ax.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f"{v:g}"))
    fig.text(0.475, 0.795, "dot = p50, tick = p95", fontsize=8.5, color=t.muted, va="bottom", ha="right")

    # Right: route cost against Dijkstra's optimum, per query.
    ax2 = fig.add_axes([0.62, 0.14, 0.355, 0.6])
    _style_axes(ax2, t)
    fig.text(0.62, 0.795, "Route cost against the optimum", fontsize=11, fontweight="bold", va="bottom")
    shades = {"warehouse_s": 0.45, "warehouse_m": 0.72, "warehouse_l": 1.0}
    labels = {"warehouse_s": "small", "warehouse_m": "medium", "warehouse_l": "large"}
    for w in WORKLOADS:
        r = _ratios(w, "bfs")
        ecdf = np.arange(1, len(r) + 1) / len(r)
        ax2.step(np.concatenate([[1.0], r]), np.concatenate([[0.0], ecdf]), where="post", color=t.series["bfs"],
                 alpha=shades[w], lw=1.8, label=f"BFS, {labels[w]}")
        ra = _ratios(w, "astar")
        ea = np.arange(1, len(ra) + 1) / len(ra)
        ax2.step(np.concatenate([[1.0], ra]), np.concatenate([[0.0], ea]), where="post", color=t.series["astar"],
                 alpha=shades[w], lw=1.8, label=f"A*, {labels[w]}")
    ax2.set_xlim(0.98, 2.4)
    ax2.set_ylim(0, 1.02)
    ax2.set_xlabel("route cost ÷ Dijkstra's optimal cost", fontsize=9.5)
    ax2.set_ylabel("share of queries at or below", fontsize=9.5)
    ax2.yaxis.set_major_formatter(plt.FuncFormatter(lambda v, _: f"{v:.0%}"))
    opt = [q[w]["variants"]["astar"]["optimal"] / q[w]["queries"] for w in WORKLOADS]
    excess = [q[w]["variants"]["astar"]["miss_excess_cost"] for w in WORKLOADS]
    if any(excess):
        lo = min(e["min"] for e in excess if e)
        hi = max(e["max"] for e in excess if e)
        note = (f"A* (blue line at 1.0): optimal on\n{min(opt):.1%}–{max(opt):.1%} of queries; the rest\n"
                f"cost {lo:.2f}–{hi:.2f} more")
    else:
        total = sum(q[w]["queries"] for w in WORKLOADS)
        note = f"A* (blue line at 1.0): Dijkstra's\noptimal cost on all {total:,} queries"
    ax2.text(1.45, 0.30, note, fontsize=8.5, color=t.text, va="center")
    bfs_med = [q[w]["variants"]["bfs"]["cost_ratio"]["p50"] for w in WORKLOADS]
    bfs_p95 = [q[w]["variants"]["bfs"]["cost_ratio"]["p95"] for w in WORKLOADS]
    ax2.text(1.62, 0.55, f"BFS (fewest edges):\nmedian ×{min(bfs_med):.2f}–×{max(bfs_med):.2f},\n"
                         f"p95 ×{min(bfs_p95):.2f}–×{max(bfs_p95):.2f}", fontsize=8.5, color=t.text, va="center")
    handles = [Line2D([], [], color=t.series["astar"], lw=1.8, label="A*"),
               Line2D([], [], color=t.series["bfs"], lw=1.8, label="BFS"),
               Line2D([], [], color=t.muted, lw=1.8, alpha=0.45, label="small"),
               Line2D([], [], color=t.muted, lw=1.8, alpha=1.0, label="large")]
    ax2.legend(handles=handles, loc="lower right", frameon=False, fontsize=8.5, ncol=2, labelcolor=t.text,
               handlelength=1.6, columnspacing=1.0)
    path = ASSETS / f"perf-latency-{theme_name}.png"
    save(fig, path)
    return path


def main(themes=("light", "dark")):
    out = []
    for theme in themes:
        out.append(render_replan(theme))
        out.append(render_latency(theme))
    return out


if __name__ == "__main__":
    for p in main():
        print(p)
