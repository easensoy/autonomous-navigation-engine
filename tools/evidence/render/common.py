"""Shared drawing code for the README figures.

Every figure is drawn from JSON/CSV files written by the C++ tools (evidence_export,
evidence_bench). Nothing here plans a route or orders a search: routes, costs and expansion
order are read from those files.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
DATA = REPO / "docs" / "data"
ASSETS = REPO / "docs" / "assets"

FONT = "DejaVu Sans"  # ships with matplotlib, so renders match across machines
MONO = "DejaVu Sans Mono"


@dataclass(frozen=True)
class Theme:
    name: str
    background: str
    text: str
    muted: str
    faint: str
    rule: str
    floor: str
    rack: str
    rack_edge: str
    lattice: str
    route: str          # the engine's current route / traversed route
    route_light: str    # planned remainder
    previous: str       # abandoned plan (dashed)
    blocked: str        # nodes reported blocked
    zone: str           # picking zone tint and hatch
    marker: str         # navigator position, start, goal
    marker_ring: str
    expand_lo: str      # expansion order ramp: early
    expand_hi: str      # expansion order ramp: late
    series: dict


# Okabe-Ito hues (colour-blind safe); each keeps one meaning across every figure.
LIGHT = Theme(
    name="light",
    background="#ffffff",
    text="#1f2328",
    muted="#59636e",
    faint="#818b98",
    rule="#d1d9e0",
    floor="#f6f8fa",
    rack="#8c959f",
    rack_edge="#6e7781",
    lattice="#c8d1da",
    route="#0b5cad",
    route_light="#56b4e9",
    previous="#6e7781",
    blocked="#d55e00",
    zone="#e69f00",
    marker="#1f2328",
    marker_ring="#ffffff",
    expand_lo="#e7ebef",
    expand_hi="#57606a",
    series={"astar": "#0b5cad", "astar_zero": "#56b4e9", "dijkstra": "#009e73", "bfs": "#cc79a7"},
)

DARK = Theme(
    name="dark",
    background="#0d1117",
    text="#f0f6fc",
    muted="#9198a1",
    faint="#6e7681",
    rule="#3d444d",
    floor="#151b23",
    rack="#3d444d",
    rack_edge="#59636e",
    lattice="#2a313c",
    route="#4ea8ff",
    route_light="#9fd3f5",
    previous="#9198a1",
    blocked="#ff7b3a",
    zone="#f0b429",
    marker="#f0f6fc",
    marker_ring="#0d1117",
    expand_lo="#262c36",
    expand_hi="#b7bdc8",
    series={"astar": "#4ea8ff", "astar_zero": "#9fd3f5", "dijkstra": "#3fcf9f", "bfs": "#e58fc4"},
)

THEMES = {"light": LIGHT, "dark": DARK}

SERIES_LABEL = {
    "astar": "A* (default heuristic)",
    "astar_zero": "A* with h = 0",
    "dijkstra": "Dijkstra",
    "bfs": "BFS",
}


def apply_rc(theme: Theme) -> None:
    plt.rcParams.update({
        "font.family": FONT,
        "font.size": 11,
        "text.color": theme.text,
        "axes.edgecolor": theme.rule,
        "axes.labelcolor": theme.muted,
        "axes.titlecolor": theme.text,
        "xtick.color": theme.muted,
        "ytick.color": theme.muted,
        "figure.facecolor": theme.background,
        "axes.facecolor": theme.background,
        "savefig.facecolor": theme.background,
        "hatch.linewidth": 0.8,
        "svg.fonttype": "path",
    })


def load_json(name: str) -> dict:
    with open(DATA / name, encoding="utf-8") as f:
        return json.load(f)


class Scene:
    """Geometry of an exported scenario: nodes, edges, racks, zones and blocked areas."""

    def __init__(self, record: dict):
        self.record = record
        sc = record["scenario"]
        self.lattice = sc["lattice"]
        self.shelves = sc["shelves"]
        self.zones = sc["zones"]
        self.blocks = sc["blocks"]
        self.xy = {n[0]: (n[1], n[2]) for n in record["graph"]["nodes"]}
        self.edges = {(e[0], e[1]): e[2] for e in record["graph"]["edges"]}

    def weight(self, a: int, b: int) -> float:
        return self.edges[(a, b)] if (a, b) in self.edges else self.edges[(b, a)]

    def has_edge(self, a: int, b: int) -> bool:
        return (a, b) in self.edges or (b, a) in self.edges

    def route_xy(self, route):
        return [self.xy[n][0] for n in route], [self.xy[n][1] for n in route]

    def route_cost(self, route) -> float:
        return sum(self.weight(a, b) for a, b in zip(route, route[1:]))

    def route_length(self, route) -> float:
        return sum(math.dist(self.xy[a], self.xy[b]) for a, b in zip(route, route[1:]))


def draw_floor(ax, scene: Scene, theme: Theme, *, lattice_dots=True, zone_label=True, rack_labels=False,
               aisle_labels=True, dot_size=1.6):
    x0, y0, x1, y1 = scene.lattice
    ax.add_patch(Rectangle((x0 - 0.5, y0 - 0.5), x1 - x0 + 1, y1 - y0 + 1, facecolor=theme.floor,
                           edgecolor=theme.rule, linewidth=1.0, zorder=0))
    for zx0, zy0, zx1, zy1, mult, label in scene.zones:
        ax.add_patch(Rectangle((zx0, zy0), zx1 - zx0, zy1 - zy0, facecolor=theme.zone, alpha=0.16,
                               edgecolor="none", zorder=1))
        ax.add_patch(Rectangle((zx0, zy0), zx1 - zx0, zy1 - zy0, facecolor="none", edgecolor=theme.zone,
                               hatch="////", linewidth=0.0, alpha=0.55, zorder=1))
        ax.add_patch(Rectangle((zx0, zy0), zx1 - zx0, zy1 - zy0, facecolor="none", edgecolor=theme.zone,
                               linewidth=1.0, alpha=0.9, zorder=1))
    for sx0, sy0, sx1, sy1, _ in scene.shelves:
        ax.add_patch(Rectangle((sx0, sy0), sx1 - sx0, sy1 - sy0, facecolor=theme.rack, edgecolor=theme.rack_edge,
                               linewidth=0.8, zorder=2))
    if lattice_dots:
        xs = [p[0] for p in scene.xy.values()]
        ys = [p[1] for p in scene.xy.values()]
        ax.scatter(xs, ys, s=dot_size, color=theme.lattice, linewidths=0, zorder=1.5)
    ax.set_xlim(x0 - 0.9, x1 + 0.9)
    ax.set_ylim(y0 - 0.9, y1 + 0.9)
    ax.set_aspect("equal")
    ax.axis("off")


def draw_blocked(ax, scene: Scene, theme: Theme, nodes, *, alpha=1.0, crosses=True, zorder=6):
    for bx0, by0, bx1, by1, _ in scene.blocks:
        ax.add_patch(Rectangle((bx0, by0), bx1 - bx0, by1 - by0, facecolor=theme.blocked, alpha=0.20 * alpha,
                               edgecolor=theme.blocked, linewidth=1.4, zorder=zorder))
    if crosses and nodes:
        xs = [scene.xy[n][0] for n in nodes]
        ys = [scene.xy[n][1] for n in nodes]
        ax.scatter(xs, ys, marker="x", s=22, color=theme.blocked, linewidths=1.4, alpha=alpha, zorder=zorder + 0.1)


def draw_endpoints(ax, scene: Scene, theme: Theme, start, goal, *, start_label=None, goal_label=None,
                   fontsize=10, start_kind="square"):
    sx, sy = scene.xy[start]
    gx, gy = scene.xy[goal]
    if start_kind == "square":
        ax.scatter([sx], [sy], marker="s", s=70, color=theme.marker, edgecolors=theme.marker_ring, linewidths=1.2,
                   zorder=9)
    ax.scatter([gx], [gy], marker="o", s=150, facecolors="none", edgecolors=theme.marker, linewidths=2.0, zorder=9)
    ax.scatter([gx], [gy], marker="o", s=28, color=theme.marker, zorder=9)
    if start_label:
        ax.annotate(start_label, (sx, sy), xytext=(0, -14), textcoords="offset points", ha="center", va="top",
                    fontsize=fontsize, color=theme.text, zorder=10)
    if goal_label:
        ax.annotate(goal_label, (gx, gy), xytext=(0, -14), textcoords="offset points", ha="center", va="top",
                    fontsize=fontsize, color=theme.text, zorder=10)


def fmt(x: float, digits: int = 1) -> str:
    return f"{x:,.{digits}f}"


def save(fig, path: Path, *, dpi=200):
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=dpi, facecolor=fig.get_facecolor())
    plt.close(fig)
