#!/usr/bin/env python3
"""Generates the Gazebo world and the planning roadmap from the warehouse layout.

maps/warehouse_layout.txt is the single source of the warehouse geometry. This script writes

  worlds/warehouse.sdf        the Gazebo world: floor, walls, racks, tables, lane and station markings
  maps/warehouse_roadmap.txt  the roadmap the engine loads: graph frame, static obstacles, cost zones,
                              stations, nodes (graph units) and edges (weights)

so the simulated world and the planning graph are aligned by construction. With --check it regenerates
both in memory and exits with status 1 if a committed file differs; the adapter tests and CI run that.

Standard library only.
"""

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
LAYOUT = PKG / "maps" / "warehouse_layout.txt"
ROADMAP = PKG / "maps" / "warehouse_roadmap.txt"
WORLD = PKG / "worlds" / "warehouse.sdf"
EPS = 1e-6


@dataclass
class Obstacle:
    kind: str
    min_x: float
    min_y: float
    max_x: float
    max_y: float
    height: float
    label: str


@dataclass
class Layout:
    name: str = ""
    hall: tuple = ()
    wall_thickness: float = 0.2
    wall_height: float = 1.5
    obstacles: list = field(default_factory=list)
    lanes: list = field(default_factory=list)  # (label, x0, y0, x1, y1)
    node_spacing: float = 0.5
    stations: list = field(default_factory=list)  # (name, x, y)
    zones: list = field(default_factory=list)  # (min_x, min_y, max_x, max_y, multiplier, label)
    frame: tuple = (1.0, 0.0, 0.0, 0.0)  # scale, yaw, origin_x, origin_y


def fmt(value):
    """Shortest fixed-point text for a coordinate: at most 9 decimals, no trailing zeros, no '-0'."""
    text = f"{value:.9f}".rstrip("0").rstrip(".")
    return "0" if text in ("-0", "") else text


def parse_layout(path):
    layout = Layout()
    for number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        key, *args = line.split()
        try:
            if key == "name":
                layout.name = args[0]
            elif key == "hall":
                layout.hall = tuple(float(v) for v in args[:4])
            elif key == "wall":
                layout.wall_thickness, layout.wall_height = float(args[0]), float(args[1])
            elif key in ("rack", "table"):
                values = [float(v) for v in args[:5]]
                layout.obstacles.append(Obstacle(key, *values, args[5]))
            elif key == "lane":
                layout.lanes.append((args[0], *(float(v) for v in args[1:5])))
            elif key == "node_spacing":
                layout.node_spacing = float(args[0])
            elif key == "station":
                layout.stations.append((args[0], float(args[1]), float(args[2])))
            elif key == "zone":
                layout.zones.append((*(float(v) for v in args[:5]), args[5]))
            elif key == "graph_frame":
                layout.frame = tuple(float(v) for v in args[:4])
            else:
                raise ValueError(f"unknown keyword '{key}'")
        except (IndexError, ValueError) as error:
            sys.exit(f"{path.name}:{number}: {error}")
    if len(layout.hall) != 4 or not layout.lanes or layout.node_spacing <= 0 or layout.frame[0] <= 0:
        sys.exit(f"{path.name}: needs a hall, at least one lane, a positive node_spacing and graph_frame scale")
    for obstacle in layout.obstacles:
        if obstacle.min_x >= obstacle.max_x or obstacle.min_y >= obstacle.max_y or obstacle.height <= 0:
            sys.exit(f"{path.name}: obstacle {obstacle.label} has an empty footprint or no height")
    return layout


def point_parameter(px, py, segment):
    """Parameter t in [0, 1] when (px, py) lies on the segment, otherwise None."""
    x0, y0, x1, y1 = segment
    dx, dy = x1 - x0, y1 - y0
    t = ((px - x0) * dx + (py - y0) * dy) / (dx * dx + dy * dy)
    if t < -EPS or t > 1 + EPS:
        return None
    if math.hypot(px - (x0 + t * dx), py - (y0 + t * dy)) > EPS:
        return None
    return min(1.0, max(0.0, t))


def crossing_parameter(a, b):
    """Parameter on a of the point where segments a and b cross, or None when parallel or apart."""
    x0, y0, x1, y1 = a
    x2, y2, x3, y3 = b
    d1x, d1y, d2x, d2y = x1 - x0, y1 - y0, x3 - x2, y3 - y2
    denominator = d1x * d2y - d1y * d2x
    if abs(denominator) < EPS:
        return None
    t = ((x2 - x0) * d2y - (y2 - y0) * d2x) / denominator
    u = ((x2 - x0) * d1y - (y2 - y0) * d1x) / denominator
    return t if (-EPS <= t <= 1 + EPS and -EPS <= u <= 1 + EPS) else None


def zone_multiplier(layout, x, y):
    multiplier = 1.0
    for min_x, min_y, max_x, max_y, factor, _ in layout.zones:
        if min_x - EPS <= x <= max_x + EPS and min_y - EPS <= y <= max_y + EPS:
            multiplier = max(multiplier, factor)
    return multiplier


def build_roadmap(layout):
    """Nodes (map metres) and weighted edges from the lanes.

    Lane end points, end points touching another lane and lane crossings split the lanes into pieces;
    each piece is divided into equal parts no longer than node_spacing. Node ids follow (x, y) order.
    """
    segments = [lane[1:] for lane in layout.lanes]
    cuts = [{0.0, 1.0} for _ in segments]
    for i, a in enumerate(segments):
        for j, b in enumerate(segments):
            if i == j:
                continue
            for px, py in ((b[0], b[1]), (b[2], b[3])):
                t = point_parameter(px, py, a)
                if t is not None:
                    cuts[i].add(round(t, 12))
            t = crossing_parameter(a, b)
            if t is not None:
                cuts[i].add(round(min(1.0, max(0.0, t)), 12))

    points = {}
    pairs = set()
    for (x0, y0, x1, y1), parameters in zip(segments, cuts):
        parameters = sorted(parameters)
        length = math.hypot(x1 - x0, y1 - y0)
        for t0, t1 in zip(parameters, parameters[1:]):
            if t1 - t0 < EPS:
                continue
            parts = max(1, math.ceil(length * (t1 - t0) / layout.node_spacing - 1e-9))
            previous = None
            for k in range(parts + 1):
                t = t0 + (t1 - t0) * k / parts
                x, y = round(x0 + t * (x1 - x0), 9), round(y0 + t * (y1 - y0), 9)
                key = (round(x, 6), round(y, 6))
                points.setdefault(key, (x, y))
                if previous is not None and previous != key:
                    pairs.add(tuple(sorted((previous, key))))
                previous = key

    ordered = sorted(points)
    ids = {key: index for index, key in enumerate(ordered)}
    nodes = [points[key] for key in ordered]
    edges = []
    for a, b in pairs:
        (ax, ay), (bx, by) = points[a], points[b]
        length = math.hypot(bx - ax, by - ay)
        weight = length * zone_multiplier(layout, (ax + bx) / 2, (ay + by) / 2)
        edges.append((min(ids[a], ids[b]), max(ids[a], ids[b]), round(weight, 9)))
    edges.sort()

    stations = []
    for name, x, y in layout.stations:
        key = (round(x, 6), round(y, 6))
        if key not in ids:
            sys.exit(f"station {name} at ({x}, {y}) is not on a roadmap node")
        stations.append((name, ids[key]))
    return nodes, edges, stations


def to_graph(layout, x, y):
    scale, yaw, origin_x, origin_y = layout.frame
    dx, dy = x - origin_x, y - origin_y
    c, s = math.cos(yaw), math.sin(yaw)
    return round((c * dx + s * dy) / scale, 9), round((-s * dx + c * dy) / scale, 9)


def wall_rects(layout):
    min_x, min_y, max_x, max_y = layout.hall
    t = layout.wall_thickness
    return [
        ("wall_south", min_x - t, min_y - t, max_x + t, min_y),
        ("wall_north", min_x - t, max_y, max_x + t, max_y + t),
        ("wall_west", min_x - t, min_y, min_x, max_y),
        ("wall_east", max_x, min_y, max_x + t, max_y),
    ]


def roadmap_text(layout):
    nodes, edges, stations = build_roadmap(layout)
    scale, yaw, origin_x, origin_y = layout.frame
    lines = [
        "ane-roadmap 1",
        "# Generated by ane_sim/scripts/generate_world.py from maps/warehouse_layout.txt; do not edit.",
        "# frame: scale yaw origin_x origin_y, with map = R(yaw) * (scale * graph) + origin; map lengths are metres.",
        "# obstacle: kind min_x min_y max_x max_y height label (map frame). zone: min_x min_y max_x max_y multiplier label.",
        "# node: id x y in graph units. edge: from to weight; two-way; weight = length in metres x zone multiplier.",
        f"name {layout.name}",
        f"frame {fmt(scale)} {fmt(yaw)} {fmt(origin_x)} {fmt(origin_y)}",
        "hall " + " ".join(fmt(v) for v in layout.hall),
    ]
    for label, min_x, min_y, max_x, max_y in wall_rects(layout):
        lines.append(f"obstacle wall {fmt(min_x)} {fmt(min_y)} {fmt(max_x)} {fmt(max_y)} {fmt(layout.wall_height)} {label}")
    for o in layout.obstacles:
        lines.append(f"obstacle {o.kind} {fmt(o.min_x)} {fmt(o.min_y)} {fmt(o.max_x)} {fmt(o.max_y)} {fmt(o.height)} {o.label}")
    for min_x, min_y, max_x, max_y, factor, label in layout.zones:
        lines.append(f"zone {fmt(min_x)} {fmt(min_y)} {fmt(max_x)} {fmt(max_y)} {fmt(factor)} {label}")
    for name, node in stations:
        lines.append(f"station {name} {node}")
    for index, (x, y) in enumerate(nodes):
        gx, gy = to_graph(layout, x, y)
        lines.append(f"node {index} {fmt(gx)} {fmt(gy)}")
    for a, b, weight in edges:
        lines.append(f"edge {a} {b} {fmt(weight)}")
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------------------------------
# Gazebo world


def material(r, g, b, a=1.0):
    return (f"<material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse>"
            f"<specular>0.1 0.1 0.1 1</specular></material>")


def box_visual(name, cx, cy, cz, sx, sy, sz, colour, yaw=0.0):
    return (f"<visual name=\"{name}\"><pose>{fmt(cx)} {fmt(cy)} {fmt(cz)} 0 0 {fmt(yaw)}</pose>"
            f"<geometry><box><size>{fmt(sx)} {fmt(sy)} {fmt(sz)}</size></box></geometry>{material(*colour)}</visual>")


def box_collision(name, cx, cy, cz, sx, sy, sz):
    return (f"<collision name=\"{name}\"><pose>{fmt(cx)} {fmt(cy)} {fmt(cz)} 0 0 0</pose>"
            f"<geometry><box><size>{fmt(sx)} {fmt(sy)} {fmt(sz)}</size></box></geometry></collision>")


RACK_STEEL = (0.18, 0.32, 0.55)
RACK_BEAM = (0.85, 0.45, 0.10)
GOODS = (0.66, 0.52, 0.36)
# Upper-level goods alternate between cardboard and plastic totes; visual only, above the scan plane.
GOODS_UPPER = ((0.66, 0.52, 0.36), (0.74, 0.62, 0.45), (0.25, 0.42, 0.62), (0.70, 0.56, 0.38))
WALL = (0.80, 0.80, 0.78)
TABLE = (0.30, 0.34, 0.40)
LANE = (0.95, 0.78, 0.10)
PAD_BORDER = (0.97, 0.97, 0.95)
DISPATCH = (0.15, 0.55, 0.25)
DELIVERY = (0.15, 0.35, 0.75)
ZONE = (0.95, 0.55, 0.15, 0.35)
FLOOR = (0.56, 0.58, 0.60)


def rack_model(o):
    """A pallet rack: solid collision box; visuals of goods (solid up to 0.5 m, so the lidar sees a
    closed face at its 0.18 m scan height), uprights, beams and boxes on the upper levels."""
    sx, sy, h = o.max_x - o.min_x, o.max_y - o.min_y, o.height
    cx, cy = (o.min_x + o.max_x) / 2, (o.min_y + o.max_y) / 2
    parts = [box_collision("collision", 0, 0, h / 2, sx, sy, h),
             box_visual("goods_floor", 0, 0, 0.25, sx - 0.04, sy - 0.04, 0.5, GOODS)]
    bays = max(1, round(sx / 1.75))
    for k in range(bays + 1):
        x = -sx / 2 + sx * k / bays
        x = min(max(x, -sx / 2 + 0.04), sx / 2 - 0.04)
        for side, y in (("a", -sy / 2 + 0.04), ("b", sy / 2 - 0.04)):
            parts.append(box_visual(f"upright_{k}{side}", x, y, h / 2, 0.08, 0.08, h, RACK_STEEL))
    for level, z in enumerate((1.0, 1.6, h - 0.04)):
        for side, y in (("a", -sy / 2 + 0.04), ("b", sy / 2 - 0.04)):
            parts.append(box_visual(f"beam_{level}{side}", 0, y, z, sx, 0.06, 0.08, RACK_BEAM))
    for k in range(bays):
        x = -sx / 2 + sx * (k + 0.5) / bays
        parts.append(box_visual(f"goods_{k}", x, 0, 1.3, sx / bays - 0.3, sy - 0.2, 0.5, GOODS_UPPER[k % len(GOODS_UPPER)]))
    body = "".join(parts)
    return (f"<model name=\"{o.label}\"><static>true</static><pose>{fmt(cx)} {fmt(cy)} 0 0 0 0</pose>"
            f"<link name=\"link\">{body}</link></model>")


def table_model(o):
    sx, sy, h = o.max_x - o.min_x, o.max_y - o.min_y, o.height
    cx, cy = (o.min_x + o.max_x) / 2, (o.min_y + o.max_y) / 2
    body = box_collision("collision", 0, 0, h / 2, sx, sy, h) + box_visual("body", 0, 0, h / 2, sx, sy, h, TABLE)
    return (f"<model name=\"{o.label}\"><static>true</static><pose>{fmt(cx)} {fmt(cy)} 0 0 0 0</pose>"
            f"<link name=\"link\">{body}</link></model>")


def walls_model(layout):
    parts = []
    for label, min_x, min_y, max_x, max_y in wall_rects(layout):
        cx, cy, sx, sy = (min_x + max_x) / 2, (min_y + max_y) / 2, max_x - min_x, max_y - min_y
        h = layout.wall_height
        parts.append(box_collision(f"{label}_collision", cx, cy, h / 2, sx, sy, h))
        parts.append(box_visual(f"{label}_visual", cx, cy, h / 2, sx, sy, h, WALL))
    return f"<model name=\"walls\"><static>true</static><link name=\"link\">{''.join(parts)}</link></model>"


def markings_model(layout):
    """Visual-only floor tape: lane centre lines, station pads, cost zone. No collision geometry."""
    parts = []
    for label, x0, y0, x1, y1 in layout.lanes:
        length = math.hypot(x1 - x0, y1 - y0)
        yaw = math.atan2(y1 - y0, x1 - x0)
        parts.append(box_visual(f"lane_{label}", (x0 + x1) / 2, (y0 + y1) / 2, 0.001, length + 0.06, 0.06, 0.002, LANE, yaw))
    for name, x, y in layout.stations:
        colour = DISPATCH if name == "dispatch" else DELIVERY
        parts.append(box_visual(f"station_{name}_border", x, y, 0.0012, 0.82, 0.82, 0.0024, PAD_BORDER))
        parts.append(box_visual(f"station_{name}", x, y, 0.0018, 0.7, 0.7, 0.0036, colour))
    for min_x, min_y, max_x, max_y, _, label in layout.zones:
        parts.append(box_visual(f"zone_{label}", (min_x + max_x) / 2, (min_y + max_y) / 2, 0.0005,
                                max_x - min_x, max_y - min_y, 0.001, ZONE))
    return f"<model name=\"floor_markings\"><static>true</static><link name=\"link\">{''.join(parts)}</link></model>"


def world_text(layout):
    min_x, min_y, max_x, max_y = layout.hall
    floor_x, floor_y = max_x - min_x + 6, max_y - min_y + 6
    models = [walls_model(layout), markings_model(layout)]
    models += [rack_model(o) if o.kind == "rack" else table_model(o) for o in layout.obstacles]
    body = "\n    ".join(models)
    return f"""<?xml version="1.0" ?>
<!-- Generated by ane_sim/scripts/generate_world.py from maps/warehouse_layout.txt; do not edit. -->
<sdf version="1.10">
  <world name="{layout.name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <!-- Lighting and shadows affect only rendered images, not the lidar (which measures depth). -->
    <scene>
      <ambient>0.45 0.45 0.47 1</ambient>
      <background>0.86 0.88 0.91 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>
    <light type="directional" name="sun">
      <visualize>false</visualize>
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.88 0.87 0.84 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
      <direction>0.45 0.55 -0.70</direction>
    </light>
    <light type="directional" name="fill">
      <visualize>false</visualize>
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.30 0.31 0.34 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>-0.5 -0.4 -0.75</direction>
    </light>
    <model name="floor">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>{fmt(floor_x)} {fmt(floor_y)}</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>{fmt(floor_x)} {fmt(floor_y)}</size></plane></geometry>{material(*FLOOR)}</visual>
      </link>
    </model>
    {body}
  </world>
</sdf>
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="fail if the committed files are out of date")
    args = parser.parse_args()
    layout = parse_layout(LAYOUT)
    outputs = {ROADMAP: roadmap_text(layout), WORLD: world_text(layout)}
    if args.check:
        stale = [path for path, text in outputs.items()
                 if not path.exists() or path.read_text(encoding="utf-8") != text]
        for path in stale:
            print(f"out of date: {path.relative_to(PKG)} (run scripts/generate_world.py)")
        if not stale:
            print("roadmap and world match the layout")
        return 1 if stale else 0
    for path, text in outputs.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8", newline="\n")
        print(f"wrote {path.relative_to(PKG)}")
    nodes, edges, stations = build_roadmap(layout)
    print(f"roadmap: {len(nodes)} nodes, {len(edges)} edges, stations {dict(stations)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
