"""The street scene, built once per map message: measured geometry first, decoration on the pavement.

Measured (what the simulation uses): the carriageway bounded by the exact kerb lines and corner arcs, the
closure barrier and works, and the parked cars the kerb scanner sees (`obstacles`). Decorative (never part
of the simulation): pavement surfacing, kerb stones' upstand, road markings, terraced buildings, lamps,
trees and bollards. Decoration is kept off the carriageway by geometry.frontage/furniture.
"""
import math

from panda3d.core import NodePath, TransparencyAttrib

from . import geometry as g
from .render_util import Builder

ASPHALT = (0.21, 0.22, 0.24, 1.0)
PAVEMENT = (0.62, 0.62, 0.60, 1.0)
KERB_STONE = (0.78, 0.78, 0.76, 1.0)
KERB_FACE = (0.55, 0.55, 0.54, 1.0)
GROUND = (0.46, 0.50, 0.40, 1.0)
MARKING = (0.92, 0.92, 0.90, 1.0)
BRICKS = [(0.62, 0.52, 0.38, 1.0), (0.56, 0.30, 0.24, 1.0), (0.86, 0.85, 0.80, 1.0), (0.66, 0.58, 0.48, 1.0)]
ROOF = (0.26, 0.28, 0.31, 1.0)
WINDOW = (0.16, 0.20, 0.25, 1.0)
FRAME = (0.93, 0.93, 0.90, 1.0)
DOORS = [(0.10, 0.12, 0.25, 1.0), (0.08, 0.08, 0.08, 1.0), (0.45, 0.07, 0.08, 1.0), (0.10, 0.26, 0.16, 1.0)]
LAMP = (0.12, 0.14, 0.13, 1.0)
TRUNK = (0.33, 0.25, 0.18, 1.0)
LEAVES = (0.27, 0.42, 0.24, 1.0)
RED = (0.72, 0.06, 0.06, 1.0)
ZONE_COLOURS = {
    "outage": (0.40, 0.45, 0.95, 0.22), "multipath": (0.95, 0.55, 0.15, 0.22), "ramp": (0.95, 0.75, 0.20, 0.20),
    "kerb_blackout": (0.75, 0.30, 0.85, 0.22), "kerb_shift": (0.95, 0.25, 0.25, 0.26), "slip": (0.30, 0.80, 0.80, 0.18),
}
H = g.KERB_HEIGHT


def _ccw(loop):
    return loop if g.signed_area(loop) > 0 else list(reversed(loop))


def build(scene_msg, quality="medium"):
    """The scene as a NodePath with children: measured (road, kerbs), decor (pavements, buildings, furniture,
    markings) and zones (sensor-condition zones, toggled)."""
    m = scene_msg["map"]
    root = NodePath("world")
    outer, holes = g.kerb_loops(m)
    x0, y0, x1, y1 = g.bounds(m, margin=70.0)

    measured = root.attachNewNode("measured")
    road = Builder("carriageway")
    for lp in outer:
        road.polygon(_ccw(lp), 0.0, ASPHALT, holes=[list(reversed(h)) if g.signed_area(h) > 0 else h for h in holes])
    measured.attachNewNode(road.node())
    kerb = Builder("kerb")
    for lp in outer + holes:
        # Vertical kerb face along the exact kerb, facing the carriageway (on the left of the loop).
        n = len(lp)
        for i in range(n):
            (ax, ay), (bx, by) = lp[i], lp[(i + 1) % n]
            kerb.quad((bx, by, 0.0), (ax, ay, 0.0), (ax, ay, H), (bx, by, H), KERB_FACE)
    measured.attachNewNode(kerb.node())

    decor = root.attachNewNode("decor")
    pave = Builder("pavement")
    for h in holes:
        pave.polygon(_ccw(h), H, PAVEMENT)
    # Outside the network: pavement up to the frontage, then ground.
    frame = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    pave.polygon(frame, H - 0.002, PAVEMENT, holes=[list(reversed(_ccw(lp))) for lp in outer])
    decor.attachNewNode(pave.node())
    stones = Builder("kerb_stones")
    for lp in outer + holes:
        inner = g.offset_loop(lp, 0.15)
        n = len(lp)
        for i in range(n):
            a, b = lp[i], lp[(i + 1) % n]
            c, d = inner[(i + 1) % n], inner[i]
            stones.quad((a[0], a[1], H + 0.003), (d[0], d[1], H + 0.003), (c[0], c[1], H + 0.003), (b[0], b[1], H + 0.003), KERB_STONE)
    decor.attachNewNode(stones.node())
    marks = Builder("markings")
    for (ax, ay), (bx, by) in g.centre_dashes(m):
        L = math.hypot(bx - ax, by - ay)
        nx, ny = -(by - ay) / L * 0.06, (bx - ax) / L * 0.06
        marks.quad((ax - nx, ay - ny, 0.01), (bx - nx, by - ny, 0.01), (bx + nx, by + ny, 0.01), (ax + nx, ay + ny, 0.01), MARKING)
    for x, y, yaw in g.one_way_arrows(m):
        c, s = math.cos(yaw), math.sin(yaw)
        def P(u, v):
            return (x + c * u - s * v, y + s * u + c * v, 0.012)
        marks.quad(P(-2.5, -0.08), P(0.8, -0.08), P(0.8, 0.08), P(-2.5, 0.08), MARKING)
        marks.tri(P(0.6, -0.45), P(2.0, 0.0), P(0.6, 0.45), MARKING)
    decor.attachNewNode(marks.node())

    plots = g.frontage(m)
    if quality == "low":
        plots = plots[::2]
    houses = Builder("buildings")
    for p in plots:
        _house(houses, p)
    decor.attachNewNode(houses.node())
    furn = Builder("furniture")
    items = g.furniture(m)
    if quality == "low":
        items = [it for it in items if it[0] != "tree"]
    for kind, x, y, yaw in items:
        if kind == "lamp":
            furn.box(x, y, H, 0.14, 0.14, 5.2, yaw, LAMP)
            hx, hy = x + 0.6 * math.cos(yaw), y + 0.6 * math.sin(yaw)
            furn.box(hx, hy, H + 5.0, 1.2, 0.22, 0.14, yaw, LAMP)
        elif kind == "tree":
            furn.box(x, y, H, 0.28, 0.28, 2.6, 0.3, TRUNK)
            for z, w in ((2.4, 3.2), (3.4, 2.6), (4.3, 1.6)):
                furn.box(x, y, H + z, w, w, 1.2, 0.3 + z, LEAVES)
        else:
            furn.box(x, y, H, 0.18, 0.18, 0.9, 0.0, LAMP)
    _landmarks(furn, m)
    decor.attachNewNode(furn.node())
    ground = Builder("ground")
    big = 600.0
    ground.polygon([(x0 - big, y0 - big), (x1 + big, y0 - big), (x1 + big, y1 + big), (x0 - big, y1 + big)], H - 0.05, GROUND,
                   holes=[[(x0, y0), (x0, y1), (x1, y1), (x1, y0)]])
    decor.attachNewNode(ground.node())

    zones = root.attachNewNode("zones")
    for kind, rects in (scene_msg.get("zones") or {}).items():
        if not rects:
            continue
        zb = Builder("zone_" + kind)
        col = ZONE_COLOURS.get(kind, (1, 1, 1, 0.2))
        for rx0, ry0, rx1, ry1 in rects:
            zb.quad((rx0, ry0, 0.03), (rx1, ry0, 0.03), (rx1, ry1, 0.03), (rx0, ry1, 0.03), col)
        zn = zones.attachNewNode(zb.node())
        zn.setTransparency(TransparencyAttrib.MAlpha)
        zn.setDepthWrite(False)
        zn.setBin("transparent", 5)

    sensed = root.attachNewNode("sensed_objects")  # objects the kerb scanner can see
    cars = Builder("parked_cars")
    for cx, cy, yaw, L, W in scene_msg.get("obstacles") or []:
        cars.box(cx, cy, 0.0, L, W, 0.95, yaw, (0.20, 0.24, 0.32, 1.0))
        cars.box(cx - 0.2 * math.cos(yaw), cy - 0.2 * math.sin(yaw), 0.95, 0.55 * L, 0.9 * W, 0.5, yaw, (0.12, 0.14, 0.18, 1.0))
    sensed.attachNewNode(cars.node())
    measured.flattenStrong()
    decor.flattenStrong()
    sensed.flattenStrong()
    return root


def _house(b, p):
    c, s = math.cos(p["heading"]), math.sin(p["heading"])  # heading = facing the street
    w, d = p["width"], p["depth"]
    storeys = p["storeys"]
    hgt = 3.0 * storeys + 0.4
    col = BRICKS[p["style"]]
    # Footprint behind the front line (centre of the front at (x, y)).
    bx, by = p["x"] - c * 0.5 * d, p["y"] - s * 0.5 * d
    yaw = p["heading"]
    b.box(bx, by, H, d, w, hgt, yaw, col, top=ROOF)
    # Pitched roof along the frontage.
    ridge = 1.8 + 0.6 * (p["style"] % 2)
    def P(u, v, z):  # u: towards the street from the house centre, v: along the frontage
        return (bx + c * u - s * v, by + s * u + c * v, z)
    z0 = H + hgt
    b.quad(P(0.5 * d, -0.5 * w, z0), P(0.5 * d, 0.5 * w, z0), P(0.0, 0.5 * w, z0 + ridge), P(0.0, -0.5 * w, z0 + ridge), ROOF)
    b.quad(P(-0.5 * d, 0.5 * w, z0), P(-0.5 * d, -0.5 * w, z0), P(0.0, -0.5 * w, z0 + ridge), P(0.0, 0.5 * w, z0 + ridge), ROOF)
    b.tri(P(0.5 * d, 0.5 * w, z0), P(-0.5 * d, 0.5 * w, z0), P(0.0, 0.5 * w, z0 + ridge), col)
    b.tri(P(-0.5 * d, -0.5 * w, z0), P(0.5 * d, -0.5 * w, z0), P(0.0, -0.5 * w, z0 + ridge), col)
    # Street facade: sash windows on every storey, a door on the ground floor.
    f = 0.5 * d + 0.02
    for k in range(storeys):
        zw = H + 0.9 + 3.0 * k
        for v in (-0.22 * w, 0.22 * w) if k > 0 else (0.22 * w,):
            b.quad(P(f, v - 0.55, zw - 0.05), P(f, v + 0.55, zw - 0.05), P(f, v + 0.55, zw + 1.65), P(f, v - 0.55, zw + 1.65), FRAME)
            b.quad(P(f + 0.01, v - 0.45, zw), P(f + 0.01, v + 0.45, zw), P(f + 0.01, v + 0.45, zw + 1.55), P(f + 0.01, v - 0.45, zw + 1.55), WINDOW)
    dv = -0.22 * w
    door = DOORS[(p["style"] + storeys) % 4]
    b.quad(P(f, dv - 0.5, H), P(f, dv + 0.5, H), P(f, dv + 0.5, H + 2.3), P(f, dv - 0.5, H + 2.3), door)


def _landmarks(b, m):
    """One red telephone kiosk and one post box, near the first junction's corner (decorative)."""
    if not m["kerb_arcs"]:
        return
    cx, cy, r, mid, half = m["kerb_arcs"][0]
    x, y = cx + (r - 2.2) * math.cos(mid), cy + (r - 2.2) * math.sin(mid)
    b.box(x, y, H, 1.0, 1.0, 2.5, mid, RED, top=RED)
    b.box(x + 1.8 * math.cos(mid + 1.2), y + 1.8 * math.sin(mid + 1.2), H, 0.45, 0.45, 1.25, mid, RED, top=(0.1, 0.1, 0.1, 1.0))
