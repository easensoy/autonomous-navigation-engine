"""The district scenes: Kensington and Knightsbridge, built from the backend's district map and an OpenStreetMap
scenery file (districts/*.json, ODbL) in the same local frame.

Measured, from the backend's map message, exactly as the simulation uses it: the carriageway bounded by the
kerb lines and arcs (Queen's Gate's central reservation included), the kerb upstand on every kerb piece, and
nothing at a patch edge, where the modelled area ends (drawn as a hatched apron level with the road: no kerb
the scanner could be expected to see). Sourced, from OpenStreetMap: building footprints, their storeys and
names, parks, trees, walls and railings, and the mapped centrelines (the provenance overlay, key V).
Decorative and approximate: facades and materials (facades.py), roofs, street trees on Queen's Gate's
reservation and in the park, lamps, a kiosk and a pillar box, name plates, road markings. Decoration is kept
off the carriageway, and every building and canopy is registered as an occluder for the cameras.
"""
import json
import math
import pathlib
import random

from panda3d.core import LineSegs, NodePath, TextNode, TransparencyAttrib, Vec4

from . import facades as F
from . import geometry as g
from .district_layout import STOREY, Kerb, layout, seg_distance, stable_hash  # noqa: F401 - Kerb is re-exported
from .occlusion import point_in_polygon
from .render_util import Builder, Materials

H = g.KERB_HEIGHT
KERB_FACE = (0.62, 0.62, 0.60, 1.0)
KERB_STONE = (0.80, 0.80, 0.78, 1.0)
WHITE = (1.0, 1.0, 1.0, 1.0)
MARK_WHITE = (0.93, 0.93, 0.90, 1.0)
MARK_YELLOW = (0.93, 0.76, 0.10, 1.0)
LAMP = (0.07, 0.10, 0.08, 1.0)
RED = (0.74, 0.06, 0.06, 1.0)
TRUNK = (0.42, 0.38, 0.32, 1.0)
LEAVES = [(0.25, 0.40, 0.20, 1.0), (0.30, 0.45, 0.22, 1.0), (0.22, 0.36, 0.19, 1.0)]
ZONE_COLOURS = {
    "outage": (0.40, 0.45, 0.95, 0.22), "multipath": (0.95, 0.55, 0.15, 0.22), "ramp": (0.95, 0.75, 0.20, 0.20),
    "kerb_blackout": (0.75, 0.30, 0.85, 0.22), "kerb_shift": (0.95, 0.25, 0.25, 0.26), "slip": (0.30, 0.80, 0.80, 0.18),
}


TOREY = {"stucco_terrace": 3.4, "mansion_block": 3.1, "brick_terrace": 3.1, "commercial": 3.6, "harrods": 4.2, "institutional": 3.8,
          "church": 4.0, "rah": 4.0}
EFAULT_LEVELS = {"stucco_terrace": 5, "mansion_block": 6, "brick_terrace": 4, "commercial": 5, "harrods": 7, "institutional": 5, "church": 4, "rah": 6}


def build(scene_msg, quality="medium"):
    m = scene_msg["map"]
    info = m["district"]
    plan = layout(scene_msg, quality)
    data, kerb, rng, occ = plan["data"], plan["kerb"], plan["rng"], plan["occluders"]
    root = NodePath("world")
    report = {"district": info["id"], "buildings": len(plan["buildings"]), "styles": {}, "dropped": plan["dropped"],
              "trees_osm": sum(1 for t in plan["trees"] if t[2] == "osm"), "trees_decorative": sum(1 for t in plan["trees"] if t[2] != "osm"),
              "barriers": 0}
    x0, y0, x1, y1 = data["bounds"]

    # ---- Measured: the carriageway and its kerb, exactly the backend's --------------------------------
    measured = root.attachNewNode("measured")
    road = Materials()
    for lp in kerb.outer:
        road["asphalt"].hpoly(lp, 0.0, WHITE, holes=[list(reversed(h)) if g.signed_area(h) > 0 else h for h in kerb.holes])
    road.attach(measured)
    kf = Builder("kerb")
    for piece in plan["kerb_pieces"]:  # the backend's kerb lines and arcs; none at a patch edge
        for (ax, ay), (bx, by) in zip(piece, piece[1:]):
            kf.quad((bx, by, 0.0), (ax, ay, 0.0), (ax, ay, H), (bx, by, H), KERB_FACE)
    measured.attachNewNode(kf.node())

    # ---- Patch edges: the road carries on beyond the model, shown as a hatched apron -------------------
    decor = root.attachNewNode("decor")
    mats = Materials()
    vis_outer = [list(lp) for lp in kerb.outer]
    for i in sorted(kerb.boundary):
        ax, ay, bx, by, nx, ny = m["kerb_lines"][i]
        for lp in vis_outer:
            k = _find_edge(lp, (ax, ay), (bx, by))
            if k is None:
                continue
            a, b = lp[k], lp[(k + 1) % len(lp)]
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            t = ((b[0] - a[0]) / L, (b[1] - a[1]) / L)
            out = (t[1], -t[0])  # the carriageway is on the left: beyond the edge is on the right
            ext, side = 18.0, 4.0
            a2 = (a[0] - t[0] * side, a[1] - t[1] * side)
            b2 = (b[0] + t[0] * side, b[1] + t[1] * side)
            a3 = (a2[0] + out[0] * ext, a2[1] + out[1] * ext)
            b3 = (b2[0] + out[0] * ext, b2[1] + out[1] * ext)
            lp[k + 1:k + 1] = [a2, a3, b3, b2]
            mats["hatch"].hpoly([a2, b2, b3, a3][::-1] if g.signed_area([a2, b2, b3, a3]) < 0 else [a2, b2, b3, a3], 0.003, (0.85, 0.85, 0.85, 1.0))
            break

    # ---- Pavements, islands, parks ---------------------------------------------------------------------
    frame = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    mats["paving"].hpoly(frame, H, WHITE, holes=[list(reversed(_ccw(lp))) for lp in vis_outer])
    for h in kerb.holes:
        mats["paving"].hpoly(h if g.signed_area(h) > 0 else list(reversed(h)), H, WHITE)
    for ring in plan["parks"]:
        mats["grass"].hpoly(ring, H + 0.01, WHITE)
    _kerb_stones(mats, m, kerb)

    # ---- Buildings ---------------------------------------------------------------------------------------
    signs = []
    footprints = [b for b in data["buildings"] if not b["part"]]
    for item in plan["buildings"]:
        b, ring, style = item["b"], item["ring"], item["style"]
        if style == "rah":
            _royal_albert_hall(mats, b)
        else:
            signs += _building(mats, b, ring, style, item["levels"], item["height"], item["fronts"], kerb, rng, quality)
        report["styles"][style] = report["styles"].get(style, 0) + 1

    # ---- Trees, railings, street furniture -------------------------------------------------------------
    for x, y, src in plan["trees"]:
        _tree(mats, x, y, rng)
    for bar in data["barriers"]:
        if _barrier(mats, bar, kerb):
            report["barriers"] += 1
    _furniture(mats, m, kerb, footprints, rng, info["id"])
    mats.attach(decor)

    # ---- Markings ----------------------------------------------------------------------------------------
    marks = Builder("markings")
    _markings(marks, m)
    decor.attachNewNode(marks.node())

    # ---- Signs and name plates (text) -------------------------------------------------------------------
    text_root = root.attachNewNode("signs")  # kept out of the flattened decor: flattening turns text nodes askew
    for (x, y, z, yaw, col, word) in signs[:60]:
        _sign(text_root, x, y, z, yaw, word, col)
    _name_plates(text_root, m, kerb, footprints)

    # ---- Zones and sensed objects, as on the grids -------------------------------------------------------
    zones = root.attachNewNode("zones")
    for kind, rects in (scene_msg.get("zones") or {}).items():
        if not rects:
            continue
        zb = Builder("zone_" + kind)
        col = ZONE_COLOURS.get(kind, (1, 1, 1, 0.2))
        for rx0, ry0, rx1, ry1 in _union_rows(rects):  # the zone's area once: overlapping rectangles do not darken
            zb.quad((rx0, ry0, 0.03), (rx1, ry0, 0.03), (rx1, ry1, 0.03), (rx0, ry1, 0.03), col)
        zn = zones.attachNewNode(zb.node())
        zn.setTransparency(TransparencyAttrib.MAlpha)
        zn.setDepthWrite(False)
        zn.setBin("transparent", 5)
    sensed = root.attachNewNode("sensed_objects")
    cars = Builder("parked_cars")
    for cx, cy, yaw, L, W in scene_msg.get("obstacles") or []:
        cars.box(cx, cy, 0.0, L, W, 0.95, yaw, (0.20, 0.24, 0.32, 1.0))
    sensed.attachNewNode(cars.node())

    # ---- Provenance overlay: OpenStreetMap centrelines (cyan) and modelled lanes (amber), key V ---------
    prov = root.attachNewNode("provenance")
    ls = LineSegs("osm")
    ls.setThickness(2.0)
    ls.setColor(Vec4(0.2, 0.9, 1.0, 1.0))
    for w in data["ways"]:
        if w["highway"] in ("footway", "path", "steps", "cycleway", "corridor", "elevator"):
            continue
        pts = w["pts"]
        ls.moveTo(pts[0][0], pts[0][1], 0.35)
        for x, y in pts[1:]:
            ls.drawTo(x, y, 0.35)
    prov.attachNewNode(ls.create())
    ls2 = LineSegs("lanes")
    ls2.setThickness(2.0)
    ls2.setColor(Vec4(1.0, 0.7, 0.15, 1.0))
    for ln in m["lanes"]:
        ls2.moveTo(ln["x0"], ln["y0"], 0.4)
        ls2.drawTo(ln["x1"], ln["y1"], 0.4)
    prov.attachNewNode(ls2.create())
    prov.setLightOff(1)
    prov.setShaderOff(1)
    prov.hide()

    _sky(root)
    measured.flattenStrong()
    decor.flattenStrong()
    root.setPythonTag("occluders", occ)
    root.setPythonTag("district_report", report)
    return root


# ---- Helpers ---------------------------------------------------------------------------------------------

def _union_rows(rects, cell=1.0):
    """The union of axis-aligned rectangles as non-overlapping row strips on a `cell` grid."""
    rows = {}
    for x0, y0, x1, y1 in rects:
        for j in range(int(math.floor(y0 / cell)), int(math.ceil(y1 / cell))):
            rows.setdefault(j, []).append((x0, x1))
    out = []
    for j, spans in sorted(rows.items()):
        spans.sort()
        cur0, cur1 = spans[0]
        for a, b in spans[1:] + [(None, None)]:
            if a is not None and a <= cur1:
                cur1 = max(cur1, b)
                continue
            out.append((cur0, j * cell, cur1, (j + 1) * cell))
            if a is not None:
                cur0, cur1 = a, b
    return out


def _ccw(lp):
    return lp if g.signed_area(lp) > 0 else list(reversed(lp))


def _find_edge(lp, a, b, tol=1e-3):
    n = len(lp)
    for k in range(n):
        p, q = lp[k], lp[(k + 1) % n]
        if (abs(p[0] - a[0]) < tol and abs(p[1] - a[1]) < tol and abs(q[0] - b[0]) < tol and abs(q[1] - b[1]) < tol) or \
           (abs(p[0] - b[0]) < tol and abs(p[1] - b[1]) < tol and abs(q[0] - a[0]) < tol and abs(q[1] - a[1]) < tol):
            return k
    return None


def _kerb_stones(mats, m, kerb):
    b = mats["stone"]
    for i, (ax, ay, bx, by, nx, ny) in enumerate(m["kerb_lines"]):
        if i in kerb.boundary:
            continue
        dx, dy = ny, -nx
        if (bx - ax) * dx + (by - ay) * dy < 0:
            ax, ay, bx, by = bx, by, ax, ay
        ox, oy = -nx * 0.15, -ny * 0.15
        b.quad((ax, ay, H + 0.003), (ax + ox, ay + oy, H + 0.003), (bx + ox, by + oy, H + 0.003), (bx, by, H + 0.003), KERB_STONE)


WALLS = {"stucco_terrace": ("stucco", None), "mansion_block": ("brick_red", None), "brick_terrace": ("brick_stock", None),
         "commercial": (None, None), "harrods": ("terracotta", None), "institutional": (None, None), "church": ("stone", None)}
STUCCO_TINTS = [(1.0, 1.0, 1.0, 1.0), (0.98, 0.97, 0.94, 1.0), (1.0, 0.98, 0.93, 1.0), (0.95, 0.95, 0.95, 1.0), (0.99, 0.96, 0.90, 1.0)]


def _building(mats, b, ring, style, levels, height, fronts, kerb, rng, quality):
    h = stable_hash(b["osm"])
    wall_mat = WALLS[style][0]
    if style == "commercial":
        wall_mat = ("stone", "brick_red", "brick_stock", "stone")[h % 4]
    elif style == "institutional":
        wall_mat = ("concrete", "brick_brown", "concrete")[h % 3]
    tint = STUCCO_TINTS[h % len(STUCCO_TINTS)] if wall_mat == "stucco" else (1.0, 1.0, 1.0, 1.0)
    base, top = H, H + height
    n = len(ring)
    u = 0.0
    for i in range(n):
        a, c = ring[i], ring[(i + 1) % n]
        mats[wall_mat].wall(a, c, base, top, tint, u)
        u += math.hypot(c[0] - a[0], c[1] - a[1])
    for hole in b["inner"]:
        for i in range(len(hole)):
            mats[wall_mat].wall(hole[i], hole[(i + 1) % len(hole)], base, top, tint)
    roof_mat = "slate" if style in ("stucco_terrace", "mansion_block", "brick_terrace", "church") else "concrete"
    mats[roof_mat].hpoly(ring, top, (0.85, 0.85, 0.85, 1.0), holes=b["inner"])
    signs = []
    brng = random.Random(h)
    storeys = max(2, int(round(height / STOREY[style])))
    detail = quality != "low"
    for i in range(n):
        f = fronts[i]
        a, c = tuple(ring[i]), tuple(ring[(i + 1) % n])
        wall = F.Wall(mats, a, c, base, top)
        if f is None:
            if detail and wall.L > 6.0:
                F.plain(wall, STOREY[style], brng)
            continue
        room = max(0.0, f[1] - 1.6)  # pavement depth that may hold an area, steps or a portico
        if style == "stucco_terrace":
            F.stucco_terrace(wall, storeys, brng, room)
        elif style in ("mansion_block", "brick_terrace"):
            F.mansion_block(wall, storeys, brng, room, wall_mat)
        elif style == "commercial":
            for uc, z, col, word in F.commercial(wall, storeys, brng, room, wall_mat) or []:
                x, y, _ = wall.P(uc, 0.18, z)
                signs.append((x, y, z, math.atan2(wall.n[1], wall.n[0]), col, word))
        elif style == "harrods":
            F.harrods(wall, storeys, brng, room)
        elif style == "institutional":
            F.institutional(wall, storeys, brng, room, wall_mat)
        else:
            F.plain(wall, STOREY[style], brng)
    if style == "harrods":
        _harrods_skyline(mats, ring, fronts, top)
    if style in ("stucco_terrace", "brick_terrace", "mansion_block") and detail and abs(g.signed_area(ring)) < 450:
        # A chimney stack on the party wall, set back from the street.
        cx = sum(p[0] for p in ring) / n
        cy = sum(p[1] for p in ring) / n
        mats["brick_stock"].box(cx, cy, top, 0.8, 2.2, 1.5, brng.random() * 3.14, (1, 1, 1, 1))
        for k in (-0.6, 0.0, 0.6):
            mats["dark"].box(cx + k * 0.0, cy + k, top + 1.5, 0.25, 0.25, 0.4, 0.0, (0.55, 0.30, 0.22, 1.0))
    return signs


def _harrods_skyline(mats, ring, fronts, top):
    """After the list entry: octagonal cupolas at the corners of the Brompton Road front, a copper mansard attic
    along it, and over its centre the octagonal dome with terracotta ribs and facings and a cupola above."""
    n = len(ring)
    best = None
    for i in range(n):
        f = fronts[i]
        if f and f[2] == "Brompton Road":
            a, c = ring[i], ring[(i + 1) % n]
            L = math.hypot(c[0] - a[0], c[1] - a[1])
            if best is None or L > best[0]:
                best = (L, a, c)
    if best is None:
        return
    L, a, c = best
    t = ((c[0] - a[0]) / L, (c[1] - a[1]) / L)
    nrm = (t[1], -t[0])
    for (px, py) in (a, c):  # corner cupolas
        sgn = 1.0 if (px, py) == tuple(a) else -1.0
        cx, cy = px - nrm[0] * 3.5 + t[0] * 3.5 * sgn, py - nrm[1] * 3.5 + t[1] * 3.5 * sgn
        _drum(mats, "terracotta", cx, cy, top, top + 4.2, 2.4, (1, 1, 1, 1), sides=8)
        _dome(mats, cx, cy, top + 4.2, 2.4, 2.6, F.COPPER, sides=8)
        _drum(mats, "trim", cx, cy, top + 6.6, top + 7.7, 0.5, F.COPPER, sides=8)
    # The mansard attic along the front: copper, set back behind the parapet.
    for k in range(int(L // 4.0)):
        u0, u1 = k * 4.0 + 6.0, k * 4.0 + 9.0
        if u1 > L - 6.0:
            break
        p0 = (a[0] + t[0] * u0 - nrm[0] * 1.2, a[1] + t[1] * u0 - nrm[1] * 1.2)
        p1 = (a[0] + t[0] * u1 - nrm[0] * 1.2, a[1] + t[1] * u1 - nrm[1] * 1.2)
        mats["trim"].quad((p0[0], p0[1], top), (p1[0], p1[1], top), (p1[0] - nrm[0] * 1.6, p1[1] - nrm[1] * 1.6, top + 3.0),
                          (p0[0] - nrm[0] * 1.6, p0[1] - nrm[1] * 1.6, top + 3.0), F.COPPER)
    mx, my = 0.5 * (a[0] + c[0]) - nrm[0] * 8.0, 0.5 * (a[1] + c[1]) - nrm[1] * 8.0
    _drum(mats, "terracotta", mx, my, top, top + 4.5, 5.5, (1, 1, 1, 1), sides=8)
    _dome(mats, mx, my, top + 4.5, 5.5, 7.0, (0.88, 0.66, 0.53, 1.0), sides=8, rings=9, ribs=(0.70, 0.47, 0.36, 1.0))  # terracotta ribs and facings
    _drum(mats, "trim", mx, my, top + 11.3, top + 13.3, 1.0, F.COPPER, sides=8)


def _drum(mats, mat, cx, cy, z0, z1, r, col, sides=16):
    for k in range(sides):
        a0, a1 = 2 * math.pi * k / sides, 2 * math.pi * (k + 1) / sides
        mats[mat].wall((cx + r * math.cos(a0), cy + r * math.sin(a0)), (cx + r * math.cos(a1), cy + r * math.sin(a1)), z0, z1, col)


def _dome(mats, cx, cy, z0, r, rise, col, sides=16, rings=6, ribs=None):
    """A dome of `sides` faces; with `ribs` (a colour), a raised rib runs up each edge to the crown."""
    b = mats["trim"]
    prof = [(math.cos(0.5 * math.pi * j / rings), z0 + rise * math.sin(0.5 * math.pi * j / rings)) for j in range(rings + 1)]
    prev = [(cx + r * math.cos(2 * math.pi * k / sides), cy + r * math.sin(2 * math.pi * k / sides), z0) for k in range(sides)]
    for f, z in prof[1:]:
        cur = [(cx + r * f * math.cos(2 * math.pi * k / sides), cy + r * f * math.sin(2 * math.pi * k / sides), z) for k in range(sides)]
        for k in range(sides):
            a, bb = prev[k], prev[(k + 1) % sides]
            c, d = cur[(k + 1) % sides], cur[k]
            b.quad(a, bb, c, d, col)
        prev = cur
    if ribs is None:
        return
    w, lift = 0.22, 0.12  # half-width and height of a rib, m
    for k in range(sides):
        a = 2 * math.pi * k / sides
        ca, sa = math.cos(a), math.sin(a)
        tx, ty = -sa, ca
        pts = []
        for f, z in prof[:-1]:
            # Out from the surface: along the profile's normal (radial and up).
            nr, nz = rise * f, r * math.sqrt(max(0.0, 1.0 - f * f))
            L = math.hypot(nr, nz) or 1.0
            px, py = cx + r * f * ca + ca * lift * nr / L, cy + r * f * sa + sa * lift * nr / L
            pz = z + lift * nz / L
            pts.append(((px - tx * w, py - ty * w, pz), (px + tx * w, py + ty * w, pz)))
        for (l0, r0), (l1, r1) in zip(pts, pts[1:]):
            b.quad(l0, r0, r1, l1, ribs)


def _royal_albert_hall(mats, b):
    """The Royal Albert Hall (1867-71, listed grade I) on its mapped elliptical footprint: red brick drum, the
    buff terracotta frieze below the cornice, two tiers of arched windows, and the glass-and-iron dome."""
    ring = b["outer"]
    n = len(ring)
    cx = sum(p[0] for p in ring) / n
    cy = sum(p[1] for p in ring) / n
    base, cornice, top = H, H + 23.0, H + 26.0
    u = 0.0
    for i in range(n):
        a, c = ring[i], ring[(i + 1) % n]
        mats["brick_red"].wall(a, c, base, cornice, (1, 1, 1, 1), u)
        mats["terracotta"].wall(a, c, cornice - 4.5, cornice - 2.6, (0.95, 0.88, 0.70, 1.0), u)  # the frieze band
        mats["stone"].wall(a, c, cornice, top, (1, 1, 1, 1), u)
        u += math.hypot(c[0] - a[0], c[1] - a[1])
    for i in range(n):
        a, c = tuple(ring[i]), tuple(ring[(i + 1) % n])
        w = F.Wall(mats, a, c, base, top)
        if w.L < 3.0:
            continue
        for uc in [w.L * (k + 0.5) / max(1, int(w.L / 4.0)) for k in range(max(1, int(w.L / 4.0)))]:
            w.face("glass", uc - 0.7, uc + 0.7, base + 9.0, base + 13.0, 0.04, (0.8, 0.8, 0.82, 1.0), uv=((0, 0), (1, 0), (1, 1), (0, 1)))
            w.face("glass", uc - 0.55, uc + 0.55, base + 14.5, base + 17.0, 0.04, (0.8, 0.8, 0.82, 1.0), uv=((0, 0), (1, 0), (1, 1), (0, 1)))
            w.face("dark", uc - 0.9, uc + 0.9, base, base + 4.2, 0.03, (0.16, 0.12, 0.10, 1.0))
        w.slab("stone", 0.0, w.L, cornice - 0.2, cornice + 0.4, 0.0, 0.55, (1, 1, 1, 1))
    # The dome: the footprint scaled inwards ring by ring.
    rings, rise = 7, 15.0
    prev = [(p[0], p[1], top) for p in ring]
    for j in range(1, rings + 1):
        f = 0.86 * math.cos(0.5 * math.pi * j / rings) + 0.02
        z = top + rise * math.sin(0.5 * math.pi * j / rings)
        cur = [(cx + (p[0] - cx) * f, cy + (p[1] - cy) * f, z) for p in ring]
        for k in range(n):
            a, bb = prev[k], prev[(k + 1) % n]
            c, d = cur[(k + 1) % n], cur[k]
            mats["trim"].quad(a, bb, c, d, (0.55, 0.60, 0.63, 1.0) if j > 1 else (0.48, 0.44, 0.40, 1.0))
        prev = cur


def _tree(mats, x, y, rng):
    r = random.Random(stable_hash(round(x, 1), round(y, 1)))
    trunk_h = 3.2 + r.random() * 1.2
    mats["trim"].box(x, y, H, 0.45, 0.45, trunk_h + 1.0, r.random(), TRUNK)
    b = mats["trim"]
    for k in range(4):
        cx = x + r.uniform(-1.2, 1.2)
        cy = y + r.uniform(-1.2, 1.2)
        cz = H + trunk_h + 1.8 + r.uniform(0.0, 2.5)
        rad = 2.2 + r.random() * 1.2
        _blob(b, cx, cy, cz, rad, r.choice(LEAVES))


def _blob(b, cx, cy, cz, r, col):
    """A low-poly canopy blob: an octahedron subdivided once and pushed out to a sphere."""
    v = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
    faces = [(0, 2, 4), (2, 1, 4), (1, 3, 4), (3, 0, 4), (2, 0, 5), (1, 2, 5), (3, 1, 5), (0, 3, 5)]

    def norm(p):
        L = math.sqrt(p[0] ** 2 + p[1] ** 2 + p[2] ** 2)
        return (p[0] / L, p[1] / L, p[2] / L)

    def mid(a, c):
        return norm(((a[0] + c[0]) / 2, (a[1] + c[1]) / 2, (a[2] + c[2]) / 2))

    def P(p):
        return (cx + p[0] * r, cy + p[1] * r, cz + p[2] * r * 0.78)

    tris = [(v[i], v[j], v[k]) for i, j, k in faces]
    for _ in range(2):
        nxt = []
        for a, c, d in tris:
            ab, cd, da = mid(a, c), mid(c, d), mid(d, a)
            nxt += [(a, ab, da), (ab, c, cd), (da, cd, d), (ab, cd, da)]
        tris = nxt
    for t in tris:
        # Smooth shading: each vertex's own normal, slightly varied so the canopy is not a perfect ball.
        b.tri(P(t[0]), P(t[1]), P(t[2]), col, (0, 0), (1, 0), (0, 1),
              norm(((t[0][0] + t[1][0] + t[2][0]), (t[0][1] + t[1][1] + t[2][1]), (t[0][2] + t[1][2] + t[2][2]))))


def _barrier(mats, bar, kerb):
    pts = bar["pts"]
    kind = bar["kind"]
    if kind == "hedge":
        h, mat, col = 1.4, "grass", (0.6, 0.8, 0.55, 1.0)
    elif kind in ("wall", "retaining_wall"):
        h, mat, col = bar["height"] or 1.6, "brick_stock", (1, 1, 1, 1)
    else:
        h, mat, col = bar["height"] or 1.5, "railing", WHITE
    drew = False
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        if kerb.on_road(ax, ay) or kerb.on_road(bx, by) or kerb.distance(ax, ay) < 0.6 or kerb.distance(bx, by) < 0.6:
            continue
        L = math.hypot(bx - ax, by - ay)
        if L < 0.2:
            continue
        if mat == "railing":
            mats["railing"].quad((ax, ay, H), (bx, by, H), (bx, by, H + h), (ax, ay, H + h), WHITE, ((0, 0), (L, 0), (L, 1), (0, 1)))
        else:
            nx, ny = (by - ay) / L * 0.15, -(bx - ax) / L * 0.15
            mats[mat].wall((ax - nx, ay - ny), (bx - nx, by - ny), H, H + h, col)
            mats[mat].wall((bx + nx, by + ny), (ax + nx, ay + ny), H, H + h, col)
            mats[mat].hpoly([(ax - nx, ay - ny), (bx - nx, by - ny), (bx + nx, by + ny), (ax + nx, ay + ny)], H + h, col)
        drew = True
    return drew


def _clear_spot(x, y, kerb, footprints, gap=0.7):
    if kerb.on_road(x, y) or kerb.distance(x, y) < 0.45:
        return False
    for b in footprints:
        ring = b["outer"]
        if abs(ring[0][0] - x) > 200 or abs(ring[0][1] - y) > 200:
            continue
        if point_in_polygon(x, y, ring):
            return False
    return True


def _furniture(mats, m, kerb, footprints, rng, district):
    """Victorian-pattern lamp columns along the kerbs, bollards at corners, a K6 kiosk and a pillar box."""
    for i, (ax, ay, bx, by, nx, ny) in enumerate(m["kerb_lines"]):
        if i in kerb.boundary:
            continue
        L = math.hypot(bx - ax, by - ay)
        if L < 24.0:
            continue
        s = 10.0 + (i % 3) * 4.0
        while s < L - 8.0:
            x = ax + (bx - ax) * s / L - nx * 0.65
            y = ay + (by - ay) * s / L - ny * 0.65
            if _clear_spot(x, y, kerb, footprints):
                _lamp(mats, x, y, math.atan2(ny, nx))
            s += 30.0
    for cx, cy, r, mid, half in m["kerb_arcs"]:
        if r > 8.0 or r < 1.5:
            continue
        for a in (mid - 0.6 * half, mid + 0.6 * half):
            x, y = cx + (r - 0.5) * math.cos(a), cy + (r - 0.5) * math.sin(a)
            if _clear_spot(x, y, kerb, footprints):
                mats["dark"].box(x, y, H, 0.2, 0.2, 0.95, 0.0, LAMP)
    # One telephone kiosk and one pillar box, on the first roomy pavement corner near the first junction.
    placed = 0
    for cx, cy, r, mid, half in m["kerb_arcs"]:
        if placed or r < 3.5 or r > 8.0:
            continue
        x, y = cx + (r - 3.0) * math.cos(mid), cy + (r - 3.0) * math.sin(mid)
        x2, y2 = cx + (r - 3.0) * math.cos(mid + 0.9), cy + (r - 3.0) * math.sin(mid + 0.9)
        if _clear_spot(x, y, kerb, footprints) and _clear_spot(x2, y2, kerb, footprints) and kerb.distance(x, y) > 2.0:
            mats["trim"].box(x, y, H, 1.05, 1.05, 2.55, mid, RED)
            mats["trim"].box(x, y, H + 2.55, 1.15, 1.15, 0.12, mid, RED)
            mats["glass"].box(x, y, H + 1.0, 1.07, 1.07, 1.2, mid, (0.8, 0.8, 0.8, 1.0))
            _drum(mats, "trim", x2, y2, H, H + 1.35, 0.3, RED, sides=12)
            mats["trim"].hpoly([(x2 + 0.3 * math.cos(2 * math.pi * k / 12), y2 + 0.3 * math.sin(2 * math.pi * k / 12)) for k in range(12)], H + 1.35, RED)
            placed = 1


def _lamp(mats, x, y, yaw):
    """A Victorian-pattern lamp column: a fluted base, a slim shaft, a cross-arm and a lantern (decorative)."""
    b = mats["dark"]
    _drum(mats, "dark", x, y, H, H + 0.9, 0.16, LAMP, sides=8)
    _drum(mats, "dark", x, y, H + 0.9, H + 4.6, 0.075, LAMP, sides=8)
    b.box(x, y, H + 4.45, 0.7, 0.05, 0.05, yaw + 1.5708, LAMP)
    _drum(mats, "glass", x, y, H + 4.6, H + 5.15, 0.21, (1.0, 0.98, 0.88, 1.0), sides=6)
    _dome(mats, x, y, H + 5.15, 0.26, 0.22, LAMP, sides=6, rings=2)


def _markings(b, m):
    """White centre lines on two-way streets, double yellow lines along the kerbs, parking bays on Queen's Gate,
    give-way lines where a side street meets a main one. Along each street's straight lane length only."""
    info = m.get("street_info") or []
    lanes = m["lanes"]
    for (a, bb, ax, ay, bx, by), si in zip(m["streets"], info):
        L = math.hypot(bx - ax, by - ay)
        ux, uy = (bx - ax) / L, (by - ay) / L
        nx, ny = -uy, ux
        # Where lanes run: the extent of this street's lanes along it.
        ss = []
        for ln in lanes:
            for (px, py) in ((ln["x0"], ln["y0"]), (ln["x1"], ln["y1"])):
                s = (px - ax) * ux + (py - ay) * uy
                lat = (px - ax) * nx + (py - ay) * ny
                if -0.5 <= s <= L + 0.5 and abs(lat) <= si["half_width"] + si["half_width_right"] and seg_distance(px, py, ax, ay, bx, by) < 8.0:
                    ss.append(s)
        if not ss:
            continue
        s0, s1 = min(ss) + 1.0, max(ss) - 1.0
        two_way = sum(1 for ln in lanes if seg_distance(0.5 * (ln["x0"] + ln["x1"]), 0.5 * (ln["y0"] + ln["y1"]), ax, ay, bx, by) < 8.0) >= 2
        if two_way and si["median"] <= 0.0:
            s = s0
            while s + 3.0 <= s1:
                _strip(b, ax, ay, ux, uy, nx, ny, s, s + 3.0, 0.0, 0.10, MARK_WHITE)
                s += 9.0
        for side, w in ((1, si["half_width"]), (-1, si["half_width_right"])):
            if si["median"] > 0.0:  # Queen's Gate: parking bays along the kerb and along the reservation
                for edge, off in ((side * w, side * (w - 2.0)), (side * 0.5 * si["median"], side * (0.5 * si["median"] + 2.0))):
                    _strip(b, ax, ay, ux, uy, nx, ny, s0 + 4.0, s1 - 4.0, off, 0.10, MARK_WHITE)
                    s = s0 + 4.0
                    while s <= s1 - 4.0:
                        lo, hi = sorted((edge, off))
                        _cross(b, ax, ay, ux, uy, nx, ny, s, lo, hi, 0.10, MARK_WHITE)
                        s += 5.4
            else:
                for off in (side * (w - 0.30), side * (w - 0.52)):
                    _strip(b, ax, ay, ux, uy, nx, ny, s0, s1, off, 0.10, MARK_YELLOW)


def _cross(b, ax, ay, ux, uy, nx, ny, s, lo, hi, width, col, z=0.006):
    p = lambda ss, o: (ax + ux * ss + nx * o, ay + uy * ss + ny * o, z)  # noqa: E731
    b.quad(p(s - 0.5 * width, lo), p(s + 0.5 * width, lo), p(s + 0.5 * width, hi), p(s - 0.5 * width, hi), col)


def _strip(b, ax, ay, ux, uy, nx, ny, s0, s1, off, width, col, z=0.006):
    p = lambda s, o: (ax + ux * s + nx * o, ay + uy * s + ny * o, z)  # noqa: E731
    b.quad(p(s0, off - 0.5 * width), p(s1, off - 0.5 * width), p(s1, off + 0.5 * width), p(s0, off + 0.5 * width), col)


def _sign(parent, x, y, z, yaw, word, col):
    tn = TextNode("sign")
    tn.setText(word)
    tn.setAlign(TextNode.ACenter)
    lum = 0.3 * col[0] + 0.59 * col[1] + 0.11 * col[2]
    tn.setTextColor(0.08, 0.08, 0.08, 1) if lum > 0.6 else tn.setTextColor(0.95, 0.92, 0.82, 1)
    np_ = parent.attachNewNode(tn)
    np_.setScale(0.42)
    np_.setPos(x, y, z - 0.14)
    np_.setH(math.degrees(yaw) + 90.0)  # text reads facing the wall's outward normal
    np_.setLightOff(1)


def _name_plates(parent, m, kerb, footprints):
    """A street name plate near each junction arm, on a low frame at the back of the pavement, facing the street
    (black capitals on white, as the boroughs' plates are)."""
    info = m.get("street_info") or []
    done = set()
    for j, jn in enumerate(m["junctions"]):
        for (a, b, ax, ay, bx, by), si in zip(m["streets"], info):
            if j not in (a, b) or (j, si["name"]) in done:
                continue
            ox, oy = (bx, by) if a == j else (ax, ay)
            L = math.hypot(ox - jn["x"], oy - jn["y"])
            if L < 30.0:
                continue
            ux, uy = (ox - jn["x"]) / L, (oy - jn["y"]) / L
            lx, ly = -uy, ux  # left of the arm, seen from the junction
            # The arm's left kerb: the street's left if the junction is its a end, else its right.
            w = (si["half_width"] if a == j else si["half_width_right"]) + 1.3
            px, py = jn["x"] + ux * 17.0 + lx * w, jn["y"] + uy * 17.0 + ly * w
            if not _clear_spot(px, py, kerb, footprints) or kerb.distance(px, py) < 1.0:
                continue
            done.add((j, si["name"]))
            frame = Builder("plate")
            vx, vy = -lx, -ly  # facing the street
            heading = math.atan2(vx, -vy)
            for e in (-1.05, 1.05):
                frame.box(px + ux * e, py + uy * e, H, 0.07, 0.07, 1.15, math.atan2(uy, ux), (0.08, 0.08, 0.08, 1.0))
            q = lambda u, z: (px + ux * u, py + uy * u, z)  # noqa: E731
            frame.quad(q(-1.0, H + 1.1), q(1.0, H + 1.1), q(1.0, H + 1.55), q(-1.0, H + 1.55), (0.97, 0.97, 0.95, 1.0))
            fn = parent.attachNewNode(frame.node())
            fn.setTwoSided(True)
            for back in (False, True):
                tn = TextNode("plate")
                tn.setText(si["name"].upper())
                tn.setAlign(TextNode.ACenter)
                tn.setTextColor(0.05, 0.05, 0.05, 1)
                tp = parent.attachNewNode(tn)
                tp.setScale(0.21)
                k = -0.02 if not back else 0.02
                tp.setPos(px + vx * -k, py + vy * -k, H + 1.25)
                tp.setH(math.degrees(heading) + (180.0 if back else 0.0))
                tp.setLightOff(1)


def _sky(root):
    """A sky dome around the scene: haze at the horizon, a soft blue overhead (drawn first, never lit)."""
    sky = Builder("sky")
    R = 1800.0
    rows, cols = 8, 32
    ring = []
    for j in range(rows + 1):
        el = 0.5 * math.pi * j / rows
        z = R * math.sin(el) - 60.0
        r = R * math.cos(el)
        t = j / rows
        col = (0.78 - 0.30 * t, 0.83 - 0.22 * t, 0.88 - 0.10 * t, 1.0)
        ring.append([(r * math.cos(2 * math.pi * k / cols), r * math.sin(2 * math.pi * k / cols), z, col) for k in range(cols)])
    for j in range(rows):
        for k in range(cols):
            a, b = ring[j][k], ring[j][(k + 1) % cols]
            c, d = ring[j + 1][(k + 1) % cols], ring[j + 1][k]
            # Inward facing: seen from the centre.
            sky.tri(a[:3], d[:3], c[:3], a[3])
            sky.tri(a[:3], c[:3], b[:3], b[3])
    np_ = root.attachNewNode(sky.node())
    np_.setLightOff(1)
    np_.setShaderOff(1)
    np_.setFogOff(1)
    np_.setBin("background", 0)
    np_.setDepthWrite(False)
    np_.setCompass()
    np_.setName("sky")
