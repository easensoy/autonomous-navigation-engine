"""What a district scene contains and where, in pure Python (no Panda3D), so it can be tested anywhere.

From the backend's map message (the kerb the simulation uses) and the district's OpenStreetMap scenery file:
which buildings are drawn, in which style, how tall and which of their walls face a modelled street; the
trees; the parks cut back from the carriageway; the kerb pieces that get an upstand (every kerb line and arc
except the patch edges); and the prisms the cameras keep out of. district_scene.py turns this into geometry.
"""
import json
import math
import pathlib
import random

from . import geometry as g
from .occlusion import Occluders, point_in_polygon

DISTRICTS = pathlib.Path(__file__).resolve().parents[1] / "districts"
FILES = {"KEN": "kensington.json", "KNB": "knightsbridge.json"}
H = g.KERB_HEIGHT
STOREY = {"stucco_terrace": 3.4, "mansion_block": 3.1, "brick_terrace": 3.1, "commercial": 3.6, "harrods": 4.2, "institutional": 3.8,
          "church": 4.0, "rah": 4.0}
DEFAULT_LEVELS = {"stucco_terrace": 5, "mansion_block": 6, "brick_terrace": 4, "commercial": 5, "harrods": 7, "institutional": 5, "church": 4, "rah": 6}
EXTRA_TOP = {"harrods": 8.0 + 6.0, "rah": 15.0}  # domes and turrets above the parapet (prism tops)


def load(district_id):
    return json.loads((DISTRICTS / FILES[district_id]).read_text(encoding="utf-8"))


def stable_hash(*vals):
    h = 2166136261
    for v in vals:
        for ch in str(v).encode():
            h = ((h ^ ch) * 16777619) & 0xFFFFFFFF
    return h


class Kerb:
    """The backend's kerb for quick queries: distance to the nearest kerb piece and the carriageway test."""

    def __init__(self, m):
        self.m = m
        self.outer, self.holes = g.kerb_loops(m)
        self.lines = m["kerb_lines"]
        self.arcs = m["kerb_arcs"]
        self.boundary = set(m.get("kerb_boundary") or [])

    def on_road(self, x, y):
        return g.on_carriageway(x, y, self.outer, self.holes)

    def distance(self, x, y):
        return g.distance_to_kerb(x, y, self.m)

    def clearance(self, x, y):
        """Signed distance: positive on the carriageway."""
        d = self.distance(x, y)
        return d if self.on_road(x, y) else -d


def kerb_pieces(m):
    """The kerb pieces that are drawn with an upstand, as polylines with the carriageway on their left: every
    kerb line and arc of the backend's map except its patch edges (where there is no kerb)."""
    boundary = set(m.get("kerb_boundary") or [])
    out = []
    for i, (ax, ay, bx, by, nx, ny) in enumerate(m["kerb_lines"]):
        if i in boundary:
            continue
        dx, dy = ny, -nx
        out.append([(ax, ay), (bx, by)] if (bx - ax) * dx + (by - ay) * dy >= 0 else [(bx, by), (ax, ay)])
    for cx, cy, r, mid, half in m["kerb_arcs"]:
        n = max(2, int(math.ceil(2.0 * half / g.ARC_STEP)))
        out.append([(cx + r * math.cos(mid + half - 2 * half * i / n), cy + r * math.sin(mid + half - 2 * half * i / n)) for i in range(n + 1)])
    return out


def classify(b, district, fronts):
    """The building's style, from its OSM tags and name and the streets it fronts."""
    name, kind = b["name"], b["type"]
    if name == "Royal Albert Hall":
        return "rah"
    if name == "Harrods":
        return "harrods"
    if kind in ("church", "cathedral", "chapel", "oratory") or b["amenity"] == "place_of_worship":
        return "church"
    if ("Mansions" in name or "Court" in name or "Royal College of Music" in name or "Alexandra" in name) and kind not in ("university",):
        return "mansion_block"
    if kind in ("university", "college", "school", "government", "civic", "office", "hospital", "parking") or (b["levels"] or 0) >= 9:
        return "institutional"
    if kind in ("retail", "commercial") or b["shop"] or (district == "KNB" and "Brompton Road" in fronts):
        return "commercial"
    if b["material"] == "brick":
        return "mansion_block"
    if district == "KEN":
        return "stucco_terrace"
    return "mansion_block" if stable_hash(b["osm"]) % 3 else "brick_terrace"


def edge_samples(ring, step=2.0):
    n = len(ring)
    for i in range(n):
        ax, ay = ring[i]
        bx, by = ring[(i + 1) % n]
        L = math.hypot(bx - ax, by - ay)
        k = max(1, int(L / step))
        for j in range(k):
            yield ax + (bx - ax) * j / k, ay + (by - ay) * j / k


def seg_distance(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    l2 = vx * vx + vy * vy
    t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / l2)) if l2 else 0.0
    return math.hypot(px - ax - t * vx, py - ay - t * vy)


def fronts(ring, kerb, streets):
    """For each edge of a footprint (anticlockwise): (distance to the carriageway in front, the pavement depth at
    its middle, the street's name) if it faces a modelled street within 30 m, else None."""
    out = []
    n = len(ring)
    for i in range(n):
        ax, ay = ring[i]
        bx, by = ring[(i + 1) % n]
        L = math.hypot(bx - ax, by - ay)
        if L < 2.5:
            out.append(None)
            continue
        nx, ny = (by - ay) / L, -(bx - ax) / L  # outward
        mx, my = 0.5 * (ax + bx), 0.5 * (ay + by)
        hit = None
        for d in (1.0, 2.0, 3.0, 4.5, 6.0, 8.0, 11.0, 15.0, 20.0, 26.0, 30.0):
            if kerb.on_road(mx + nx * d, my + ny * d):
                hit = d
                break
        if hit is None:
            out.append(None)
            continue
        px, py = mx + nx * hit, my + ny * hit
        best = min(streets, key=lambda s: seg_distance(px, py, s[1][2], s[1][3], s[1][4], s[1][5])) if streets else ("", None)
        out.append((hit, kerb.distance(mx, my), best[0]))
    return out


def clear_of_road(ring, kerb, gap):
    """A park outline with every part nearer than `gap` to the carriageway cut off: clipped with the half-plane
    behind the nearest kerb piece (a line, or an arc's tangent; never a patch edge) while a vertex is too close."""
    pts = [tuple(p) for p in ring]
    for _ in range(16):
        bad = None
        for x, y in pts:
            if kerb.on_road(x, y) or kerb.distance(x, y) < gap:
                bad = (x, y)
                break
        if bad is None:
            return pts if len(pts) >= 3 and abs(g.signed_area(pts)) > 4.0 else None
        best = None
        for i, (ax, ay, bx, by, nx, ny) in enumerate(kerb.lines):
            if i in kerb.boundary:
                continue
            vx, vy = bx - ax, by - ay
            l2 = vx * vx + vy * vy
            t = max(0.0, min(1.0, ((bad[0] - ax) * vx + (bad[1] - ay) * vy) / l2)) if l2 else 0.0
            d = math.hypot(bad[0] - ax - t * vx, bad[1] - ay - t * vy)
            if best is None or d < best[0]:
                best = (d, ax + t * vx, ay + t * vy, nx, ny)
        for cx, cy, r, mid, half in kerb.arcs:
            th = math.atan2(bad[1] - cy, bad[0] - cx)
            th = mid + max(-half, min(half, (th - mid + math.pi) % (2 * math.pi) - math.pi))
            qx, qy = cx + r * math.cos(th), cy + r * math.sin(th)
            d = math.hypot(bad[0] - qx, bad[1] - qy)
            if best is None or d < best[0]:
                best = (d, qx, qy, math.cos(th), math.sin(th))
        if best is None:
            return None
        _, ax, ay, nx, ny = best
        keep = lambda p: (p[0] - ax) * -nx + (p[1] - ay) * -ny >= gap  # noqa: E731
        out = []
        for i in range(len(pts)):
            a, b = pts[i - 1], pts[i]
            ka, kb = keep(a), keep(b)
            if kb:
                if not ka:
                    out.append(_cut(a, b, ax, ay, nx, ny, gap))
                out.append(b)
            elif ka:
                out.append(_cut(a, b, ax, ay, nx, ny, gap))
        pts = out
        if len(pts) < 3:
            return None
    return None


def _cut(a, b, ax, ay, nx, ny, gap):
    fa = (a[0] - ax) * -nx + (a[1] - ay) * -ny - gap
    fb = (b[0] - ax) * -nx + (b[1] - ay) * -ny - gap
    t = fa / (fa - fb)
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def decorative_trees(m, data, kerb, rng, quality):
    """London planes along Queen's Gate's central reservation, and park trees where OpenStreetMap maps few."""
    out = []
    if quality == "low":
        return out
    for (a, b, ax, ay, bx, by), si in zip(m["streets"], m.get("street_info") or []):
        if si["median"] > 3.0:
            L = math.hypot(bx - ax, by - ay)
            s = 22.0
            while s < L - 22.0:
                out.append((ax + (bx - ax) * s / L, ay + (by - ay) * s / L, "decor"))
                s += 24.0
    have = [(x, y) for x, y in data["trees"]]
    for park in data["parks"]:
        if park["kind"] != "park":
            continue
        ring = park["outer"]
        xs = [p[0] for p in ring]
        ys = [p[1] for p in ring]
        step = 19.0
        y = min(ys) + 6.0
        while y < max(ys):
            x = min(xs) + 6.0 + (rng.random() * 6.0)
            while x < max(xs):
                jx, jy = x + rng.uniform(-4, 4), y + rng.uniform(-4, 4)
                if (point_in_polygon(jx, jy, ring) and kerb.distance(jx, jy) > 9.0 and not kerb.on_road(jx, jy)
                        and all(math.hypot(jx - hx, jy - hy) > 9.0 for hx, hy in have) and abs(jx) < 260 and abs(jy) < 300):
                    out.append((jx, jy, "decor"))
                    have.append((jx, jy))
                x += step
            y += step
    return out


def layout(scene_msg, quality="medium"):
    """Everything the district scene places, and the occluders that match it."""
    m = scene_msg["map"]
    info = m["district"]
    data = load(info["id"])
    kerb = Kerb(m)
    rng = random.Random(info["id"])
    streets = [(si["name"], s) for si, s in zip(m.get("street_info") or [], m["streets"])]
    occ = Occluders()
    buildings, dropped = [], []
    for b in data["buildings"]:
        if b["part"]:
            continue
        ring = b["outer"]
        if any(kerb.on_road(x, y) or kerb.distance(x, y) < 0.5 for x, y in edge_samples(ring)):
            dropped.append(b["osm"])  # it would stand on or against the modelled kerb: not drawn
            continue
        fr = fronts(ring, kerb, streets)
        style = classify(b, info["id"], {f[2] for f in fr if f})
        levels = b["levels"] or DEFAULT_LEVELS[style]
        height = b["height"] or levels * data["level_height"]
        top = H + height + EXTRA_TOP.get(style, 0.6)
        if style == "rah":
            top = H + 26.0 + EXTRA_TOP["rah"]
        occ.add(ring, top, 0.0, "building")
        buildings.append({"b": b, "ring": ring, "style": style, "levels": levels, "height": height, "fronts": fr, "top": top})
    trees = [(x, y, "osm") for x, y in data["trees"] if kerb.distance(x, y) > 0.8 and not kerb.on_road(x, y)]
    trees += decorative_trees(m, data, kerb, rng, quality)
    for x, y, _ in trees:
        occ.add_disc(x, y, 3.0, H + 3.0, H + 11.0, "tree")
    parks = []
    for park in data["parks"]:
        ring = clear_of_road(park["outer"], kerb, 2.0)
        if ring:
            parks.append(ring)
    return {"data": data, "kerb": kerb, "rng": rng, "buildings": buildings, "dropped": dropped, "trees": trees, "parks": parks, "occluders": occ,
            "kerb_pieces": kerb_pieces(m)}
