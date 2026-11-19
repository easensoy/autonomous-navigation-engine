"""Turn an OpenStreetMap extract (Overpass JSON) into a district scenery file for the 3D viewer.

    python district_extract.py KEN kensington.json ../viewer/districts/kensington.json
    python district_extract.py KNB knightsbridge.json ../viewer/districts/knightsbridge.json
    python district_extract.py SF-SOMA osm-extract-soma.json ../viewer/districts/san_francisco_soma.json
    python district_extract.py --check SF-SOMA osm-extract-soma.json   (the local frame against WGS84 geodesics)

The extract is fetched once with the Overpass query printed by `--query KEN` (or KNB, SF-SOMA). The London files
were made from the extracts of 2026-11-18T18:32:48Z, the San Francisco file from the extract of
2026-11-19T00:42:21Z; the extracts are attached to the releases that carry the showcase videos. The output is in
the district's local frame (metres east and north of its origin, the same frame as the backend's district
network in road_driving/src/district.cpp) and keeps only what the viewer draws: building footprints with their
tags, parks and grass, trees, walls, fences and railings, the mapped centrelines (for the provenance overlay)
and, in San Francisco, the rail and light-rail tracks and platforms. Nothing here reaches the simulation.

Frames. The London districts use an equirectangular scaling (111320 cos(lat0) and 110574 m per degree). San
Francisco uses the WGS84 tangent plane at its origin: the ellipsoid's radii of curvature at lat0 (M north, N east),
which reproduce geodesic distances across the patch to within millimetres (--check).

Licence: the output is a derivative of OpenStreetMap data, (c) OpenStreetMap contributors, and is available
under the Open Database License 1.0 (https://opendatacommons.org/licenses/odbl/1-0/).
"""
import hashlib
import json
import math
import sys

DISTRICTS = {
    # id: origin (as in district.cpp), the extract's bounding box (lat, lon) and the part kept (local metres)
    "KEN": {"title": "Kensington: Kensington Gore and Queen's Gate", "lat0": 51.4998, "lon0": -0.1788,
            "bbox": (51.4955, -0.1845, 51.5040, -0.1730), "keep": (-300.0, -330.0, 300.0, 330.0)},
    "KNB": {"title": "Knightsbridge: Brompton Road, Hans Road and Basil Street", "lat0": 51.4998, "lon0": -0.1640,
            "bbox": (51.4965, -0.1700, 51.5032, -0.1585), "keep": (-230.0, -260.0, 300.0, 280.0)},
    "SF-SOMA": {"title": "San Francisco, SoMa: Townsend, Brannan, 4th and 5th Streets", "lat0": 37.7770, "lon0": -122.3950,
                "bbox": (37.7700, -122.4060, 37.7830, -122.3870), "keep": (-470.0, -330.0, 150.0, 300.0), "frame": "wgs84",
                "rail": True},
}
KY = 110574.0
LEVEL_HEIGHT = 3.2  # m per storey where OSM gives levels but no height (an assumption, recorded per building)
WGS84_A, WGS84_E2 = 6378137.0, 0.00669437999014
# Pairs of mapped junction nodes whose geodesic length the --check compares with the local frame's.
CHECK_PAIRS = {"SF-SOMA": [(65325061, 65315481, "5th Street, Townsend to Brannan"), (65325061, 65334473, "Townsend Street, 5th to 4th"),
                           (65315481, 65315477, "Brannan Street, 5th to 4th"), (65315477, 65334473, "4th Street, Brannan to Townsend"),
                           (65293797, 65300049, "Townsend Street, 6th to 3rd")]}


def scales(cfg):
    """Metres per degree of longitude and latitude in the district's frame."""
    lat0 = cfg["lat0"]
    if cfg.get("frame") == "wgs84":
        s = math.sin(math.radians(lat0))
        w = math.sqrt(1.0 - WGS84_E2 * s * s)
        return WGS84_A / w * math.cos(math.radians(lat0)) * math.pi / 180.0, WGS84_A * (1.0 - WGS84_E2) / w ** 3 * math.pi / 180.0
    return 111320.0 * math.cos(math.radians(lat0)), KY


def query(district):
    b = "%.4f,%.4f,%.4f,%.4f" % DISTRICTS[district]["bbox"]
    parts = ['way["highway"]', 'way["building"]', 'relation["building"]', 'way["building:part"]', 'relation["type"="restriction"]',
             'way["leisure"="park"]', 'way["landuse"]', 'way["barrier"]', 'node["natural"="tree"]', 'way["area:highway"]']
    if DISTRICTS[district].get("rail"):
        parts += ['way["railway"]', 'way["public_transport"]', 'node["highway"~"traffic_signals|stop|crossing"]']
        return "[out:json][timeout:120];(" + "".join("%s(%s);" % (p, b) for p in parts) + ");out body geom;"
    return "[out:json][timeout:90];(" + "".join("%s(%s);" % (p, b) for p in parts) + ");out body geom;"


def vincenty(lat1, lon1, lat2, lon2):
    """Geodesic distance on the WGS84 ellipsoid (Vincenty's inverse formula), metres."""
    a, f = WGS84_A, 1.0 / 298.257223563
    b = a * (1.0 - f)
    L = math.radians(lon2 - lon1)
    U1, U2 = math.atan((1.0 - f) * math.tan(math.radians(lat1))), math.atan((1.0 - f) * math.tan(math.radians(lat2)))
    sU1, cU1, sU2, cU2 = math.sin(U1), math.cos(U1), math.sin(U2), math.cos(U2)
    lam = L
    for _ in range(200):
        sl, cl = math.sin(lam), math.cos(lam)
        ss = math.hypot(cU2 * sl, cU1 * sU2 - sU1 * cU2 * cl)
        cs = sU1 * sU2 + cU1 * cU2 * cl
        sig = math.atan2(ss, cs)
        sa = cU1 * cU2 * sl / ss
        c2a = 1.0 - sa * sa
        c2m = cs - 2.0 * sU1 * sU2 / c2a if c2a else 0.0
        C = f / 16.0 * c2a * (4.0 + f * (4.0 - 3.0 * c2a))
        prev, lam = lam, L + (1.0 - C) * f * sa * (sig + C * ss * (c2m + C * cs * (-1.0 + 2.0 * c2m * c2m)))
        if abs(lam - prev) < 1e-13:
            break
    u2 = c2a * (a * a - b * b) / (b * b)
    A = 1.0 + u2 / 16384.0 * (4096.0 + u2 * (-768.0 + u2 * (320.0 - 175.0 * u2)))
    B = u2 / 1024.0 * (256.0 + u2 * (-128.0 + u2 * (74.0 - 47.0 * u2)))
    ds = B * ss * (c2m + B / 4.0 * (cs * (-1.0 + 2.0 * c2m ** 2) - B / 6.0 * c2m * (-3.0 + 4.0 * ss ** 2) * (-3.0 + 4.0 * c2m ** 2)))
    return b * A * (sig - ds)


def check(district, data):
    """The local frame's distances between mapped junction nodes against their WGS84 geodesics."""
    cfg = DISTRICTS[district]
    kx, ky = scales(cfg)
    nodes = {}
    for e in data["elements"]:
        if e["type"] == "way":
            for nid, g in zip(e["nodes"], e.get("geometry", [])):
                nodes[nid] = (g["lat"], g["lon"])
        elif e["type"] == "node":
            nodes[e["id"]] = (e["lat"], e["lon"])
    worst = 0.0
    for a, b, what in CHECK_PAIRS.get(district, []):
        (la, oa), (lb, ob) = nodes[a], nodes[b]
        g = vincenty(la, oa, lb, ob)
        local = math.hypot((ob - oa) * kx, (lb - la) * ky)
        worst = max(worst, abs(local - g))
        print("%-34s nodes %d-%d: geodesic %.3f m, local frame %.3f m, difference %+.4f m" % (what, a, b, g, local, local - g))
    print("largest difference %.4f m" % worst)
    return 0 if worst < 0.01 else 1


def signed_area(pts):
    return 0.5 * sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1] for i in range(len(pts)))


def ccw(pts, want=True):
    return pts if (signed_area(pts) > 0) == want else list(reversed(pts))


def clean_ring(pts):
    out = []
    for p in pts:
        if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > 0.02:
            out.append(p)
    if len(out) > 1 and math.hypot(out[0][0] - out[-1][0], out[0][1] - out[-1][1]) <= 0.02:
        out.pop()
    return out if len(out) >= 3 and abs(signed_area(out)) > 0.5 else None


def join_rings(segments):
    """Outer or inner rings of a multipolygon from its member ways (which may be split)."""
    segs = [list(s) for s in segments if len(s) >= 2]
    rings = []
    while segs:
        ring = segs.pop(0)
        changed = True
        while changed and math.hypot(ring[0][0] - ring[-1][0], ring[0][1] - ring[-1][1]) > 0.05:
            changed = False
            for i, s in enumerate(segs):
                if math.hypot(s[0][0] - ring[-1][0], s[0][1] - ring[-1][1]) < 0.05:
                    ring += s[1:]
                elif math.hypot(s[-1][0] - ring[-1][0], s[-1][1] - ring[-1][1]) < 0.05:
                    ring += list(reversed(s))[1:]
                else:
                    continue
                segs.pop(i)
                changed = True
                break
        rings.append(ring)
    return rings


def clip_rect(pts, x0, y0, x1, y1):
    """Sutherland-Hodgman against the kept rectangle (for parks that reach beyond it)."""
    def clip(poly, inside, cut):
        out = []
        for i in range(len(poly)):
            a, b = poly[i - 1], poly[i]
            if inside(b):
                if not inside(a):
                    out.append(cut(a, b))
                out.append(b)
            elif inside(a):
                out.append(cut(a, b))
        return out

    def at_x(x):
        return lambda a, b: (x, a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0]))

    def at_y(y):
        return lambda a, b: (a[0] + (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]), y)

    for inside, cut in ((lambda p: p[0] >= x0, at_x(x0)), (lambda p: p[0] <= x1, at_x(x1)), (lambda p: p[1] >= y0, at_y(y0)), (lambda p: p[1] <= y1, at_y(y1))):
        pts = clip(pts, inside, cut)
        if not pts:
            return None
    return pts


def number(text):
    try:
        return float(str(text).split()[0].replace(",", "."))
    except (ValueError, IndexError):
        return None


def extract(district, data, raw_sha256=""):
    cfg = DISTRICTS[district]
    lat0, lon0 = cfg["lat0"], cfg["lon0"]
    kx, ky = scales(cfg)
    x0, y0, x1, y1 = cfg["keep"]

    def xy(p):
        return (round((p["lon"] - lon0) * kx, 2), round((p["lat"] - lat0) * ky, 2))

    def inside(pts):
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        return x0 <= cx <= x1 and y0 <= cy <= y1

    if cfg.get("frame") == "wgs84":
        frame = "x east, y north, metres; the WGS84 tangent plane at the origin: x = (lon - lon0) * %.3f, y = (lat - lat0) * %.3f" % (kx, ky)
    else:
        frame = "x east, y north, metres; x = (lon - lon0) * 111320 cos(lat0), y = (lat - lat0) * 110574"
    out = {"format": "ane-district/1", "id": district, "title": cfg["title"], "origin": [lat0, lon0],
           "frame": frame,
           "source": {"provider": "OpenStreetMap, via the Overpass API", "timestamp": data.get("osm3s", {}).get("timestamp_osm_base", ""),
                      "query": query(district)},
           "licence": "Open Database License 1.0 (ODbL); (c) OpenStreetMap contributors",
           "derived": "road_driving/tools/district_extract.py: projected to the local frame, rounded to 1 cm, clipped to the kept area",
           "bounds": [x0, y0, x1, y1], "level_height": LEVEL_HEIGHT,
           "buildings": [], "parks": [], "trees": [], "barriers": [], "ways": []}
    if raw_sha256:
        out["source"]["extract_sha256"] = raw_sha256
    if cfg.get("rail"):
        out["rail"] = []
    for e in data["elements"]:
        t = e.get("tags", {})
        if e["type"] == "way" and "geometry" in e:
            pts = [xy(p) for p in e["geometry"]]
            closed = e["nodes"][0] == e["nodes"][-1]
            kind = t.get("building") or t.get("building:part")
            if kind and closed:
                ring = clean_ring(pts)
                if ring and inside(ring):
                    out["buildings"].append(building(e, t, ccw(ring), [], "way"))
            elif t.get("leisure") in ("park", "garden") or t.get("landuse") in ("grass", "greenery", "recreation_ground", "flowerbed"):
                if closed:
                    ring = clean_ring(pts)
                    ring = clip_rect(ring, x0, y0, x1, y1) if ring else None
                    if ring and len(ring) >= 3:
                        out["parks"].append({"osm": "way/%d" % e["id"], "name": t.get("name", ""), "kind": t.get("leisure") or t.get("landuse"),
                                             "outer": [[round(x, 2), round(y, 2)] for x, y in ccw(ring)]})
            elif t.get("barrier") in ("wall", "fence", "railing", "hedge", "retaining_wall"):
                if any(x0 <= x <= x1 and y0 <= y <= y1 for x, y in pts):
                    out["barriers"].append({"osm": "way/%d" % e["id"], "kind": t["barrier"], "height": number(t.get("height")),
                                            "material": t.get("material", t.get("fence_type", "")), "pts": [list(p) for p in pts]})
            elif "highway" in t:
                if any(x0 <= x <= x1 and y0 <= y <= y1 for x, y in pts):
                    out["ways"].append({"osm": "way/%d" % e["id"], "name": t.get("name", ""), "highway": t["highway"], "oneway": t.get("oneway", ""),
                                        "area": t.get("area", "") == "yes", "pts": [list(p) for p in pts]})
            elif "rail" in out and t.get("railway") in ("rail", "light_rail", "platform") and t.get("tunnel", "") in ("", "no"):
                if any(x0 <= x <= x1 and y0 <= y <= y1 for x, y in pts) and t.get("layer", "0") in ("0", ""):
                    out["rail"].append({"osm": "way/%d" % e["id"], "railway": t["railway"], "name": t.get("name", ""), "closed": closed,
                                        "pts": [list(p) for p in pts]})
        elif e["type"] == "relation" and (t.get("building") or t.get("building:part")) and t.get("type") == "multipolygon":
            outer = [[xy(p) for p in m["geometry"]] for m in e.get("members", []) if m.get("role") == "outer" and "geometry" in m]
            inner = [[xy(p) for p in m["geometry"]] for m in e.get("members", []) if m.get("role") == "inner" and "geometry" in m]
            rings = [clean_ring(r) for r in join_rings(outer)]
            holes = [h for h in (clean_ring(r) for r in join_rings(inner)) if h]
            for ring in rings:
                if ring and inside(ring):
                    mine = [ccw(h, False) for h in holes if point_in(h[0], ring)]
                    out["buildings"].append(building(e, t, ccw(ring), mine, "relation"))
        elif e["type"] == "node" and t.get("natural") == "tree":
            p = xy(e)
            if x0 <= p[0] <= x1 and y0 <= p[1] <= y1:
                out["trees"].append([p[0], p[1]])
    out["buildings"].sort(key=lambda b: b["osm"])
    out["parks"].sort(key=lambda b: b["osm"])
    out["barriers"].sort(key=lambda b: b["osm"])
    out["ways"].sort(key=lambda b: b["osm"])
    out["trees"].sort()
    if "rail" in out:
        out["rail"].sort(key=lambda b: b["osm"])
    return out


def point_in(p, ring):
    x, y = p
    inside = False
    j = len(ring) - 1
    for i in range(len(ring)):
        xi, yi = ring[i]
        xj, yj = ring[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def building(e, t, ring, holes, kind):
    levels = number(t.get("building:levels"))
    height = number(t.get("height"))
    roof_levels = number(t.get("roof:levels"))
    b = {"osm": "%s/%d" % (kind, e["id"]), "name": t.get("name", ""), "type": t.get("building") or t.get("building:part"),
         "part": "building:part" in t and "building" not in t, "levels": levels, "height": height, "roof_levels": roof_levels,
         "min_level": number(t.get("building:min_level")), "roof": t.get("roof:shape", ""), "material": t.get("building:material", ""),
         "colour": t.get("building:colour", ""), "roof_colour": t.get("roof:colour", ""), "heritage": t.get("heritage", ""),
         "shop": t.get("shop", ""), "amenity": t.get("amenity", ""), "start_date": t.get("start_date", ""), "architect": t.get("architect", ""),
         "outer": [[round(x, 2), round(y, 2)] for x, y in ring], "inner": [[[round(x, 2), round(y, 2)] for x, y in h] for h in holes]}
    # How tall it is drawn, and on what basis: the mapped height, else the mapped storeys, else unknown.
    if height:
        b["drawn_height"], b["height_basis"] = height, "osm height"
    elif levels:
        b["drawn_height"], b["height_basis"] = round(levels * LEVEL_HEIGHT, 2), "osm building:levels x %.1f m" % LEVEL_HEIGHT
    else:
        b["drawn_height"], b["height_basis"] = None, "unknown (the viewer infers it from its neighbours and style)"
    return b


def main(argv):
    if len(argv) == 3 and argv[1] == "--query":
        print(query(argv[2]))
        return 0
    if len(argv) == 4 and argv[1] == "--check" and argv[2] in DISTRICTS:
        return check(argv[2], json.load(open(argv[3], encoding="utf-8")))
    if len(argv) != 4 or argv[1] not in DISTRICTS:
        print(__doc__)
        return 2
    raw = open(argv[2], "rb").read()
    data = json.loads(raw.decode("utf-8"))
    # The San Francisco file records the extract's SHA-256 (the London files predate the field).
    out = extract(argv[1], data, hashlib.sha256(raw).hexdigest() if DISTRICTS[argv[1]].get("frame") == "wgs84" else "")
    with open(argv[3], "w", encoding="utf-8", newline="\n") as f:
        json.dump(out, f, ensure_ascii=False, separators=(",", ":"))
        f.write("\n")
    print("%s: %d buildings, %d parks, %d trees, %d barriers, %d ways%s -> %s" % (
        argv[1], len(out["buildings"]), len(out["parks"]), len(out["trees"]), len(out["barriers"]), len(out["ways"]),
        ", %d rail ways" % len(out["rail"]) if "rail" in out else "", argv[3]))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
