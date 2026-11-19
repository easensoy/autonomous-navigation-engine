"""How far each district street's modelled centreline departs from the OpenStreetMap ways it follows.

    python district_deviation.py KEN kensington_overpass.json
    python district_deviation.py KNB knightsbridge_overpass.json
    python district_deviation.py SF-SOMA osm-extract-soma.json --scene sf-soma.scene.json

Reads the district definitions from road_driving/src/district.cpp (junction positions and each street's
"OSM way(s) ..." source) and the Overpass extract they were taken from. For every OSM node of a street's ways
that lies along the modelled street (its projection falls on it), the distance to the modelled centreline;
for Queen's Gate, whose two carriageways are modelled about their midline, the distance of each carriageway
from the midline. Prints a Markdown table (PROVENANCE.md).

San Francisco's junctions are computed in district.cpp (each street's line continued to its patch edges), so they
are read from the backend's scene map instead (`road_drive map SF-SOMA --scene FILE`), and its frame is the WGS84
tangent plane (district_extract.py). 4th Street's two carriageways, modelled about the light-rail trackway's
centre, are reported like Queen's Gate's.
"""
import json
import math
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
SRC = HERE.parent / "src" / "district.cpp"
ORIGIN = {"KEN": (51.4998, -0.1788), "KNB": (51.4998, -0.1640), "SF-SOMA": (37.7770, -122.3950)}
TWO_CARRIAGEWAYS = ("Queen's Gate", "4th Street")


def scales(district):
    if district == "SF-SOMA":
        sys.path.insert(0, str(HERE))
        from district_extract import DISTRICTS, scales as frame_scales
        return frame_scales(DISTRICTS[district])
    return 111320.0 * math.cos(math.radians(ORIGIN[district][0])), 110574.0


def district_block(text, district):
    fn = {"KEN": "districtKensington", "KNB": "districtKnightsbridge", "SF-SOMA": "districtSanFranciscoSoma"}[district]
    start = text.index("DistrictSpec %s()" % fn)
    end = text.index("\n}\n", start)
    return text[start:end]


def junctions(block):
    out = {}
    for name, x, y in re.findall(r'\{"(\w+)", (-?[\d.]+), (-?[\d.]+), [\d.]+, -?[\d.]+, (?:true|false),\s*"', block):
        out[name] = (float(x), float(y))
    if "onQG" in block:  # Queen's Gate's junctions on its midline
        qx, qy, ux, uy = (float(v) for v in re.search(r"qx = (-?[\d.]+), qy = (-?[\d.]+), ux = (-?[\d.]+), uy = (-?[\d.]+)", block).groups())
        for var, s in re.findall(r"(\w+) = onQG\(([\d.]+)\)", block.replace("const auto ", "").replace(", ", "\n")):
            pass
        for name, var in (("QPC", "qpc"), ("QGT", "qgt"), ("QEP", "qep"), ("QGS", "qgs")):
            s = float(re.search(var + r" = onQG\(([\d.]+)\)", block).group(1))
            out[name] = (qx + ux * s, qy + uy * s)
    return out


def streets(block):
    out = []
    for m in re.finditer(r'\{"([^"]+)", "(\w+)", "(\w+)", (-?\d), ([\d.]+), ([\d.]+), ([\d.]+),\s*"([^"]*)"', block):
        out.append({"name": m.group(1), "a": m.group(2), "b": m.group(3), "source": m.group(8)})
    if "qgWays" in block:
        ways = re.findall(r'"(OSM ways [^"]+)"', block[block.index("qgWays"):block.index("qgEnds")])
        ends = re.findall(r'\{"(\w+)", "(\w+)"\}', block[block.index("qgEnds"):])
        for (a, b), src in zip(ends, ways):
            out.append({"name": "Queen's Gate", "a": a, "b": b, "source": src})
    return out


def main(district, extract, scene=None):
    text = SRC.read_text(encoding="utf-8")
    block = district_block(text, district)
    if scene:
        # The scene map lists the streets in district.cpp's order, each as (junction index, junction index, end points),
        # index -1 at a patch edge, which is not a junction of the map: its position is the street's end point.
        m = json.load(open(scene, encoding="utf-8"))
        m = m.get("map", m)
        names = [j["name"] for j in m["junctions"]]
        rows = streets(block)
        if len(rows) != len(m["streets"]):
            sys.exit("the scene map and district.cpp list different streets: rebuild road_drive and write the scene again")
        js = {}
        for st, (a, b, ax, ay, bx, by) in zip(rows, m["streets"]):
            for end, index, x, y in ((st["a"], a, ax, ay), (st["b"], b, bx, by)):
                if index >= 0 and names[index] != end:
                    sys.exit("%s %s-%s: the scene map has junction %s there" % (st["name"], st["a"], st["b"], names[index]))
                js.setdefault(end, (x, y))
    else:
        js = junctions(block)
    lat0, lon0 = ORIGIN[district]
    kx, ky = scales(district)
    data = json.load(open(extract, encoding="utf-8"))
    ways = {e["id"]: e for e in data["elements"] if e["type"] == "way"}
    print("| Street | Modelled from to | OSM ways | Largest distance of the mapped centreline from the model |")
    print("|---|---|---|---|")
    for st in streets(block):
        ids = [int(i) for i in re.findall(r"\d{5,}", st["source"])]
        (ax, ay), (bx, by) = js[st["a"]], js[st["b"]]
        L = math.hypot(bx - ax, by - ay)
        ux, uy = (bx - ax) / L, (by - ay) / L
        worst, sides = 0.0, []
        for wid in ids:
            w = ways.get(wid)
            if not w:
                continue
            for p in w["geometry"]:
                x, y = (p["lon"] - lon0) * kx, (p["lat"] - lat0) * ky
                s = (x - ax) * ux + (y - ay) * uy
                if s < 3.0 or s > L - 3.0:
                    continue
                d = (x - ax) * -uy + (y - ay) * ux
                if st["name"] in TWO_CARRIAGEWAYS:
                    sides.append(abs(d))
                else:
                    worst = max(worst, abs(d))
        if st["name"] in TWO_CARRIAGEWAYS:
            cell = ("the two carriageways run %.1f to %.1f m either side of the modelled midline" % (min(sides), max(sides)) if sides
                    else "mapped nodes only at its junctions")
        else:
            cell = "%.1f m" % worst
        print("| %s | %s-%s | %s | %s |" % (st["name"], st["a"], st["b"], ", ".join(str(i) for i in ids), cell))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[4] if len(sys.argv) > 4 and sys.argv[3] == "--scene" else None)
