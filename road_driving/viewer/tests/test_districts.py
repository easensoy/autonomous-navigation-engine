"""The district scenes' layout, without Panda3D: the kerb drawn is the backend's, scenery stays off the
carriageway, the occluders match what is drawn, provenance is kept, and the overlay names the right streets."""
import json
import math
import pathlib
import sys
import unittest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from ane_viewer import district_layout as dl  # noqa: E402
from ane_viewer import geometry as g  # noqa: E402
from ane_viewer.presentation import PresentEvents, Streets  # noqa: E402

SCENES = ("scene_ken.json", "scene_knb.json")


def fixture(name):
    return json.loads((HERE / "fixtures" / name).read_text(encoding="utf-8"))


class SharedGeometry(unittest.TestCase):
    def test_the_kerb_drawn_is_the_backends_less_its_patch_edges(self):
        for name in SCENES:
            m = fixture(name)["map"]
            boundary = set(m["kerb_boundary"])
            self.assertGreater(len(boundary), 0)
            pieces = dl.kerb_pieces(m)
            lines = [p for p in pieces if len(p) == 2]
            want = {tuple(sorted(((round(a, 3), round(b, 3)), (round(c, 3), round(d, 3))))) for i, (a, b, c, d, _, _) in enumerate(m["kerb_lines"])
                    if i not in boundary}
            got = {tuple(sorted(((round(p[0][0], 3), round(p[0][1], 3)), (round(p[1][0], 3), round(p[1][1], 3))))) for p in lines}
            self.assertEqual(got, want, name)
            self.assertEqual(len(pieces) - len(lines), len(m["kerb_arcs"]), name)
            # Every drawn piece lies along a kerb the backend has (arcs as chords 6 degrees apart: within their sagitta).
            for piece in pieces:
                for (ax, ay), (bx, by) in zip(piece, piece[1:]):
                    mx, my = 0.5 * (ax + bx), 0.5 * (ay + by)
                    chord = math.hypot(bx - ax, by - ay)  # an arc's chord is off it by its sagitta, chord x tan(step / 4)
                    self.assertLess(g.distance_to_kerb(mx, my, m), 1e-6 if len(piece) == 2 else 1e-6 + chord * math.tan(g.ARC_STEP / 4.0))

    def test_the_carriageway_closes_through_its_patch_edges(self):
        m = fixture("scene_ken.json")["map"]
        outer, holes = g.kerb_loops(m)
        self.assertEqual(len(outer), 1)
        medians = sum(1 for si in m["street_info"] if si["median"] > 0)
        # Queen's Gate's central reservation, one island per block, and the city block that Queen's Gate,
        # Queen's Gate Terrace, Gore Street and Elvaston Place enclose.
        self.assertEqual(len(holes), medians + 1)
        m = fixture("scene_knb.json")["map"]
        outer, holes = g.kerb_loops(m)
        self.assertEqual((len(outer), len(holes)), (1, 0))

    def test_lanes_lie_on_the_carriageway(self):
        for name in SCENES:
            m = fixture(name)["map"]
            outer, holes = g.kerb_loops(m)
            for ln in m["lanes"]:
                for f in (0.0, 0.5, 1.0):
                    x = ln["x0"] + (ln["x1"] - ln["x0"]) * f
                    y = ln["y0"] + (ln["y1"] - ln["y0"]) * f
                    self.assertTrue(g.on_carriageway(x, y, outer, holes), (name, ln["name"]))
                    self.assertGreater(g.distance_to_kerb(x, y, m), 1.0, (name, ln["name"]))


class Scenery(unittest.TestCase):
    def setUp(self):
        self.plans = {name: dl.layout(fixture(name)) for name in SCENES}

    def test_buildings_stand_off_the_carriageway(self):
        for name, plan in self.plans.items():
            kerb = plan["kerb"]
            self.assertGreater(len(plan["buildings"]), 300)
            for item in plan["buildings"]:
                for x, y in dl.edge_samples(item["ring"], 1.0):
                    self.assertFalse(kerb.on_road(x, y), (name, item["b"]["osm"]))
                    self.assertGreater(kerb.distance(x, y), 0.5, (name, item["b"]["osm"]))

    def test_trees_and_parks_are_off_the_road(self):
        for name, plan in self.plans.items():
            kerb = plan["kerb"]
            for x, y, _ in plan["trees"]:
                self.assertFalse(kerb.on_road(x, y))
                self.assertGreater(kerb.distance(x, y), 0.8)
            for ring in plan["parks"]:
                for x, y in ring:
                    self.assertFalse(kerb.on_road(x, y))
                    self.assertGreaterEqual(kerb.distance(x, y), 2.0 - 1e-6)

    def test_occluders_are_the_drawn_buildings_and_canopies(self):
        for name, plan in self.plans.items():
            occ = plan["occluders"]
            self.assertEqual(sum(1 for p in occ.prisms if p.kind == "building"), len(plan["buildings"]))
            self.assertEqual(sum(1 for p in occ.prisms if p.kind == "tree"), len(plan["trees"]))
            for item, prism in zip(plan["buildings"], (p for p in occ.prisms if p.kind == "building")):
                self.assertEqual(len(prism.pts), len(item["ring"]))
                self.assertGreaterEqual(prism.z1, dl.H + item["height"])

    def test_landmarks_are_found_by_name(self):
        styles = {item["b"]["name"]: item["style"] for plan in self.plans.values() for item in plan["buildings"] if item["b"]["name"]}
        self.assertEqual(styles.get("Royal Albert Hall"), "rah")
        self.assertEqual(styles.get("Harrods"), "harrods")
        self.assertEqual(styles.get("Albert Hall Mansions (49-86)"), "mansion_block")
        stucco = sum(1 for item in self.plans["scene_ken.json"]["buildings"] if item["style"] == "stucco_terrace")
        self.assertGreater(stucco, 200)  # Kensington's stucco terraces

    def test_the_scenery_files_carry_their_licence_and_source(self):
        for district in ("KEN", "KNB"):
            data = dl.load(district)
            self.assertIn("ODbL", data["licence"])
            self.assertIn("OpenStreetMap contributors", data["licence"])
            self.assertTrue(data["source"]["timestamp"].startswith("2026-11-18"))
            self.assertIn("out body geom", data["source"]["query"])
            for b in data["buildings"]:
                self.assertTrue(b["osm"].startswith(("way/", "relation/")))
                self.assertIn(b["height_basis"].split()[0], ("osm", "unknown"))


class Overlay(unittest.TestCase):
    def test_street_names_follow_the_routes(self):
        st = Streets(fixture("scene_ken.json")["map"])
        self.assertEqual(st.street("KGE>KGW/2"), "Kensington Gore")
        self.assertEqual(st.street("QGN>QPC/4"), "Queen's Gate")
        self.assertEqual(st.street("QEP>EGS/1"), "Elvaston Place")
        self.assertEqual(st.street("QGN:KGW>QPC"), "Kensington Gore into Queen's Gate")
        self.assertEqual(st.street("QPC:QGN>QGT"), "Queen's Gate")
        st = Streets(fixture("scene_knb.json")["map"])
        self.assertEqual(st.street("BRE>BRG/0"), "Brompton Road")
        self.assertEqual(st.street("HRJ:BRM>HR1"), "Brompton Road into Hans Road")
        self.assertEqual(st.street("HBJ>HCB/3"), "Basil Street")

    def test_events_are_the_reported_transitions(self):
        class S:
            pass

        def tick(t, gnss, status, started=True):
            return {"t": t, "stack": {"started": started}, "est": {"gnss": gnss, "status": status, "kerb": "MATCHED"}}

        ev = PresentEvents()
        s = S()
        s.events = []
        for t, gnss, status, started in ((0.1, "-", "INIT", False), (0.2, "TRUSTED", "OK", False), (5.0, "TRUSTED", "OK", True),
                                         (30.0, "OUTAGE", "DEAD_RECKONING", True), (52.0, "TRUSTED", "OK", True)):
            s.tick = tick(t, gnss, status, started)
            ev.update(s)
        texts = [(t, text) for t, text, _ in ev.items]
        self.assertEqual(texts, [(30.0, "GNSS outage: no fixes"), (30.0, "dead reckoning on IMU, wheels and kerb scans"),
                                 (52.0, "GNSS restored and trusted"), (52.0, "integrity OK again")])


if __name__ == "__main__":
    unittest.main()
