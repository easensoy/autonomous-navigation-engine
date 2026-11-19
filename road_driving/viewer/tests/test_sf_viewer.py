"""The San Francisco scene and the Cybercab-inspired proxy, without Panda3D: the scenery's provenance, scenery off the
carriageway, the station frontage, no trees on the light-rail trackway, the overlay's street names and vehicle label,
the proxy against the backend's footprint, the map error, and the demonstrations."""
import json
import math
import pathlib
import sys
import unittest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from ane_viewer import demos  # noqa: E402
from ane_viewer import district_layout as dl  # noqa: E402
from ane_viewer import geometry as g  # noqa: E402
from ane_viewer import vehicle_mesh as vm  # noqa: E402
from ane_viewer.presentation import Streets, vehicle_text  # noqa: E402

SCENE = json.loads((HERE / "fixtures" / "scene_sf.json").read_text(encoding="utf-8"))


class Scenery(unittest.TestCase):
    def test_the_scenery_file_carries_its_source_frame_and_licence(self):
        data = dl.load("SF-SOMA")
        self.assertEqual(data["id"], "SF-SOMA")
        self.assertIn("ODbL", data["licence"])
        self.assertEqual(data["source"]["timestamp"], "2026-11-19T00:42:21Z")
        self.assertEqual(data["source"]["extract_sha256"], "595686579e8c438f9a33cafe44f53525c8f8925c0f198d89332a17d3b8d094c0")
        self.assertIn("WGS84", data["frame"])
        self.assertGreater(len(data["rail"]), 10)

    def test_no_building_stands_on_or_against_the_kerb_and_no_tree_near_it(self):
        plan = dl.layout(SCENE, "high")
        kerb = plan["kerb"]
        self.assertGreater(len(plan["buildings"]), 150)
        for item in plan["buildings"]:
            for x, y in dl.edge_samples(item["ring"]):
                self.assertFalse(kerb.on_road(x, y), item["b"]["osm"])
                self.assertGreaterEqual(kerb.distance(x, y), 0.5, item["b"]["osm"])
        for x, y, _ in plan["trees"]:
            self.assertFalse(kerb.on_road(x, y))
            self.assertGreater(kerb.distance(x, y), 0.8)

    def test_the_station_is_drawn_as_the_station_and_the_trackway_gets_no_trees(self):
        plan = dl.layout(SCENE, "high")
        styles = [item["style"] for item in plan["buildings"]]
        self.assertEqual(styles.count("station"), 1)
        self.assertTrue(dl.us_style("SF-SOMA") and not dl.us_style("KEN"))
        self.assertEqual([t for t in plan["trees"] if t[2] != "osm"], [])  # 4th Street's reservation is a light-rail trackway

    def test_every_lane_lies_on_the_carriageway(self):
        m = SCENE["map"]
        outer, holes = g.kerb_loops(m)
        for ln in m["lanes"]:
            for f in (0.0, 0.5, 1.0):
                x, y = ln["x0"] + (ln["x1"] - ln["x0"]) * f, ln["y0"] + (ln["y1"] - ln["y0"]) * f
                self.assertTrue(g.on_carriageway(x, y, outer, holes), ln["name"])


class Overlay(unittest.TestCase):
    def test_the_overlay_names_the_streets_and_the_vehicle(self):
        streets = Streets(SCENE["map"])
        self.assertEqual(streets.street("T4>T5/3"), "Townsend Street")
        self.assertEqual(streets.street("X5>B5/1"), "5th Street")
        text = vehicle_text(SCENE)
        self.assertIn("Cybercab-inspired proxy", text)
        self.assertIn("not a Tesla vehicle", text)
        self.assertNotIn("Waymo", text)
        self.assertNotIn("Jaguar", text)
        self.assertEqual(SCENE["map"]["district"]["traffic"], "right")

    def test_the_map_error_is_the_vehicles_map_beside_the_true_kerb(self):
        errs = SCENE["map_errors"]
        self.assertEqual(len(errs), 2)
        for e in errs:
            ax, ay, bx, by = e["map"]
            tx, ty, ux, uy = e["true"]
            self.assertAlmostEqual(abs(e["offset"]), 0.6, places=3)
            self.assertAlmostEqual(math.hypot(ax - tx, ay - ty), 0.6, delta=0.002)
            self.assertAlmostEqual(math.hypot(bx - ux, by - uy), 0.6, delta=0.002)
            self.assertGreater(math.hypot(bx - ax, by - ay), 100.0)


class CybercabProxy(unittest.TestCase):
    def setUp(self):
        self.p = dict(SCENE["vehicle"], antenna_x=SCENE["sensors"]["gnss"]["antenna_x"])
        self.parts, self.wheels, self.positions = vm.proxy(self.p)

    def test_the_body_matches_the_backends_footprint(self):
        (x0, y0, z0), (x1, y1, z1) = self.parts["body"].bounds()
        self.assertEqual(self.p["id"], "cybercab_proxy_v1")
        self.assertAlmostEqual(x0, -self.p["rear_overhang"], delta=0.005)
        self.assertAlmostEqual(x1, self.p["wheelbase"] + self.p["front_overhang"], delta=0.005)
        self.assertAlmostEqual(y1 - y0, self.p["width"], delta=0.005)
        self.assertAlmostEqual(z1, self.p["height"], delta=0.01)
        self.assertAlmostEqual(z0, 0.147, delta=0.01)  # Tesla's published ground clearance

    def test_no_mirrors_no_roof_rig_and_the_antenna_where_the_profile_mounts_it(self):
        self.assertEqual(self.parts["mirrors"].v, [])
        self.assertEqual(self.parts["roof_rig"].v, [])
        (ax0, _, _), (ax1, _, _) = self.parts["gnss_antenna"].bounds()
        self.assertAlmostEqual(0.5 * (ax0 + ax1), 1.30, delta=0.01)
        (sx0, _, _), (sx1, _, _) = self.parts["kerb_scanner"].bounds()
        self.assertAlmostEqual(sx1, self.p["wheelbase"] + self.p["front_overhang"] + 0.033, delta=0.01)

    def test_wheels_sit_on_the_axles_and_the_ground(self):
        self.assertEqual(self.positions["fl"][0], self.p["wheelbase"])
        for x, y, z in self.positions.values():
            self.assertAlmostEqual(z, self.p["wheel_radius"])
            self.assertLess(abs(y) + 0.12, 0.5 * self.p["width"])
        (_, _, z0), _ = self.wheels["fl"].bounds()
        self.assertAlmostEqual(z0, -self.p["wheel_radius"], delta=1e-6)

    def test_an_older_scene_message_still_draws_the_ipace_proxy(self):
        old = {k: v for k, v in json.loads((HERE / "fixtures" / "scene_r7.json").read_text())["vehicle"].items()}
        self.assertEqual(vm.design(old)["kind"], "ipace")
        self.assertGreater(len(vm.proxy(old)[0]["mirrors"].v), 0)


class Demonstrations(unittest.TestCase):
    def test_the_san_francisco_demonstrations(self):
        keys = {d.key: d for d in demos.DEMOS}
        for k in ("SF", "SFB", "SFD", "SFM"):
            d = keys[k]
            self.assertEqual(d.config["vehicle"], "cybercab_proxy_v1")
            self.assertIn(d.config["condition"], demos.CONDITIONS["SF"])
            self.assertEqual(demos.network_of(d.config["scenario"]), "SF")
            self.assertGreaterEqual(d.config["seed"], 901)  # development seeds: the scenes are not the evaluation
        self.assertEqual(demos.CONDITIONS["SF"], ["nominal_sf", "gnss_outage_sf", "gnss_drift_sf", "kerb_map_error_sf"])
        self.assertIn("SF-A", demos.ROUTES)
        self.assertIn("SF-B", demos.ROUTES)


if __name__ == "__main__":
    unittest.main()
