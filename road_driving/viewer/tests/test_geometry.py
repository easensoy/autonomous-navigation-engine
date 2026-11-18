"""Scene geometry from the simulation's map (no Panda3D needed)."""
import json
import math
import pathlib
import sys
import unittest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from ane_viewer import geometry as g  # noqa: E402


def scene(name):
    return json.loads((HERE / "fixtures" / ("scene_%s.json" % name)).read_text())


class KerbLoops(unittest.TestCase):
    def test_grids_close_into_one_outer_kerb_and_one_loop_per_block(self):
        for name, blocks in (("r1", 4), ("r7", 12)):
            m = scene(name)["map"]
            outer, holes = g.kerb_loops(m)
            self.assertEqual(len(outer), 1, name)
            self.assertEqual(len(holes), blocks, name)
            self.assertGreater(g.signed_area(outer[0]), sum(-g.signed_area(h) for h in holes) * 0.05)

    def test_merging_keeps_every_kerb_line(self):
        m = scene("r7")["map"]
        merged = g.merged_kerb_lines(m)
        self.assertLess(len(merged), len(m["kerb_lines"]))
        for ax, ay, bx, by, nx, ny in m["kerb_lines"]:
            mx, my = 0.5 * (ax + bx), 0.5 * (ay + by)
            on = any(abs((mx - p) * q - (my - r) * s) < 1e-6 and min(p, u) - 1e-6 <= mx <= max(p, u) + 1e-6 and min(r, w) - 1e-6 <= my <= max(r, w) + 1e-6
                     for p, r, u, w, _, _ in merged for q, s in [((w - r) / (math.hypot(u - p, w - r) or 1), (u - p) / (math.hypot(u - p, w - r) or 1))])
            self.assertTrue(on, (ax, ay, bx, by))

    def test_lanes_are_on_the_carriageway(self):
        for name in ("r1", "r7"):
            m = scene(name)["map"]
            outer, holes = g.kerb_loops(m)
            for lane in m["lanes"]:
                for f in (0.25, 0.5, 0.75):
                    x = lane["x0"] + f * (lane["x1"] - lane["x0"])
                    y = lane["y0"] + f * (lane["y1"] - lane["y0"])
                    self.assertTrue(g.on_carriageway(x, y, outer, holes), (name, lane["name"], f))

    def test_decoration_stays_off_the_carriageway(self):
        for name in ("r1", "r7"):
            m = scene(name)["map"]
            outer, holes = g.kerb_loops(m)
            plots = g.frontage(m)
            self.assertGreater(len(plots), 100)
            for p in plots:
                c, s = math.cos(p["heading"]), math.sin(p["heading"])
                for u in (0.0, -p["depth"]):
                    for v in (-0.5 * p["width"], 0.5 * p["width"]):
                        x, y = p["x"] + c * u - s * v, p["y"] + s * u + c * v
                        self.assertFalse(g.on_carriageway(x, y, outer, holes), (name, x, y))
                        self.assertGreater(g.distance_to_kerb(x, y, m), 3.0)
            for kind, x, y, _ in g.furniture(m):
                self.assertFalse(g.on_carriageway(x, y, outer, holes), (name, kind, x, y))


class Overlays(unittest.TestCase):
    def test_covariance_ellipse_axes(self):
        pts = g.covariance_ellipse(4.0, 0.0, 1.0, 1.0)
        self.assertAlmostEqual(max(abs(x) for x, _ in pts), 2.0, places=6)
        self.assertAlmostEqual(max(abs(y) for _, y in pts), 1.0, places=2)
        rot = g.covariance_ellipse(2.5, 1.5, 2.5, 1.0)  # principal axes along the diagonals: variances 4 and 1
        self.assertAlmostEqual(max(math.hypot(x, y) for x, y in rot), 2.0, places=4)

    def test_scan_end_points_from_the_true_pose(self):
        scan = {"true_pose": [10.0, 5.0, math.pi / 2], "mount_x": 3.84, "first_bearing": -math.pi / 2, "bearing_step": math.pi / 2,
                "ranges": [2.0, -1, 4.0]}
        (sx, sy), pts = g.scan_points(scan)
        self.assertAlmostEqual(sx, 10.0)
        self.assertAlmostEqual(sy, 8.84)
        self.assertEqual(len(pts), 2)  # the missing return is skipped
        self.assertAlmostEqual(pts[0][0], 12.0)  # first beam: to the car's right (east, when heading north)
        self.assertAlmostEqual(pts[1][0], 6.0)  # last beam, 4 m: to its left (west)

    def test_footprint_is_the_backend_rectangle(self):
        fp = g.footprint(0.0, 0.0, 0.0, 4.682, 2.011, 0.842)
        xs, ys = [p[0] for p in fp], [p[1] for p in fp]
        self.assertAlmostEqual(min(xs), -0.842)
        self.assertAlmostEqual(max(xs), 4.682 - 0.842)
        self.assertAlmostEqual(max(ys) - min(ys), 2.011)


if __name__ == "__main__":
    unittest.main()
