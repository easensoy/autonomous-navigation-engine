"""The proxy vehicle and the imported-model alignment match the backend's footprint (no Panda3D needed)."""
import json
import math
import pathlib
import sys
import unittest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from ane_viewer import vehicle_mesh as vm  # noqa: E402

PARAMS = json.loads((HERE / "fixtures" / "scene_r7.json").read_text())["vehicle"]


class Proxy(unittest.TestCase):
    def setUp(self):
        self.parts, self.wheels, self.positions = vm.proxy(PARAMS)

    def test_body_matches_the_footprint(self):
        (x0, y0, z0), (x1, y1, z1) = self.parts["body"].bounds()
        self.assertAlmostEqual(x0, -PARAMS["rear_overhang"], delta=0.005)
        self.assertAlmostEqual(x1, PARAMS["wheelbase"] + PARAMS["front_overhang"], delta=0.005)
        self.assertAlmostEqual(y1 - y0, PARAMS["width"], delta=0.005)
        self.assertAlmostEqual(z1, vm.HEIGHT, delta=0.01)
        self.assertGreater(z0, 0.1)  # ground clearance

    def test_mirrors_span_the_clearance_width(self):
        (_, y0, _), (_, y1, _) = self.parts["mirrors"].bounds()
        self.assertAlmostEqual(y1 - y0, PARAMS["width_mirrors"], delta=0.005)

    def test_wheels_sit_on_the_axles_and_the_ground(self):
        self.assertEqual(self.positions["rl"][0], 0.0)
        self.assertEqual(self.positions["fl"][0], PARAMS["wheelbase"])
        for x, y, z in self.positions.values():
            self.assertAlmostEqual(z, vm.WHEEL_RADIUS)
            self.assertLess(abs(y) + 0.5 * vm.WHEEL_WIDTH, 0.5 * PARAMS["width"])
        (_, _, z0), (_, _, z1) = self.wheels["fl"].bounds()
        self.assertAlmostEqual(z0, -vm.WHEEL_RADIUS, delta=1e-6)

    def test_scanner_is_at_the_simulated_mount(self):
        (x0, _, _), (x1, _, _) = self.parts["kerb_scanner"].bounds()
        mount = json.loads((HERE / "fixtures" / "scene_r7.json").read_text())["sensors"]["kerb"]["mount_x"]
        self.assertLess(abs(0.5 * (x0 + x1) - mount), 0.05)

    def test_body_faces_point_outwards(self):
        body = self.parts["body"]
        normals = body.normals()
        outward = 0
        sides = 0
        for (x, y, z), n in zip(body.v, normals):
            if abs(y) > 0.8 and 0.5 < z < 0.9:  # door skins
                sides += 1
                outward += 1 if n[1] * y > 0 else 0
        self.assertGreater(sides, 50)
        self.assertGreaterEqual(outward, 0.99 * sides)  # all but the loft's seam vertices

    def test_meshes_are_well_formed(self):
        for mesh in list(self.parts.values()) + list(self.wheels.values()):
            n = len(mesh.v)
            self.assertEqual(len(mesh.c), n, mesh.name)
            for tri in mesh.t:
                self.assertTrue(all(0 <= i < n for i in tri), mesh.name)


class Alignment(unittest.TestCase):
    def test_a_model_is_scaled_and_placed_on_the_rear_axle_frame(self):
        from ane_viewer.align import alignment
        # A model twice the size, offset and not centred.
        bmin, bmax = (10.0, -3.0, 2.0), (10.0 + 2 * 4.682, -3.0 + 2 * 2.011, 2.0 + 3.13)
        scale, offset, report = alignment(bmin, bmax, PARAMS, {"fit_length": True})
        self.assertAlmostEqual(scale, 0.5)
        self.assertAlmostEqual(scale * bmin[0] + offset[0], -PARAMS["rear_overhang"])
        self.assertAlmostEqual(scale * 0.5 * (bmin[1] + bmax[1]) + offset[1], 0.0)
        self.assertAlmostEqual(scale * bmin[2] + offset[2], 0.0)
        self.assertAlmostEqual(report["length_error_m"], 0.0, places=3)
        self.assertAlmostEqual(report["width_error_m"], 0.0, places=3)


if __name__ == "__main__":
    unittest.main()
