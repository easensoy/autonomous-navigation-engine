"""Panda3D scene graph checks without a window (skipped when Panda3D is not installed, as in CI)."""
import json
import pathlib
import sys
import unittest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

try:
    from panda3d.core import LPoint3f, NodePath, loadPrcFileData
    loadPrcFileData("", "window-type none")
    HAVE_PANDA = True
except ImportError:
    HAVE_PANDA = False


@unittest.skipUnless(HAVE_PANDA, "Panda3D is not installed")
class Scene(unittest.TestCase):
    def setUp(self):
        self.scene = json.loads((HERE / "fixtures" / "scene_r7.json").read_text())

    def test_world_builds_measured_and_decorative_layers(self):
        from ane_viewer import world
        root = world.build(self.scene, "medium")
        for name in ("measured", "decor", "zones", "sensed_objects"):
            self.assertFalse(root.find(name).isEmpty(), name)
        lo, hi = LPoint3f(), LPoint3f()
        root.find("measured").calcTightBounds(lo, hi)
        self.assertLess(lo[2], 0.001)  # the carriageway is the ground plane
        self.assertLess(hi[2], 0.2)    # nothing measured stands above the kerb

    def test_proxy_vehicle_node_matches_the_footprint(self):
        from ane_viewer.vehicle3d import Vehicle
        v = Vehicle(None, self.scene["vehicle"])
        lo, hi = LPoint3f(), LPoint3f()
        v.root.calcTightBounds(lo, hi)
        p = self.scene["vehicle"]
        self.assertAlmostEqual(lo[0], -p["rear_overhang"], delta=0.01)
        self.assertAlmostEqual(hi[0], p["wheelbase"] + p["front_overhang"], delta=0.05)  # the scanner face stands 3 cm proud
        self.assertAlmostEqual(hi[1] - lo[1], p["width_mirrors"], delta=0.01)
        v.set_pose(10.0, 20.0, 1.2, steer=0.3, v=5.0, dt=0.01)
        self.assertAlmostEqual(v.root.getX(), 10.0)
        self.assertAlmostEqual(v.root.getH(), 1.2 * 57.29577951308232, places=4)
        steer, _ = v.wheels["fl"]
        self.assertAlmostEqual(steer.getH(), 0.3 * 57.29577951308232, places=4)

    def test_axis_rotation_turns_gltf_axes_into_the_body_frame(self):
        from ane_viewer.vehicle3d import axis_rotation
        n = NodePath("m")
        n.setHpr(axis_rotation("-Z", "+Y"))  # a glTF model facing -Z with +Y up
        fwd = n.getMat().xformVec((0, 0, -1))
        up = n.getMat().xformVec((0, 1, 0))
        self.assertAlmostEqual(fwd[0], 1.0, places=5)
        self.assertAlmostEqual(up[2], 1.0, places=5)


if __name__ == "__main__":
    unittest.main()
