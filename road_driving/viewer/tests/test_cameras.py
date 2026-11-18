"""Camera obstruction handling, without Panda3D: the chase, orbit and street-level cameras keep out of the
scenery and keep the car in view over whole recorded routes, junctions and turns included."""
import json
import math
import pathlib
import sys
import unittest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from ane_viewer import cameras as cm  # noqa: E402
from ane_viewer import district_layout as dl  # noqa: E402
from ane_viewer.director import Director  # noqa: E402
from ane_viewer.director import clear_foreground as dl_clear  # noqa: E402
from ane_viewer.occlusion import Occluders, ViewSolver, synthetic_occluders, visible_share  # noqa: E402


def fixture(name):
    return json.loads((HERE / "fixtures" / name).read_text(encoding="utf-8"))


class StubNP:
    def __init__(self):
        self.p = (0.0, 0.0, 0.0)

    def setPos(self, *a):
        self.p = tuple(float(v) for v in a)

    def lookAt(self, *a):
        pass

    def setHpr(self, *a):
        pass

    def getPos(self, other=None):
        return self.p


class StubLens:
    def setFov(self, *a):
        pass

    def setNearFar(self, *a):
        pass


class StubBase:
    def __init__(self):
        self.camera = StubNP()
        self.camLens = StubLens()
        self.render = None


def replay(occ, rows, mode, step=2, orbit_rate=0.0, dist=None):
    """The camera at every `step`th truth row (10 Hz rows): frames inside a prism, and the worst share of the
    car's points in view."""
    base = StubBase()
    cams = cm.Cameras(base)
    cams.set_occluders(occ)
    cams.set_mode(mode, rows[0][3])
    cams.orbit_rate = orbit_rate
    if dist:
        cams.orbit_dist = dist
    inside, worst, frames = 0, 1.0, 0
    for t, x, y, yaw in rows[::step]:
        cams.update(0.1 * step, {"x": x, "y": y, "yaw": yaw})
        p = base.camera.p
        inside += 1 if occ.inside(p, 0.0) else 0
        worst = min(worst, visible_share(occ, p, x, y, yaw))
        frames += 1
    return inside, worst, frames


class Obstruction(unittest.TestCase):
    def setUp(self):
        self.r7 = synthetic_occluders(fixture("scene_r7.json")["map"])
        self.r7_rows = fixture("truth_r7a_seed7.json")["rows"]

    def test_the_old_fixed_orbit_was_blocked_on_demo_c(self):
        """The defect reproduced: c3d65f5's orbit (fixed world heading, 19 m, 34 degrees, no line-of-sight check)."""
        blocked = 0
        for t, x, y, yaw in self.r7_rows:
            h, p, d = math.radians(220.0), math.radians(34.0), 19.0
            cam = (x + d * math.cos(p) * math.cos(h), y + d * math.cos(p) * math.sin(h), 0.8 + d * math.sin(p))
            blocked += 1 if (self.r7.inside(cam) or visible_share(self.r7, cam, x, y, yaw) < 0.5) else 0
        self.assertGreater(blocked / len(self.r7_rows), 0.3)

    def test_chase_and_orbit_stay_clear_on_demo_c(self):
        for mode in ("chase", "orbit"):
            inside, worst, frames = replay(self.r7, self.r7_rows, mode, step=1)
            self.assertEqual(inside, 0, mode)
            self.assertGreaterEqual(worst, 0.75, mode)
            self.assertGreater(frames, 700)

    def test_district_routes_keep_the_car_in_view(self):
        for scene, truth in (("scene_ken.json", "truth_ken_a.json"), ("scene_knb.json", "truth_knb_a.json")):
            occ = dl.layout(fixture(scene))["occluders"]
            rows = fixture(truth)["rows"]
            for mode, rate, dist in (("chase", 0.0, None), ("orbit", math.radians(-9.0), 15.0), ("orbit", 0.0, 25.0)):
                inside, worst, frames = replay(occ, rows, mode, step=1, orbit_rate=rate, dist=dist)
                self.assertEqual(inside, 0, (scene, mode, dist))
                self.assertGreaterEqual(worst, 0.75, (scene, mode, dist))

    def test_the_solver_pulls_in_before_a_wall_and_never_enters_it(self):
        occ = Occluders()
        occ.add([(5.0, -20.0), (6.0, -20.0), (6.0, 20.0), (5.0, 20.0)], 30.0)  # a wall 5 m behind the pivot
        cam, info = ViewSolver(occ).place((0.0, 0.0, 1.3), 0.0, math.radians(10.0), 12.0, min_dist=3.0, car=(0.0, 0.0, 0.0))
        self.assertFalse(occ.inside(cam, 0.6))
        self.assertLess(cam[0], 5.0)
        self.assertGreaterEqual(visible_share(occ, cam, 0.0, 0.0, 0.0), 0.99)
        # A wall too close to pull in to: the camera swings or rises instead, still clear.
        occ.add([(1.5, -20.0), (2.0, -20.0), (2.0, 20.0), (1.5, 20.0)], 30.0)
        cam, info = ViewSolver(occ).place((0.0, 0.0, 1.3), 0.0, math.radians(10.0), 12.0, min_dist=3.0, car=(0.0, 0.0, 0.0))
        self.assertFalse(occ.inside(cam, 0.6))
        self.assertTrue(info["yaw"] != 0.0 or info["raise"] > 0.0 or info["fallback"])

    def test_every_cut_starts_from_a_valid_view(self):
        occ = dl.layout(fixture("scene_ken.json"))["occluders"]
        rows = fixture("truth_ken_a.json")["rows"]
        base = StubBase()
        cams = cm.Cameras(base)
        cams.set_occluders(occ)
        for i in range(0, len(rows), 37):
            t, x, y, yaw = rows[i]
            for mode in ("chase", "orbit", "overhead"):
                cams.set_mode(mode, yaw)
                cams.update(0.02, {"x": x, "y": y, "yaw": yaw})
                p = base.camera.p
                self.assertFalse(occ.inside(p, 0.0), (t, mode))
                self.assertGreaterEqual(visible_share(occ, p, x, y, yaw), 0.75, (t, mode))

    def test_street_cameras_stand_on_the_pavement_with_a_view(self):
        scene = fixture("scene_ken.json")
        plan = dl.layout(scene)
        truth = fixture("truth_ken_a.json")
        path = truth["path_at_9s"]
        t0 = path["t"]
        row = min(truth["rows"], key=lambda r: abs(r[0] - t0))

        class App:
            kerb = plan["kerb"]
            cameras = None

        class State:
            pass

        st = State()
        st.path = path
        st.tick = {"t": row[0], "truth": {"x": row[1], "y": row[2], "yaw": row[3]}}
        pos = Director(App(), []).street_position(st, plan["occluders"])
        self.assertIsNotNone(pos)
        self.assertFalse(plan["kerb"].on_road(pos[0], pos[1]))
        self.assertFalse(plan["occluders"].inside(pos, 0.7))
        self.assertGreaterEqual(visible_share(plan["occluders"], pos, row[1], row[2], row[3]), 0.75)
        # A lamp column 6 m from that camera, in line with the car: the camera moves, and its new view of the
        # car's approach has no lamp in the near foreground.
        pts = path["pts"]
        cx, cy = row[1] - pos[0], row[2] - pos[1]
        d = math.hypot(cx, cy)
        lamp = (pos[0] + 6.0 * cx / d, pos[1] + 6.0 * cy / d, "lamp")
        plan["occluders"].furniture.append(lamp)
        moved = Director(App(), []).street_position(st, plan["occluders"])
        self.assertIsNotNone(moved)
        self.assertNotEqual(tuple(moved), tuple(pos))
        near = min(range(len(pts)), key=lambda i: math.hypot(pts[i][0] - moved[0], pts[i][1] - moved[1]))
        self.assertTrue(dl_clear(moved, pts[:near + 1], [lamp]))
        self.assertFalse(dl_clear(pos, pts[:near + 1], [lamp]))  # the first camera would have had it in view


if __name__ == "__main__":
    unittest.main()
