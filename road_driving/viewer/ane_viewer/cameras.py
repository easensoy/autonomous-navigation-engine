"""Chase, overhead, driver's-eye, orbit and street-level cameras around the true vehicle pose.

The chase and orbit cameras keep out of the scenery (occlusion.Occluders, registered by the world builders
from the same footprints they draw): each frame the wanted view is checked from the car outwards; when a
building or a tree canopy is in the way the camera is pulled in along its ray at once and eases back out
slowly, and when pulling in is not enough it is raised or swung round to the nearest clear view. It never
goes below 1.2 m or inside a prism, and a cut to another camera starts from its solved, valid pose. The car's
position is only ever read here.
"""
import math

from .occlusion import Occluders, ViewSolver, visible_share

MODES = ("chase", "overhead", "driver", "orbit", "street")
PIVOT_AHEAD, PIVOT_UP = 1.4, 1.3  # the point the cameras look at: over the middle of the car


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Cameras:
    def __init__(self, base):
        self.base = base
        self.mode = "chase"
        self.occ = Occluders()
        self.solver = ViewSolver(self.occ)
        self.pos = None
        self.yaw = None
        self.overhead_height = 75.0
        self.overhead_heading_up = False
        self.orbit_heading = math.radians(220.0)
        self.orbit_pitch = math.radians(34.0)
        self.orbit_dist = 19.0
        self.orbit_rate = 0.0       # rad/s: a scripted orbit turns round the car on its own
        self.orbit_target = None    # None: follow the car
        self.chase_dist, self.chase_height = 10.0, 3.8
        self.street_pos = None      # a fixed camera on the pavement that turns to follow the car
        self.street_fov = 50.0
        self._dist = None           # the chase / orbit camera's current distance (pulled in at once, eased out)
        self._off = None            # the orbit camera's current offset from its target
        self._prefer = (0.0, 0.0)
        self._drag = None
        self._pan = None
        self.info = {}

    def set_occluders(self, occ):
        self.occ = occ if occ is not None else Occluders()
        self.solver = ViewSolver(self.occ)
        self.pos = None
        self._dist = None
        self._off = None

    def set_mode(self, mode, yaw=None):
        if mode == "orbit" and self.mode != "orbit" and yaw is not None:
            self.orbit_heading = yaw + math.radians(205.0)  # behind and to the right of the car
        self.mode = mode
        self.pos = None  # a cut: the next frame starts from this camera's solved view, not from the last one
        self._dist = None
        self._off = None
        self._prefer = (0.0, 0.0)
        fov = 70.0 if mode == "driver" else (self.street_fov if mode == "street" else 55.0)
        self.base.camLens.setFov(fov)
        self.base.camLens.setNearFar(0.15 if mode == "driver" else 0.3, 3000.0)

    def zoom(self, factor):
        if self.mode == "overhead":
            self.overhead_height = max(12.0, min(400.0, self.overhead_height * factor))
        elif self.mode == "orbit":
            self.orbit_dist = max(4.0, min(400.0, self.orbit_dist * factor))

    def mouse(self, right_down, middle_down):
        mw = self.base.mouseWatcherNode
        if not mw.hasMouse():
            self._drag = self._pan = None
            return
        m = (mw.getMouseX(), mw.getMouseY())
        if self.mode == "orbit" and right_down:
            if self._drag is not None:
                self.orbit_heading -= (m[0] - self._drag[0]) * 3.0
                self.orbit_pitch = max(math.radians(8.0), min(math.radians(80.0), self.orbit_pitch + (m[1] - self._drag[1]) * 2.0))
            self._drag = m
        else:
            self._drag = None
        if self.mode == "orbit" and middle_down:
            if self._pan is not None and self.orbit_target is not None:
                k = self.orbit_dist * 0.8
                dx, dy = (m[0] - self._pan[0]) * k, (m[1] - self._pan[1]) * k
                c, s = math.cos(self.orbit_heading), math.sin(self.orbit_heading)
                self.orbit_target = (self.orbit_target[0] - c * dx + s * dy, self.orbit_target[1] - s * dx - c * dy)
            self._pan = m
        else:
            self._pan = None

    def detach(self, x, y):
        self.orbit_target = (x, y) if self.orbit_target is None else None

    # ---- Placement --------------------------------------------------------------------------------
    def _solve(self, pivot, heading, elevation, dist, min_dist, car, dt):
        """The obstruction-free camera for the wanted view, with its distance pulled in at once and eased out."""
        cam, info = self.solver.place(pivot, heading, elevation, dist, min_dist=min_dist, car=car, prefer=self._prefer)
        self._prefer = (info["yaw"], info["raise"])
        d = math.dist(cam, pivot)
        if self._dist is not None and d > self._dist + 1e-3:
            # Ease back out along the same ray (2 m/s), as long as every point on the way is clear.
            grow = min(d, self._dist + 2.0 * max(dt, 1.0 / 60.0))
            ray = [(c - p) / d for c, p in zip(cam, pivot)]
            near = tuple(p + r * grow for p, r in zip(pivot, ray))
            if not self.occ.inside(near, self.solver.margin) and self.occ.visible(near, pivot):
                cam, d = near, grow
        self._dist = d
        info["distance"] = d
        return cam, info

    def update(self, dt, truth):
        cam = self.base.camera
        x, y, yaw = truth["x"], truth["y"], truth["yaw"]
        c, s = math.cos(yaw), math.sin(yaw)
        pivot = (x + PIVOT_AHEAD * c, y + PIVOT_AHEAD * s, PIVOT_UP)
        car = (x, y, yaw)
        self.info = {"mode": self.mode}
        if self.mode == "chase":
            if self.yaw is None or self.pos is None:
                self.yaw = yaw
            self.yaw += _wrap(yaw - self.yaw) * min(1.0, dt / 0.35)
            back = self.chase_dist
            rise = self.chase_height - PIVOT_UP
            want, info = self._solve(pivot, self.yaw + math.pi, math.atan2(rise, back), math.hypot(back, rise), 3.5, car, dt)
            if self.pos is None:
                self.pos = want
            else:
                k = min(1.0, dt / 0.15)
                eased = tuple(p + (w - p) * k for p, w in zip(self.pos, want))
                # Smoothing must not carry the camera into the scenery or behind it: then it cuts to the solved view.
                ok = not self.occ.inside(eased, 0.3) and visible_share(self.occ, eased, *car) >= 0.75
                self.pos = eased if ok else want
            cam.setPos(*self.pos)
            cam.lookAt(x + 5.0 * c, y + 5.0 * s, 1.0)
            self.info.update(info)
        elif self.mode == "overhead":
            cam.setPos(x, y, self.overhead_height)
            cam.setHpr(math.degrees(yaw) - 90.0 if self.overhead_heading_up else 0.0, -90.0, 0.0)
        elif self.mode == "driver":
            # Right-hand drive: the driver sits right of the centreline.
            ex, ey = x + 1.05 * c + 0.37 * s, y + 1.05 * s - 0.37 * c
            cam.setPos(ex, ey, 1.22)
            cam.setHpr(math.degrees(yaw) - 90.0, -4.0, 0.0)
        elif self.mode == "street" and self.street_pos is not None:
            cam.setPos(*self.street_pos)
            cam.lookAt(pivot[0], pivot[1], 1.1)
            self.info["share"] = visible_share(self.occ, self.street_pos, *car)
        else:
            self.orbit_heading += self.orbit_rate * dt
            tx, ty = self.orbit_target if self.orbit_target is not None else (pivot[0], pivot[1])
            want, info = self._solve((tx, ty, 0.8), self.orbit_heading, self.orbit_pitch, self.orbit_dist, min(7.0, self.orbit_dist), car, dt)
            target = (tx, ty, 0.8)
            off = tuple(w - p for w, p in zip(want, target))
            if self.pos is not None and self._off is not None:
                # A change of alternative glides over a quarter of a second (the offset from the car is eased,
                # so the camera does not trail a moving car), as long as the eased pose is clear and sees the
                # car; otherwise it cuts to the solved view.
                k = min(1.0, dt / 0.25)
                eased = tuple(o + (w - o) * k for o, w in zip(self._off, off))
                p = tuple(t + o for t, o in zip(target, eased))
                if not self.occ.inside(p, 0.3) and visible_share(self.occ, p, *car) >= 0.75:
                    off = eased
            self._off = off
            self.pos = tuple(t + o for t, o in zip(target, off))
            cam.setPos(*self.pos)
            cam.lookAt(tx, ty, 0.8)
            self.info.update(info)
        p = cam.getPos(self.base.render)
        self.info["pos"] = (round(p[0], 2), round(p[1], 2), round(p[2], 2))
        self.info["inside"] = self.mode in ("chase", "orbit", "street") and self.occ.inside((p[0], p[1], p[2]), 0.0)
        if "share" not in self.info:
            self.info["share"] = 1.0 if self.mode in ("overhead", "driver") else visible_share(self.occ, (p[0], p[1], p[2]), *car)
