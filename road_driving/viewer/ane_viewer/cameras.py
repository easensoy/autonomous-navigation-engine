"""Chase, overhead, driver's-eye and free-orbit cameras around the true vehicle pose."""
import math

MODES = ("chase", "overhead", "driver", "orbit")


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Cameras:
    def __init__(self, base):
        self.base = base
        self.mode = "chase"
        self.pos = None
        self.yaw = None
        self.overhead_height = 75.0
        self.orbit_heading = math.radians(220.0)
        self.orbit_pitch = math.radians(34.0)
        self.orbit_dist = 19.0
        self.orbit_target = None   # None: follow the car
        self._drag = None
        self._pan = None

    def set_mode(self, mode, yaw=None):
        if mode == "orbit" and self.mode != "orbit" and yaw is not None:
            self.orbit_heading = yaw + math.radians(205.0)  # behind and to the right of the car
        self.mode = mode
        self.pos = None
        fov = 70.0 if mode == "driver" else 55.0
        self.base.camLens.setFov(fov)
        self.base.camLens.setNearFar(0.15 if mode == "driver" else 0.5, 3000.0)

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
                self.orbit_pitch = max(math.radians(3.0), min(math.radians(88.0), self.orbit_pitch + (m[1] - self._drag[1]) * 2.0))
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

    def update(self, dt, truth):
        cam = self.base.camera
        x, y, yaw = truth["x"], truth["y"], truth["yaw"]
        if self.mode == "chase":
            if self.yaw is None:
                self.yaw = yaw
            self.yaw += _wrap(yaw - self.yaw) * min(1.0, dt / 0.35)
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            want = (x - 10.0 * c, y - 10.0 * s, 3.8)
            if self.pos is None:
                self.pos = want
            k = min(1.0, dt / 0.15)
            self.pos = tuple(p + (w - p) * k for p, w in zip(self.pos, want))
            cam.setPos(*self.pos)
            cam.lookAt(x + 5.0 * c, y + 5.0 * s, 1.0)
        elif self.mode == "overhead":
            cam.setPos(x, y, self.overhead_height)
            cam.setHpr(0.0, -90.0, 0.0)
        elif self.mode == "driver":
            c, s = math.cos(yaw), math.sin(yaw)
            # Right-hand drive: the driver sits right of the centreline.
            ex, ey = x + 1.05 * c + 0.37 * s, y + 1.05 * s - 0.37 * c
            cam.setPos(ex, ey, 1.22)
            cam.setHpr(math.degrees(yaw) - 90.0, -4.0, 0.0)
        else:
            tx, ty = self.orbit_target if self.orbit_target is not None else (x, y)
            ch, sh = math.cos(self.orbit_heading), math.sin(self.orbit_heading)
            cp, sp = math.cos(self.orbit_pitch), math.sin(self.orbit_pitch)
            cam.setPos(tx + self.orbit_dist * cp * ch, ty + self.orbit_dist * cp * sh, 0.8 + self.orbit_dist * sp)
            cam.lookAt(tx, ty, 0.8)
