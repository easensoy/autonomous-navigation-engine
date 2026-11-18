"""What the camera must not enter or look through: the scenery's buildings and tree canopies as upright prisms.

Pure Python (no Panda3D), so camera placement can be tested anywhere. The world builders register a prism for
every building and canopy they draw, from the same footprints; the cameras then keep out of them, keep
a clear line of sight to the car, and choose another view when the shot they want is blocked. Nothing here
moves the car or reaches the simulation.
"""
import math


def point_in_polygon(x, y, pts):
    inside = False
    n = len(pts)
    j = n - 1
    for i in range(n):
        xi, yi = pts[i]
        xj, yj = pts[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi + 1e-300) + xi:
            inside = not inside
        j = i
    return inside


def _seg_dist(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    l2 = vx * vx + vy * vy
    t = 0.0 if l2 == 0.0 else max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / l2))
    return math.hypot(px - ax - t * vx, py - ay - t * vy)


class Prism:
    __slots__ = ("pts", "z0", "z1", "box", "kind")

    def __init__(self, pts, z0, z1, kind):
        self.pts = [(float(x), float(y)) for x, y in pts]
        self.z0, self.z1 = float(z0), float(z1)
        xs = [p[0] for p in self.pts]
        ys = [p[1] for p in self.pts]
        self.box = (min(xs), min(ys), max(xs), max(ys))
        self.kind = kind

    def distance_2d(self, x, y):
        """Distance from a point to the footprint (0 inside)."""
        if point_in_polygon(x, y, self.pts):
            return 0.0
        n = len(self.pts)
        return min(_seg_dist(x, y, *self.pts[i], *self.pts[(i + 1) % n]) for i in range(n))

    def contains(self, p, margin=0.0):
        x, y, z = p
        if z < self.z0 - margin or z > self.z1 + margin:
            return False
        b = self.box
        if x < b[0] - margin or x > b[2] + margin or y < b[1] - margin or y > b[3] + margin:
            return False
        return self.distance_2d(x, y) <= margin

    def first_hit(self, a, b):
        """The smallest t in [0, 1] at which the segment a -> b is inside the prism, or None."""
        ax, ay, az = a
        bx, by, bz = b
        dx, dy, dz = bx - ax, by - ay, bz - az
        # The z slab: t where z is within [z0, z1].
        if abs(dz) < 1e-12:
            if az < self.z0 or az > self.z1:
                return None
            tz0, tz1 = 0.0, 1.0
        else:
            ta, tb = (self.z0 - az) / dz, (self.z1 - az) / dz
            tz0, tz1 = max(0.0, min(ta, tb)), min(1.0, max(ta, tb))
            if tz0 > tz1:
                return None
        # The footprint: where the 2D segment crosses its edges, inside between alternate crossings.
        ts = []
        n = len(self.pts)
        for i in range(n):
            px, py = self.pts[i]
            qx, qy = self.pts[(i + 1) % n]
            ex, ey = qx - px, qy - py
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            wx, wy = px - ax, py - ay
            t = (wx * ey - wy * ex) / den
            s = (wx * dy - wy * dx) / den
            if 0.0 <= t <= 1.0 and 0.0 <= s < 1.0:
                ts.append(t)
        ts.sort()
        inside = point_in_polygon(ax, ay, self.pts)
        start = 0.0
        for t in ts + [1.0]:
            if inside:
                lo, hi = max(start, tz0), min(t, tz1)
                if lo <= hi:
                    return lo
            inside = not inside
            start = t
        return None


class Occluders:
    def __init__(self, cell=10.0):
        self.cell = cell
        self.prisms = []
        self.grid = {}
        # Street furniture, (x, y, kind): too slender to hide the car, so not prisms the chase and orbit cameras
        # steer round, but a street-level camera keeps clear of it (director.py).
        self.furniture = []

    def __len__(self):
        return len(self.prisms)

    def add(self, pts, top, base=0.0, kind="building"):
        if len(pts) < 3 or top <= base:
            return
        p = Prism(pts, base, top, kind)
        i = len(self.prisms)
        self.prisms.append(p)
        c = self.cell
        x0, y0, x1, y1 = p.box
        for gx in range(int(math.floor(x0 / c)), int(math.floor(x1 / c)) + 1):
            for gy in range(int(math.floor(y0 / c)), int(math.floor(y1 / c)) + 1):
                self.grid.setdefault((gx, gy), []).append(i)

    def add_box(self, cx, cy, yaw, sx, sy, z0, z1, kind="building"):
        c, s = math.cos(yaw), math.sin(yaw)
        hx, hy = 0.5 * sx, 0.5 * sy
        self.add([(cx + c * x - s * y, cy + s * x + c * y) for x, y in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))], z1, z0, kind)

    def add_disc(self, cx, cy, r, z0, z1, kind="tree", sides=8):
        self.add([(cx + r * math.cos(2 * math.pi * k / sides), cy + r * math.sin(2 * math.pi * k / sides)) for k in range(sides)], z1, z0, kind)

    def _near(self, x0, y0, x1, y1):
        c = self.cell
        seen = set()
        for gx in range(int(math.floor(min(x0, x1) / c)), int(math.floor(max(x0, x1) / c)) + 1):
            for gy in range(int(math.floor(min(y0, y1) / c)), int(math.floor(max(y0, y1) / c)) + 1):
                for i in self.grid.get((gx, gy), ()):
                    if i not in seen:
                        seen.add(i)
                        yield self.prisms[i]

    def inside(self, p, margin=0.0):
        """Is the point within `margin` of a prism (or in it)?"""
        x, y, _ = p
        return any(pr.contains(p, margin) for pr in self._near(x - margin, y - margin, x + margin, y + margin))

    def first_hit(self, a, b, kinds=None):
        """The smallest fraction along a -> b at which the segment enters a prism, or None."""
        best = None
        for pr in self._near(a[0], a[1], b[0], b[1]):
            if kinds and pr.kind not in kinds:
                continue
            bx = pr.box
            if max(a[0], b[0]) < bx[0] or min(a[0], b[0]) > bx[2] or max(a[1], b[1]) < bx[1] or min(a[1], b[1]) > bx[3]:
                continue
            t = pr.first_hit(a, b)
            if t is not None and (best is None or t < best):
                best = t
        return best

    def visible(self, cam, target):
        return self.first_hit(cam, target) is None


def car_points(x, y, yaw, length=4.68, rear=0.95, height=1.56):
    """Points on the car (rear-axle pose) a clear view must reach: body centre, roof, nose and tail."""
    c, s = math.cos(yaw), math.sin(yaw)
    mid = 0.5 * length - rear
    pts = [(mid, 0.0, 0.9), (mid, 0.0, height), (length - rear - 0.3, 0.0, 0.8), (-rear + 0.3, 0.0, 0.8)]
    return [(x + c * u - s * v, y + s * u + c * v, z) for u, v, z in pts]


def visible_share(occ, cam, x, y, yaw):
    pts = car_points(x, y, yaw)
    return sum(1 for p in pts if occ.visible(cam, p)) / float(len(pts))


class ViewSolver:
    """Places a camera that wants to look at the car from (heading, elevation, distance): pulled in along its
    ray when scenery is in the way, raised or swung round when pulling in is not enough, never inside a prism
    or below the minimum height. Returns the camera position and how it was found."""

    def __init__(self, occ, margin=0.7, min_height=1.2):
        self.occ = occ
        self.margin = margin
        self.min_height = min_height

    TRIES = [(0.0, 0.0), (0.0, 0.25), (0.0, 0.5)] + [(math.radians(a), e) for a in (15, -15, 30, -30, 50, -50, 75, -75, 105, -105, 140, -140, 180)
                                                     for e in (0.0, 0.35)] + [(0.0, 0.9)]

    def place(self, pivot, heading, elevation, dist, min_dist=4.0, car=None, prefer=(0.0, 0.0)):
        """The wanted view if it is clear (pulled in if need be); otherwise the cheapest clear alternative,
        trying first the one chosen last time so that the camera does not hunt between alternatives."""
        best = None
        for k, (dyaw, delev) in enumerate([prefer] + self.TRIES):
            e = min(math.radians(82.0), elevation + delev)
            cam, d = self._along(pivot, heading + dyaw, e, dist, min_dist)
            if cam is None:
                continue
            share = visible_share(self.occ, cam, *car) if car else 1.0
            # Pulling in a little is cheapest; a big pull-in (the car filling the frame) costs more than rising or turning.
            pull = dist - d
            cost = abs(dyaw) * 6.0 + delev * 8.0 + pull * 0.6 + max(0.0, pull - 4.0) * 1.2 + (1.0 - share) * 40.0
            if k == 0 and prefer != (0.0, 0.0):
                cost -= 1.0  # last frame's alternative wins near-ties, so the camera does not hunt between two
            if best is None or cost < best[0]:
                best = (cost, cam, d, dyaw, delev, share)
            # The wanted view, or last frame's choice, clear and not pulled in: take it without searching.
            if k <= 1 and share >= 0.99 and d >= dist - 1e-6 and (k == 0 or prefer == (0.0, 0.0)):
                break
            if k == 0 and share >= 0.99 and prefer != (0.0, 0.0) and pull <= 2.0:
                # Keep last frame's alternative while it needs little pulling in, unless the wanted view itself
                # has come clear. One that has to pull in further is weighed against every other alternative.
                cam0, d0 = self._along(pivot, heading, elevation, dist, min_dist)
                if cam0 is not None and d0 >= dist - 1e-6 and visible_share(self.occ, cam0, *car if car else (0, 0, 0)) >= 0.99:
                    best = (0.0, cam0, d0, 0.0, 0.0, 1.0)
                break
        if best is None:
            # Straight above the car: the view of last resort.
            cam = (pivot[0], pivot[1], pivot[2] + max(dist, 12.0))
            return cam, {"pulled": 0.0, "yaw": 0.0, "raise": math.pi / 2 - elevation, "share": 1.0, "fallback": True}
        _, cam, d, dyaw, delev, share = best
        return cam, {"pulled": dist - d, "yaw": dyaw, "raise": delev, "share": share, "fallback": False}

    def _along(self, pivot, heading, elevation, dist, min_dist):
        ce = math.cos(elevation)
        ux, uy, uz = ce * math.cos(heading), ce * math.sin(heading), math.sin(elevation)
        want = (pivot[0] + ux * dist, pivot[1] + uy * dist, pivot[2] + uz * dist)
        t = self.occ.first_hit(pivot, want)
        d = dist if t is None else max(0.0, t * dist - self.margin)
        # Keep clear of the surface of whatever stopped the ray, and above the ground.
        while d >= min_dist:
            cam = (pivot[0] + ux * d, pivot[1] + uy * d, max(self.min_height, pivot[2] + uz * d))
            if not self.occ.inside(cam, self.margin):
                return cam, d
            d -= 0.5
        return None, 0.0


def synthetic_occluders(m, quality="medium"):
    """The prisms of the synthetic grids' scenery, from the plots and trees world.py draws (geometry.frontage
    and geometry.furniture): each house up to its ridge, each canopy as an octagon."""
    from . import geometry as g
    occ = Occluders()
    plots = g.frontage(m)
    if quality == "low":
        plots = plots[::2]
    for p in plots:
        c, s = math.cos(p["heading"]), math.sin(p["heading"])
        d, w = p["depth"], p["width"]
        top = g.KERB_HEIGHT + 3.0 * p["storeys"] + 0.4 + 1.8 + 0.6 * (p["style"] % 2)
        occ.add_box(p["x"] - c * 0.5 * d, p["y"] - s * 0.5 * d, p["heading"], d, w, 0.0, top)
    if quality != "low":
        for kind, x, y, _ in g.furniture(m):
            if kind in ("lamp", "bollard"):
                occ.furniture.append((x, y, kind))
            if kind == "tree":
                occ.add_disc(x, y, 2.0, g.KERB_HEIGHT + 2.3, g.KERB_HEIGHT + 5.7)
    return occ
