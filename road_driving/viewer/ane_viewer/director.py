"""A camera sequence for showcase recordings: chase, street-level, orbit and overhead shots at set simulated
times, each cut to from a view that is checked first.

A shot starts from its own solved pose (cameras.py keeps chase and orbit views clear of the scenery). A
street-level shot is placed on the pavement ahead of the car, beside its planned path (the stack's own path
message), at eye height, outside every building and canopy and with a clear view of the path the car is
about to drive; if no such place exists the director keeps the chase camera. An orbit sweeps round the side
of the car where the scenery leaves more room along the path ahead (in left-hand traffic usually the right,
across the carriageway). While a street or orbit shot runs, the director cuts back to the chase camera if
the car leaves the view. It only ever reads the car.
"""
import math

from .occlusion import visible_share

PLANS = {
    # Simulated times (s) of the cuts, for the two district showcase runs; any other run gets the default.
    "KEN-A": [(0.0, "chase"), (9.0, "street"), (17.0, "chase_low"), (24.0, "orbit"), (33.0, "chase"), (40.0, "overhead"), (49.0, "chase_low"),
              (56.0, "street"), (64.0, "orbit"), (73.0, "chase"), (80.0, "overhead"), (86.0, "chase")],
    # Along the department store's frontage the camera stays at street level: from overhead, its dome and
    # cupolas would stand close under the camera and fill the frame.
    "KNB-A": [(0.0, "chase"), (8.0, "orbit"), (17.0, "street"), (25.0, "chase_low"), (36.5, "street"), (58.0, "orbit"),
              (67.0, "chase_low"), (74.0, "overhead"), (82.0, "chase")],
    "default": [(0.0, "chase"), (15.0, "orbit"), (30.0, "chase"), (45.0, "overhead"), (60.0, "chase")],
}


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clear_foreground(cam, pts, furniture, near=12.0, half=0.5, keep=11.0):
    """True if no lamp column, kiosk or pillar box stands within `near` metres inside the arc a street camera at
    `cam` sweeps while it follows the car along `pts` (until the car comes within `keep` metres, when the director
    cuts away), widened by `half` radians, about half the lens's width; and no furniture at all within 2.5 m."""
    seen = [p for p in pts if math.hypot(p[0] - cam[0], p[1] - cam[1]) >= keep]
    if not seen:
        return True
    ref = math.atan2(seen[0][1] - cam[1], seen[0][0] - cam[0])
    rel = [_wrap(math.atan2(p[1] - cam[1], p[0] - cam[0]) - ref) for p in seen]
    lo, hi = min(rel) - half, max(rel) + half
    for fx, fy, kind in furniture:
        d = math.hypot(fx - cam[0], fy - cam[1])
        if d < 2.5:
            return False
        if kind in ("lamp", "kiosk", "pillar_box") and d < near and lo <= _wrap(math.atan2(fy - cam[1], fx - cam[0]) - ref) <= hi:
            return False
    return True


class Director:
    def __init__(self, app, plan):
        self.app = app
        self.plan = list(plan)
        self.current = None
        self.since = 0.0
        self.lost = 0.0
        self.log = []  # (t, shot, note)

    def update(self, state, dt):
        tk = state.tick
        if tk is None:
            return
        t = tk["t"]
        while self.plan and t >= self.plan[0][0]:
            _, shot, *opt = self.plan.pop(0)
            self._cut(state, shot, t, height=opt[0] if opt else None)
        self.since += dt
        cams = self.app.cameras
        share = cams.info.get("share", 1.0)
        if self.current in ("street", "orbit"):
            tr = tk["truth"]
            d = math.hypot(cams.street_pos[0] - tr["x"], cams.street_pos[1] - tr["y"]) if cams.mode == "street" and cams.street_pos else None
            if d is not None and d < 11.0:
                self._cut(state, "chase_low", t, note="the car reached the street camera")
                return
            far = d is not None and d > 55.0
            self.lost = self.lost + dt if share < 0.5 else 0.0
            if far or self.lost > 0.25:
                self._cut(state, "chase", t, note="car left the view" if not far else "car passed the street camera")

    def _cut(self, state, shot, t, note="", height=None):
        cams = self.app.cameras
        tr = state.tick["truth"]
        cams.orbit_rate = 0.0
        cams.overhead_heading_up = False
        cams.chase_dist, cams.chase_height = 10.0, 3.8
        if shot == "street":
            pos = self.street_position(state, cams.occ)
            if pos is None:
                shot, note = "chase", "no clear street-level view: kept the chase camera"
            else:
                cams.street_pos = pos
                cams.set_mode("street")
        if shot == "chase_low":
            cams.set_mode("chase")
            cams.chase_dist, cams.chase_height = 7.5, 2.4
        elif shot == "chase":
            cams.set_mode("chase")
        elif shot == "orbit":
            side = self.orbit_side(state, cams.occ)
            cams.set_mode("orbit", tr["yaw"])
            # From behind the car on that side, sweeping round towards its front at 9 degrees a second.
            cams.orbit_heading = tr["yaw"] + side * math.radians(150.0)
            cams.orbit_pitch = math.radians(24.0)
            cams.orbit_dist = 15.0
            cams.orbit_rate = -side * math.radians(9.0)
            note = note or ("orbit on the car's %s" % ("left" if side > 0 else "right"))
        elif shot == "overhead":
            cams.set_mode("overhead")
            cams.overhead_height = height or 62.0
            cams.overhead_heading_up = True
        self.current = shot
        self.since = 0.0
        self.lost = 0.0
        self.log.append((round(t, 2), shot, note))

    def orbit_side(self, state, occ, dist=15.0, pitch=math.radians(24.0), duration=9.0):
        """+1 to orbit on the car's left, -1 on its right: the side on which the orbit's rays, cast from where
        the car will be as the orbit turns (along the planned path), run further before meeting scenery."""
        tr = state.tick["truth"]
        path = (state.path or {}).get("pts") or []
        speed = max(0.0, tr.get("v", 0.0))
        room = {1: 0.0, -1: 0.0}
        for step in range(4):
            tau = duration * step / 3.0
            x, y, yaw = tr["x"], tr["y"], tr["yaw"]
            if path and speed > 0.1:
                # The path point about speed * tau ahead, and the heading there.
                i0 = min(range(min(len(path), 20)), key=lambda i: math.hypot(path[i][0] - x, path[i][1] - y))
                k, run = i0, 0.0
                while k + 1 < len(path) and run < speed * tau:
                    run += math.hypot(path[k + 1][0] - path[k][0], path[k + 1][1] - path[k][1])
                    k += 1
                a, b = path[max(0, k - 1)], path[min(len(path) - 1, k + 1)]
                if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-6:
                    yaw = math.atan2(b[1] - a[1], b[0] - a[0])
                x, y = path[k][0], path[k][1]
            rel = math.radians(150.0 - 9.0 * tau)  # where the orbit will be by then, relative to the heading
            pivot = (x + 1.4 * math.cos(yaw), y + 1.4 * math.sin(yaw), 0.8)
            for side in (1, -1):
                h = yaw + side * rel
                ce = math.cos(pitch)
                want = (pivot[0] + ce * math.cos(h) * dist, pivot[1] + ce * math.sin(h) * dist, pivot[2] + math.sin(pitch) * dist)
                hit = occ.first_hit(pivot, want)
                room[side] += dist if hit is None else hit * dist
        return 1 if room[1] > room[-1] + 1e-6 else -1

    def street_position(self, state, occ):
        """A camera on the pavement 24-46 m ahead along the planned path, at eye height, seeing that stretch."""
        path = (state.path or {}).get("pts") or []
        if len(path) < 30 or self.app.kerb is None:
            return None
        kerb = self.app.kerb
        tr = state.tick["truth"]
        # Index of the path point nearest the car.
        i0 = min(range(min(len(path), 20)), key=lambda i: math.hypot(path[i][0] - tr["x"], path[i][1] - tr["y"]))
        best = None
        furniture = getattr(occ, "furniture", [])
        for ahead in (36, 33, 39, 30, 42, 27, 45):
            k = min(len(path) - 2, i0 + ahead)
            (px, py), (qx, qy) = path[k - 1], path[k + 1]
            L = math.hypot(qx - px, qy - py) or 1.0
            ux, uy = (qx - px) / L, (qy - py) / L
            for side in (-1, 1):  # the far pavement first: the car passes it at a distance
                nx, ny = -uy * side, ux * side
                x, y = path[k]
                cam = None
                for lat in [0.5 * j for j in range(4, 40)]:
                    if not kerb.on_road(x + nx * lat, y + ny * lat):
                        lat += 1.7
                        cam = (x + nx * lat, y + ny * lat, 1.75)
                        break
                if cam is None or occ.inside(cam, 0.7) or kerb.on_road(cam[0], cam[1]):
                    continue
                pts = path[i0:min(len(path), k + 10):3]
                # No lamp column or kiosk standing big in the foreground of any view the camera turns to.
                if not clear_foreground(cam, path[i0:min(len(path), k + 10)], furniture):
                    continue
                vis = sum(1 for p in pts if occ.visible(cam, (p[0], p[1], 1.0))) / float(len(pts))
                share = visible_share(occ, cam, tr["x"], tr["y"], tr["yaw"])
                score = vis + 0.5 * share - 0.004 * abs(ahead - 34)
                if best is None or score > best[0]:
                    best = (score, cam, vis, share)
            if best and best[2] >= 0.95 and best[3] >= 0.99:
                break
        if best is None or best[2] < 0.85 or best[3] < 0.75:
            return None
        return best[1]
