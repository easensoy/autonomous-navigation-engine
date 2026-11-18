"""An original, procedurally built I-PACE-inspired proxy vehicle, as plain triangle meshes (no Panda3D).

Dimensions come from the backend's VehicleParams (scene message): wheelbase 2.990 m, length 4.682 m, body
width 2.011 m, 2.139 m over the mirrors, overhangs 0.850 m front and 0.842 m rear. The height (1.565 m),
wheel size (245/50 R20, radius 0.377 m) and track (1.64 m) are published-figure approximations used only
for drawing. The silhouette is a loft of hand-made cross-sections: a short bonnet, a cab-forward
windscreen, a long roof falling to a tall tail with a lip spoiler. No third-party model or texture is used.

Frame: the simulation's body frame. Origin at the rear-axle centre on the ground, x forward, y left, z up.
The pose the backend reports is this origin, so the mesh needs no other transform.

The roof rig is decorative: the simulation has no 3D lidar or camera. The only ranging sensor it simulates
is the planar kerb scanner on the front bumper, drawn separately and labelled as active.
"""
import math

WHITE = (0.93, 0.94, 0.95, 1.0)
GLASS = (0.09, 0.12, 0.16, 1.0)
TRIM = (0.06, 0.06, 0.07, 1.0)
LIGHT = (0.97, 0.98, 1.00, 1.0)
TAIL = (0.72, 0.04, 0.05, 1.0)
RIG = (0.88, 0.89, 0.90, 1.0)
RIG_DARK = (0.10, 0.11, 0.13, 1.0)
TYRE = (0.05, 0.05, 0.05, 1.0)
RIM = (0.70, 0.71, 0.73, 1.0)
SCANNER = (0.12, 0.12, 0.12, 1.0)
SCANNER_MARK = (1.00, 0.55, 0.08, 1.0)

HEIGHT = 1.565
WHEEL_RADIUS = 0.377
WHEEL_WIDTH = 0.245
TRACK = 1.64

# Side profile, rear bumper (u = 0) to front bumper (u = 1): bottom, top, width fraction, beltline (m).
PROFILE = [
    (0.000, 0.44, 0.98, 0.80, 0.98),
    (0.012, 0.34, 1.12, 0.90, 1.02),
    (0.030, 0.29, 1.22, 0.95, 1.04),
    (0.060, 0.25, 1.29, 0.98, 1.05),
    (0.100, 0.22, 1.37, 1.00, 1.05),
    (0.150, 0.20, 1.45, 1.00, 1.04),
    (0.220, 0.19, 1.52, 1.00, 1.03),
    (0.300, 0.19, 1.555, 1.00, 1.02),
    (0.380, 0.19, 1.565, 1.00, 1.01),
    (0.460, 0.19, 1.555, 1.00, 1.00),
    (0.530, 0.19, 1.51, 1.00, 0.99),
    (0.590, 0.19, 1.40, 1.00, 0.98),
    (0.650, 0.19, 1.22, 1.00, 0.97),
    (0.700, 0.20, 1.06, 0.99, 0.96),
    (0.760, 0.21, 0.94, 0.98, 0.94),
    (0.830, 0.23, 0.85, 0.96, 0.85),
    (0.900, 0.27, 0.78, 0.93, 0.78),
    (0.950, 0.31, 0.72, 0.88, 0.72),
    (0.985, 0.37, 0.65, 0.80, 0.65),
    (1.000, 0.44, 0.57, 0.70, 0.57),
]


class Mesh:
    def __init__(self, name):
        self.name = name
        self.v, self.c, self.t = [], [], []

    def add(self, pts, color, tris):
        base = len(self.v)
        self.v.extend(pts)
        self.c.extend([color] * len(pts) if isinstance(color[0], float) else color)
        self.t.extend((a + base, b + base, c + base) for a, b, c in tris)

    def bounds(self):
        xs, ys, zs = zip(*self.v)
        return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))

    def normals(self):
        n = [[0.0, 0.0, 0.0] for _ in self.v]
        for a, b, c in self.t:
            ax, ay, az = self.v[a]
            bx, by, bz = self.v[b]
            cx, cy, cz = self.v[c]
            ux, uy, uz = bx - ax, by - ay, bz - az
            wx, wy, wz = cx - ax, cy - ay, cz - az
            fx, fy, fz = uy * wz - uz * wy, uz * wx - ux * wz, ux * wy - uy * wx
            for i in (a, b, c):
                n[i][0] += fx
                n[i][1] += fy
                n[i][2] += fz
        out = []
        for x, y, z in n:
            L = math.sqrt(x * x + y * y + z * z) or 1.0
            out.append((x / L, y / L, z / L))
        return out


def _lerp_profile(u):
    for (u0, *a), (u1, *b) in zip(PROFILE, PROFILE[1:]):
        if u0 <= u <= u1:
            f = (u - u0) / (u1 - u0)
            return [x + (y - x) * f for x, y in zip(a, b)]
    return list(PROFILE[-1][1:])


def _smooth(f):
    f = max(0.0, min(1.0, f))
    return f * f * (3.0 - 2.0 * f)


def box(mesh, cx, cy, cz, sx, sy, sz, color):
    hx, hy, hz = 0.5 * sx, 0.5 * sy, 0.5 * sz
    faces = [
        ((1, 0, 0), [(hx, -hy, -hz), (hx, hy, -hz), (hx, hy, hz), (hx, -hy, hz)]),
        ((-1, 0, 0), [(-hx, hy, -hz), (-hx, -hy, -hz), (-hx, -hy, hz), (-hx, hy, hz)]),
        ((0, 1, 0), [(hx, hy, -hz), (-hx, hy, -hz), (-hx, hy, hz), (hx, hy, hz)]),
        ((0, -1, 0), [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, -hy, hz), (-hx, -hy, hz)]),
        ((0, 0, 1), [(-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]),
        ((0, 0, -1), [(-hx, hy, -hz), (hx, hy, -hz), (hx, -hy, -hz), (-hx, -hy, -hz)]),
    ]
    for _, quad in faces:
        mesh.add([(cx + x, cy + y, cz + z) for x, y, z in quad], color, [(0, 1, 2), (0, 2, 3)])


def cylinder(mesh, cx, cy, cz, r, h, color, axis="z", seg=24, caps=True, top_color=None):
    """A cylinder along z (centred on cz) or along y (centred on cy)."""
    ring0, ring1 = [], []
    for i in range(seg):
        a = 2.0 * math.pi * i / seg
        ca, sa = math.cos(a) * r, math.sin(a) * r
        if axis == "z":
            ring0.append((cx + ca, cy + sa, cz - 0.5 * h))
            ring1.append((cx + ca, cy + sa, cz + 0.5 * h))
        else:
            ring0.append((cx + ca, cy - 0.5 * h, cz + sa))
            ring1.append((cx + ca, cy + 0.5 * h, cz + sa))
    tris = []
    for i in range(seg):
        j = (i + 1) % seg
        if axis == "z":
            tris += [(i, j, seg + j), (i, seg + j, seg + i)]
        else:
            tris += [(i, seg + i, seg + j), (i, seg + j, j)]
    mesh.add(ring0 + ring1, color, tris)
    if caps:
        for ring, up in ((ring0, False), (ring1, True)):
            centre = (cx, cy, cz + (0.5 * h if up else -0.5 * h)) if axis == "z" else (cx, cy + (0.5 * h if up else -0.5 * h), cz)
            fan = [centre] + ring
            ft = [(0, i + 1, (i + 1) % seg + 1) if (up == (axis == "z")) else (0, (i + 1) % seg + 1, i + 1) for i in range(seg)]
            mesh.add(fan, top_color if (up and top_color) else color, ft)


def body(params, stations=72, ring=40):
    """The lofted body with windows, trim, lights and arches, as one mesh."""
    L = params["length"]
    W = params["width"]
    xr = -params["rear_overhang"]
    wb = params["wheelbase"]
    mesh = Mesh("body")
    rings = []
    colors = []
    for i in range(stations + 1):
        u = i / stations
        x = xr + u * L
        zb, zt, wf, belt = _lerp_profile(u)
        scale = HEIGHT / 1.565
        zb, zt, belt = zb * scale, zt * scale, belt * scale
        # Wheel arches: lift the underside over each wheel.
        for xw in (0.0, wb):
            dx = x - xw
            R = WHEEL_RADIUS + 0.06
            if abs(dx) < R:
                zb = max(zb, WHEEL_RADIUS + math.sqrt(R * R - dx * dx) * 0.98)
        a = 0.5 * W * wf
        b = 0.5 * (zt - zb)
        zc = 0.5 * (zt + zb)
        pts, cols = [], []
        for k in range(ring):
            phi = 2.0 * math.pi * k / ring  # 0 at the bottom centre, increasing towards +y (left)
            s, c = math.sin(phi), -math.cos(phi)
            p = 3.2
            y = a * math.copysign(abs(s) ** (2.0 / p), s)
            z = zc + b * math.copysign(abs(c) ** (2.0 / p), c)
            if z > belt and zt > belt:
                y *= 1.0 - 0.27 * _smooth((z - belt) / (zt - belt))
            pts.append((x, y, z))
            cols.append(_body_colour(u, x, y, z, zb, zt, belt, a, wb, xr, L))
        rings.append(pts)
        colors.append(cols)
    for i in range(stations):
        base = len(mesh.v)
        for k in range(ring):
            mesh.v.append(rings[i][k])
            mesh.c.append(colors[i][k])
        for k in range(ring):
            mesh.v.append(rings[i + 1][k])
            mesh.c.append(colors[i + 1][k])
        for k in range(ring):
            j = (k + 1) % ring
            mesh.t += [(base + k, base + ring + j, base + ring + k), (base + k, base + j, base + ring + j)]
    # End caps.
    for idx, front in ((0, False), (stations, True)):
        pts = rings[idx]
        cx = sum(p[0] for p in pts) / len(pts)
        cz = sum(p[2] for p in pts) / len(pts)
        fan = [(cx, 0.0, cz)] + pts
        col = [TRIM if front else WHITE] + colors[idx]  # grille dark at the front; body colour behind the tail band
        tris = [(0, k + 1, (k + 1) % ring + 1) if front else (0, (k + 1) % ring + 1, k + 1) for k in range(ring)]
        mesh.add(fan, col, tris)
    return mesh


def _body_colour(u, x, y, z, zb, zt, belt, a, wb, xr, L):
    ay = abs(y)
    if z < zb + 0.07:
        return TRIM  # sills and the edge of each wheel arch (the underside is lifted over the wheels)
    if u > 0.955 and 0.33 < z < 0.56 and ay < 0.55 * a:
        return TRIM  # grille
    if u > 0.93 and 0.60 < z < 0.70 and ay > 0.48 * a:
        return LIGHT
    if u < 0.03 and 0.86 < z < 0.98:
        return TAIL
    roof_rim = zt - 0.045
    if z > belt + 0.015 and z < roof_rim:
        side = ay > 0.62 * a
        if 0.165 < u < 0.715 and side and not (0.462 < u < 0.494):
            return GLASS  # side windows, B-pillar left in body colour
        if 0.585 < u < 0.715 and not side:
            return GLASS  # windscreen
        if 0.035 < u < 0.165 and not side:
            return GLASS  # rear screen below the spoiler
    return WHITE


def wheel(params, name):
    """One wheel centred on its hub, axle along y: tyre, rim and five spokes (so its rotation shows)."""
    m = Mesh(name)
    cylinder(m, 0.0, 0.0, 0.0, WHEEL_RADIUS, WHEEL_WIDTH, TYRE, axis="y", seg=28)
    side = 1.0 if name.endswith("l") else -1.0
    face = side * (0.5 * WHEEL_WIDTH + 0.004)
    cylinder(m, 0.0, face, 0.0, 0.25, 0.01, RIM, axis="y", seg=28)
    for k in range(5):
        a = 2.0 * math.pi * k / 5
        for r in (0.10, 0.17):
            box(m, math.cos(a) * r, face + side * 0.008, math.sin(a) * r, 0.05, 0.012, 0.05, TRIM)
    return m


def wheel_positions(params):
    half = 0.5 * TRACK
    wb = params["wheelbase"]
    return {"fl": (wb, half, WHEEL_RADIUS), "fr": (wb, -half, WHEEL_RADIUS), "rl": (0.0, half, WHEEL_RADIUS), "rr": (0.0, -half, WHEEL_RADIUS)}


def details(params):
    """Mirrors, the decorative roof rig and the active kerb scanner, as named meshes."""
    W = params["width"]
    Wm = params["width_mirrors"]
    wb = params["wheelbase"]
    front = wb + params["front_overhang"]
    mirrors = Mesh("mirrors")
    for side in (1.0, -1.0):
        box(mirrors, wb - 0.36, side * (0.5 * Wm - 0.075), 1.02, 0.20, 0.15, 0.12, WHITE)
        box(mirrors, wb - 0.33, side * (0.5 * W - 0.05), 0.99, 0.10, 0.12, 0.05, TRIM)
    rig = Mesh("roof_rig")  # visual only
    roof = HEIGHT
    box(rig, 1.10, 0.0, roof + 0.02, 0.95, 0.80, 0.05, RIG)
    cylinder(rig, 1.10, 0.0, roof + 0.13, 0.34, 0.18, RIG, seg=32)
    cylinder(rig, 1.10, 0.0, roof + 0.27, 0.33, 0.10, RIG_DARK, seg=32, caps=False)
    cylinder(rig, 1.10, 0.0, roof + 0.36, 0.31, 0.08, RIG, seg=32, top_color=RIG)
    for side in (1.0, -1.0):
        cylinder(rig, wb - 0.10, side * (0.5 * W - 0.06), 0.98, 0.075, 0.16, RIG_DARK, seg=16)
        cylinder(rig, -0.55, side * (0.5 * W - 0.10), 1.06, 0.07, 0.14, RIG_DARK, seg=16)
    antenna = Mesh("gnss_antenna")  # the simulated GNSS antenna position
    cylinder(antenna, 1.60, 0.0, roof - 0.005, 0.07, 0.03, RIG_DARK, seg=16)
    scanner = Mesh("kerb_scanner")  # the active planar kerb scanner, front bumper centre at kerb height
    box(scanner, front - 0.02, 0.0, 0.20, 0.10, 0.22, 0.08, SCANNER)
    box(scanner, front + 0.031, 0.0, 0.20, 0.004, 0.18, 0.025, SCANNER_MARK)
    return {"mirrors": mirrors, "roof_rig": rig, "gnss_antenna": antenna, "kerb_scanner": scanner}


def proxy(params):
    """Every part of the proxy: body, details and the four wheels (each in its hub frame)."""
    parts = {"body": body(params)}
    parts.update(details(params))
    wheels = {k: wheel(params, k) for k in ("fl", "fr", "rl", "rr")}
    return parts, wheels, wheel_positions(params)
