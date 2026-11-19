"""Original, procedurally built proxy vehicles, as plain triangle meshes (no Panda3D): one for each vehicle
profile the backend simulates (road_driving/include/road_driving/vehicle.hpp), picked by the profile id in the
scene message.

- ipace_proxy_v1: I-PACE-inspired. Wheelbase 2.990 m, length 4.682 m, body width 2.011 m, 2.139 m over the
  mirrors, overhangs 0.850 m front and 0.842 m rear. The height (1.565 m), wheel size (245/50 R20, radius
  0.377 m) and track (1.64 m) are published-figure approximations used only for drawing. A short bonnet, a
  cab-forward windscreen, a long roof falling to a tall tail with a lip spoiler; a decorative roof rig.
- cybercab_proxy_v1: Cybercab-inspired. Wheelbase 2.750 m, length 4.200 m, width 1.930 m, no exterior mirrors
  (the profile's research assumptions); height 1.44 m, wheel radius 0.36 m and track 1.62 m are drawing
  assumptions, and the ground clearance (0.147 m) is Tesla's published figure. A low two-seat body with a long
  raked windscreen running into a glass roof, a fastback to a short tail, full-width light bars front and rear,
  aero wheel covers, a champagne-gold finish. No roof rig and no camera is drawn: none is simulated.

The dimensions that the simulation uses (wheelbase, length, widths, overhangs) come from the scene message, so
the mesh always matches the simulated footprint. No third-party model or texture is used.

Frame: the simulation's body frame. Origin at the rear-axle centre on the ground, x forward, y left, z up.
The pose the backend reports is this origin, so the mesh needs no other transform.

The only ranging sensor the simulation has is the planar kerb scanner on the front bumper, drawn separately and
labelled as active; the GNSS antenna is drawn where the profile mounts it.
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

# The Cybercab-inspired proxy's side profile, in metres at its own height (1.44 m): bottom, top, width fraction,
# beltline. The roof peaks a little behind the middle; the windscreen runs long and low towards the nose.
CYBERCAB_PROFILE = [
    (0.000, 0.30, 0.86, 0.86, 0.86),
    (0.020, 0.24, 0.97, 0.95, 0.90),
    (0.050, 0.19, 1.04, 0.98, 0.92),
    (0.100, 0.16, 1.12, 1.00, 0.93),
    (0.180, 0.15, 1.24, 1.00, 0.93),
    (0.260, 0.15, 1.34, 1.00, 0.92),
    (0.340, 0.15, 1.41, 1.00, 0.91),
    (0.420, 0.15, 1.44, 1.00, 0.90),
    (0.500, 0.15, 1.43, 1.00, 0.89),
    (0.580, 0.15, 1.36, 1.00, 0.87),
    (0.650, 0.15, 1.21, 1.00, 0.85),
    (0.720, 0.16, 1.03, 0.99, 0.82),
    (0.800, 0.17, 0.89, 0.97, 0.78),
    (0.870, 0.19, 0.79, 0.93, 0.74),
    (0.930, 0.22, 0.71, 0.88, 0.69),
    (0.975, 0.28, 0.64, 0.80, 0.62),
    (1.000, 0.36, 0.57, 0.68, 0.57),
]
GOLD = (0.60, 0.47, 0.27, 1.0)  # champagne gold, before the paint's specular highlight
LIGHT_BAR = (0.98, 0.98, 1.00, 1.0)
AERO = (0.52, 0.53, 0.55, 1.0)

# Drawing designs per profile. Everything the simulation uses comes from the scene message instead.
DESIGNS = {
    "ipace_proxy_v1": {"profile": PROFILE, "height": HEIGHT, "profile_height": 1.565, "wheel_radius": WHEEL_RADIUS, "wheel_width": WHEEL_WIDTH,
                       "track": TRACK, "paint": WHITE, "mirrors": True, "rig": True, "antenna_x": 1.60, "kind": "ipace"},
    "cybercab_proxy_v1": {"profile": CYBERCAB_PROFILE, "height": 1.44, "profile_height": 1.44, "wheel_radius": 0.36, "wheel_width": 0.24,
                          "track": 1.62, "paint": GOLD, "mirrors": False, "rig": False, "antenna_x": 1.30, "kind": "cybercab"},
}


def design(params):
    """The drawing design for the scene's vehicle (the I-PACE proxy's for an older message without an id)."""
    d = dict(DESIGNS.get(params.get("id", "ipace_proxy_v1"), DESIGNS["ipace_proxy_v1"]))
    # A newer scene message carries the drawing figures and the antenna's mount; they win.
    for key, src in (("height", "height"), ("wheel_radius", "wheel_radius"), ("track", "track"), ("antenna_x", "antenna_x")):
        if params.get(src) is not None:
            d[key] = params[src]
    return d


class Mesh:
    def __init__(self, name):
        self.name = name
        self.v, self.c, self.t = [], [], []
        self.n = None  # optional per-vertex normals; otherwise computed from the triangles

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


def _lerp_profile(u, profile=PROFILE):
    for (u0, *a), (u1, *b) in zip(profile, profile[1:]):
        if u0 <= u <= u1:
            f = (u - u0) / (u1 - u0)
            return [x + (y - x) * f for x, y in zip(a, b)]
    return list(profile[-1][1:])


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


GAP = 0.004  # m: glass and lamps stand this far off the paint, along the surface normal


def _section(params, u):
    """The loft's cross-section at station u (0 rear bumper, 1 front bumper): x, bottom, top, beltline, half-width."""
    d = design(params)
    x = -params["rear_overhang"] + u * params["length"]
    zb, zt, wf, belt = _lerp_profile(u, d["profile"])
    scale = d["height"] / d["profile_height"]
    zb, zt, belt = zb * scale, zt * scale, belt * scale
    # Wheel arches: lift the underside over each wheel.
    wr = d["wheel_radius"]
    for xw in (0.0, params["wheelbase"]):
        dx = x - xw
        R = wr + 0.06
        if abs(dx) < R:
            zb = max(zb, wr + math.sqrt(R * R - dx * dx) * 0.98)
    return x, zb, zt, belt, 0.5 * params["width"] * wf


def _ring_point(sec, phi):
    """The body surface at ring angle phi: 0 at the bottom centre, increasing towards +y (left), pi on top."""
    x, zb, zt, belt, a = sec
    b, zc = 0.5 * (zt - zb), 0.5 * (zt + zb)
    s, c = math.sin(phi), -math.cos(phi)
    p = 3.2
    y = a * math.copysign(abs(s) ** (2.0 / p), s)
    z = zc + b * math.copysign(abs(c) ** (2.0 / p), c)
    if z > belt and zt > belt:
        y *= 1.0 - 0.27 * _smooth((z - belt) / (zt - belt))
    return (x, y, z)


def _phi_where(sec, f, target):
    """The ring angle on the upper left quarter (pi/2 to pi, where z rises and y falls) at which f(point)
    equals target, or None if it does not get there."""
    lo, hi = 0.5 * math.pi, math.pi
    g_lo = f(_ring_point(sec, lo)) - target
    if g_lo * (f(_ring_point(sec, hi)) - target) > 0.0:
        return None
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if (f(_ring_point(sec, mid)) - target > 0.0) == (g_lo > 0.0):
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def _z_at(z):
    return lambda sec: _phi_where(sec, lambda q: q[2], z)


def _y_at(frac):
    return lambda sec: _phi_where(sec, lambda q: q[1], frac * sec[4])


def _first(*phis):
    """The lowest of the ring angles that exist (None if none does)."""
    phis = [f for f in phis if f is not None]
    return min(phis) if phis else None


def _normal(params, u, phi):
    e = 1e-4
    sec = _section(params, u)
    pu0 = _ring_point(_section(params, max(0.0, u - e)), phi)
    pu1 = _ring_point(_section(params, min(1.0, u + e)), phi)
    pf0, pf1 = _ring_point(sec, phi - e), _ring_point(sec, phi + e)
    du = [b - a for a, b in zip(pu0, pu1)]
    df = [b - a for a, b in zip(pf0, pf1)]
    n = (du[1] * df[2] - du[2] * df[1], du[2] * df[0] - du[0] * df[2], du[0] * df[1] - du[1] * df[0])
    p = _ring_point(sec, phi)
    out = (0.0, p[1], p[2] - 0.5 * (sec[1] + sec[2]))
    if n[0] * out[0] + n[1] * out[1] + n[2] * out[2] < 0.0:
        n = (-n[0], -n[1], -n[2])
    L = math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2) or 1.0
    return (n[0] / L, n[1] / L, n[2] / L)


def _add_quads(mesh, grid, colour, mirror=False):
    """Quads between rows of (point, normal) of equal length, each wound to face along its normals."""
    for r0, r1 in zip(grid, grid[1:]):
        if r0 is None or r1 is None:
            continue
        for j in range(len(r0) - 1):
            quad = [r0[j], r0[j + 1], r1[j + 1], r1[j]]
            if mirror:
                quad = [((x, -y, z), (nx, -ny, nz)) for (x, y, z), (nx, ny, nz) in quad]
            pts = [q[0] for q in quad]
            nrm = [sum(q[1][i] for q in quad) for i in range(3)]
            (ax, ay, az), (bx, by, bz), (cx, cy, cz) = pts[0], pts[1], pts[2]
            ux, uy, uz, wx, wy, wz = bx - ax, by - ay, bz - az, cx - ax, cy - ay, cz - az
            face = (uy * wz - uz * wy, uz * wx - ux * wz, ux * wy - uy * wx)
            tris = [(0, 1, 2), (0, 2, 3)] if sum(f * n for f, n in zip(face, nrm)) > 0.0 else [(0, 2, 1), (0, 3, 2)]
            base = len(mesh.v)
            for pt, n in quad:
                mesh.v.append(pt)
                mesh.c.append(colour)
                mesh.n.append(n)
            mesh.t += [(base + a, base + b, base + c) for a, b, c in tris]


def _patch(mesh, params, u0, u1, lo, hi, colour, nu=24, nt=10, across=False):
    """Glass or a lamp on the body surface, lifted GAP off the paint: from station u0(t) to u1(t), and at each
    station from ring angle lo(sec) to hi(sec), t running from lo to hi. The left one and its mirror image on
    the right; with `across`, one patch from lo on the left over the top to its mirror on the right (a screen)."""
    rows = []
    for i in range(nu + 1):
        s = i / nu
        row = []
        for j in range(nt + 1):
            t = j / nt
            u = u0(t) + (u1(t) - u0(t)) * s
            sec = _section(params, u)
            a = lo(sec)
            b = None if across else hi(sec)
            if a is None or (not across and (b is None or b <= a)):
                row = None
                break
            phi = a + (2.0 * math.pi - 2.0 * a) * t if across else a + (b - a) * t
            pt = _ring_point(sec, phi)
            n = _normal(params, u, phi)
            row.append(((pt[0] + n[0] * GAP, pt[1] + n[1] * GAP, pt[2] + n[2] * GAP), n))
        rows.append(row)
    _add_quads(mesh, rows, colour)
    if not across:
        _add_quads(mesh, rows, colour, mirror=True)


def _const(v):
    return lambda t: v


def body(params, stations=72, ring=40):
    """The lofted body with its trim and arches, and its glass and lamps laid on as smooth patches of its
    surface, so their outlines follow the body's lines rather than the mesh's quads."""
    mesh = Mesh("body")
    d = design(params)
    rings = []
    colors = []
    for i in range(stations + 1):
        u = i / stations
        sec = _section(params, u)
        pts = [_ring_point(sec, 2.0 * math.pi * k / ring) for k in range(ring)]
        rings.append(pts)
        colors.append([_body_colour(u, q[1], q[2], sec[1], sec[4], d) for q in pts])
    # Smooth normals over the whole loft grid, so the shading runs on across stations.
    acc = [[[0.0, 0.0, 0.0] for _ in range(ring)] for _ in range(stations + 1)]
    for i in range(stations):
        for k in range(ring):
            j = (k + 1) % ring
            for tri in (((i, k), (i + 1, j), (i + 1, k)), ((i, k), (i, j), (i + 1, j))):
                (ax, ay, az), (bx, by, bz), (cx, cy, cz) = (rings[a][b] for a, b in tri)
                ux, uy, uz = bx - ax, by - ay, bz - az
                wx, wy, wz = cx - ax, cy - ay, cz - az
                f = (uy * wz - uz * wy, uz * wx - ux * wz, ux * wy - uy * wx)
                for a, b in tri:
                    acc[a][b][0] += f[0]
                    acc[a][b][1] += f[1]
                    acc[a][b][2] += f[2]
    grid_n = [[tuple(v / (math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2) or 1.0) for v in n) for n in row] for row in acc]
    mesh.n = []
    # One colour per quad (the most common of its corners'), so the sills and grille keep crisp edges.
    for i in range(stations):
        for k in range(ring):
            j = (k + 1) % ring
            corners = ((i, k), (i, j), (i + 1, j), (i + 1, k))
            cols = [colors[a][b] for a, b in corners]
            col = max(cols, key=cols.count)
            base = len(mesh.v)
            for a, b in corners:
                mesh.v.append(rings[a][b])
                mesh.c.append(col)
                mesh.n.append(grid_n[a][b])
            mesh.t += [(base, base + 2, base + 3), (base, base + 1, base + 2)]
    # End caps.
    for idx, front in ((0, False), (stations, True)):
        pts = rings[idx]
        cx = sum(p[0] for p in pts) / len(pts)
        cz = sum(p[2] for p in pts) / len(pts)
        fan = [(cx, 0.0, cz)] + pts
        if d["kind"] == "cybercab":
            col = [d["paint"]] + colors[idx]  # no grille: the nose and the tail are body colour
        else:
            col = [TRIM if front else WHITE] + colors[idx]  # grille dark at the front
        tris = [(0, k + 1, (k + 1) % ring + 1) if front else (0, (k + 1) % ring + 1, k + 1) for k in range(ring)]
        mesh.add(fan, col, tris)
    flat = mesh.normals()  # the end caps: their own (flat) normals
    mesh.n += flat[len(mesh.n):]

    sill = lambda sec: _phi_where(sec, lambda q: q[2], sec[3] + 0.03)
    rail = lambda sec: (lambda a: None if a is None else a - 0.03)(_first(_y_at(0.62)(sec), _z_at(sec[2] - 0.05)(sec)))
    if d["kind"] == "cybercab":
        _cybercab_glass_and_lights(mesh, params, sill, rail)
        return mesh

    # Glass: two side windows a side, with the B- and C-pillars in body colour, the windscreen and the rear
    # screen. The side glass runs from just above the beltline to the roof rail.
    _patch(mesh, params, _const(0.500), lambda t: 0.708 - 0.045 * t, sill, rail, GLASS)   # front door, A-pillar raked
    _patch(mesh, params, lambda t: 0.172 + 0.05 * t, _const(0.456), sill, rail, GLASS)   # rear door and quarter
    screen = lambda sec: (lambda a: None if a is None else a + 0.03)(_y_at(0.62)(sec))
    _patch(mesh, params, _const(0.598), _const(0.706), screen, None, GLASS, nu=14, nt=24, across=True)
    _patch(mesh, params, _const(0.042), _const(0.158), screen, None, GLASS, nu=14, nt=24, across=True)
    # Lamps: slim headlights at the front corners, and the tail lamps round the rear corners.
    _patch(mesh, params, _const(0.935), _const(0.99), _z_at(0.60), lambda sec: _first(_z_at(0.69)(sec), _y_at(0.48)(sec)),
           LIGHT, nu=8, nt=4)
    _patch(mesh, params, _const(0.0), _const(0.028), _z_at(0.87), _z_at(0.95), TAIL, nu=4, nt=4)
    # The light bar across the tail (the rear end cap), between the corner lamps.
    sec0 = _section(params, 0.0)
    rows = []
    for z in (0.87, 0.89, 0.91, 0.93, 0.95):
        phi = _phi_where(sec0, lambda q: q[2], z)
        if phi is None:
            rows.append(None)
            continue
        w = abs(_ring_point(sec0, phi)[1]) - 0.01
        rows.append([((sec0[0] - 0.5 * GAP, w * (2.0 * k / 10 - 1.0), z), (-1.0, 0.0, 0.0)) for k in range(11)])
    _add_quads(mesh, rows, TAIL)
    return mesh


def _cybercab_glass_and_lights(mesh, params, sill, rail):
    """The Cybercab-inspired proxy's glass and lamps: one long side window a side (a single wide butterfly door),
    a windscreen that runs on into the glass roof, a small rear screen, and full-width light bars front and rear."""
    _patch(mesh, params, _const(0.30), lambda t: 0.66 - 0.05 * t, sill, rail, GLASS)  # side glass, the door's window
    screen = lambda sec: (lambda a: None if a is None else a + 0.03)(_y_at(0.66)(sec))
    _patch(mesh, params, _const(0.36), _const(0.72), screen, None, GLASS, nu=20, nt=24, across=True)  # windscreen into the glass roof
    _patch(mesh, params, _const(0.06), _const(0.16), screen, None, GLASS, nu=10, nt=24, across=True)  # rear screen
    # The light bars: a slim strip across each end cap.
    for u, z0, z1, col, n in ((1.0, 0.585, 0.615, LIGHT_BAR, 1.0), (0.0, 0.80, 0.84, TAIL, -1.0)):
        sec = _section(params, u)
        rows = []
        for z in (z0, 0.5 * (z0 + z1), z1):
            phi = _phi_where(sec, lambda q: q[2], z)
            if phi is None:
                rows.append(None)
                continue
            w = abs(_ring_point(sec, phi)[1]) - 0.02
            rows.append([((sec[0] + n * 0.5 * GAP, w * (2.0 * k / 10 - 1.0), z), (n, 0.0, 0.0)) for k in range(11)])
        _add_quads(mesh, rows, col)


def _body_colour(u, y, z, zb, a, d=None):
    ay = abs(y)
    if z < zb + 0.07:
        return TRIM  # sills and the edge of each wheel arch (the underside is lifted over the wheels)
    if d is not None and d["kind"] == "cybercab":
        return d["paint"]  # no grille
    if u > 0.955 and 0.33 < z < 0.56 and ay < 0.55 * a:
        return TRIM  # grille
    return WHITE


def wheel(params, name):
    """One wheel centred on its hub, axle along y: tyre, rim and five spokes (so its rotation shows); on the
    Cybercab-inspired proxy a flat aero cover with five darker vanes."""
    d = design(params)
    wr, ww = d["wheel_radius"], d["wheel_width"]
    m = Mesh(name)
    cylinder(m, 0.0, 0.0, 0.0, wr, ww, TYRE, axis="y", seg=28)
    side = 1.0 if name.endswith("l") else -1.0
    face = side * (0.5 * ww + 0.004)
    if d["kind"] == "cybercab":
        cylinder(m, 0.0, face, 0.0, wr - 0.05, 0.01, AERO, axis="y", seg=32)
        for k in range(5):
            a = 2.0 * math.pi * k / 5
            box(m, math.cos(a) * 0.16, face + side * 0.008, math.sin(a) * 0.16, 0.16, 0.012, 0.035, TRIM)
        return m
    cylinder(m, 0.0, face, 0.0, 0.25, 0.01, RIM, axis="y", seg=28)
    for k in range(5):
        a = 2.0 * math.pi * k / 5
        for r in (0.10, 0.17):
            box(m, math.cos(a) * r, face + side * 0.008, math.sin(a) * r, 0.05, 0.012, 0.05, TRIM)
    return m


def wheel_positions(params):
    d = design(params)
    half, wr = 0.5 * d["track"], d["wheel_radius"]
    wb = params["wheelbase"]
    return {"fl": (wb, half, wr), "fr": (wb, -half, wr), "rl": (0.0, half, wr), "rr": (0.0, -half, wr)}


def details(params):
    """Mirrors, the decorative roof rig, the GNSS antenna and the active kerb scanner, as named meshes (empty where
    the profile has none)."""
    d = design(params)
    W = params["width"]
    Wm = params["width_mirrors"]
    wb = params["wheelbase"]
    front = wb + params["front_overhang"]
    mirrors = Mesh("mirrors")
    if d["mirrors"]:
        for side in (1.0, -1.0):
            box(mirrors, wb - 0.36, side * (0.5 * Wm - 0.075), 1.02, 0.20, 0.15, 0.12, WHITE)
            box(mirrors, wb - 0.33, side * (0.5 * W - 0.05), 0.99, 0.10, 0.12, 0.05, TRIM)
    rig = Mesh("roof_rig")  # visual only
    roof = HEIGHT
    if d["rig"]:
        box(rig, 1.10, 0.0, roof + 0.02, 0.95, 0.80, 0.05, RIG)
        cylinder(rig, 1.10, 0.0, roof + 0.13, 0.34, 0.18, RIG, seg=32)
        cylinder(rig, 1.10, 0.0, roof + 0.27, 0.33, 0.10, RIG_DARK, seg=32, caps=False)
        cylinder(rig, 1.10, 0.0, roof + 0.36, 0.31, 0.08, RIG, seg=32, top_color=RIG)
        for side in (1.0, -1.0):
            cylinder(rig, wb - 0.10, side * (0.5 * W - 0.06), 0.98, 0.075, 0.16, RIG_DARK, seg=16)
            cylinder(rig, -0.55, side * (0.5 * W - 0.10), 1.06, 0.07, 0.14, RIG_DARK, seg=16)
    antenna = Mesh("gnss_antenna")  # the simulated GNSS antenna position
    if d["kind"] == "cybercab":
        ax = d["antenna_x"]
        top = _section(params, (ax + params["rear_overhang"]) / params["length"])[2]
        cylinder(antenna, ax, 0.0, top + 0.012, 0.07, 0.03, RIG_DARK, seg=16)
    else:
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
