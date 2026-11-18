"""Modular street fronts for the district scenes: the detail drawn on a building's street-facing walls.

Each style is a set of modules laid out in bays along a wall and in storeys up it, after the list entries for
the areas' building types (Historic England; districts/PROVENANCE.md names them): stucco terraces with
rusticated ground floors, Doric porches, a balustraded first-floor balcony, corniced windows, a main cornice
and area railings (Kensington); red-brick mansion blocks with stone bands, bay windows and Dutch gables;
shopfronts with fascias and awnings below stone or brick upper floors (Brompton Road); pinky-buff terracotta
with two storeys of display windows, giant pilasters, octagonal corner cupolas, copper mansard attics and an
octagonal terracotta dome (the Harrods frontage); and modern institutional blocks with ribbon windows. The geometry is original and approximate: proportions and
elements, not a survey of any facade. Everything here is decoration and never reaches the simulation.
"""
import math

TRIM = (0.97, 0.96, 0.92, 1.0)
WHITE = (1.0, 1.0, 1.0, 1.0)
DARK = (0.06, 0.07, 0.07, 1.0)
GLASS = (1.0, 1.0, 1.0, 1.0)
DOORS = [(0.08, 0.10, 0.22, 1.0), (0.05, 0.05, 0.05, 1.0), (0.42, 0.06, 0.07, 1.0), (0.08, 0.24, 0.15, 1.0), (0.30, 0.31, 0.33, 1.0)]
FASCIA = [(0.10, 0.12, 0.14, 1.0), (0.62, 0.50, 0.30, 1.0), (0.16, 0.24, 0.36, 1.0), (0.42, 0.10, 0.12, 1.0), (0.88, 0.86, 0.80, 1.0),
          (0.14, 0.30, 0.26, 1.0)]
AWNING = [(0.14, 0.30, 0.22, 1.0), (0.36, 0.08, 0.09, 1.0), (0.12, 0.16, 0.30, 1.0), (0.82, 0.80, 0.74, 1.0), (0.18, 0.18, 0.18, 1.0)]
HARRODS_GREEN = (0.05, 0.24, 0.15, 1.0)
COPPER = (0.43, 0.56, 0.50, 1.0)  # weathered copper: the mansard attics' fish-scale sheeting
TERRACOTTA_DOME = (1.0, 1.0, 1.0, 1.0)
SIGNS = ["CAFE", "PHARMACY", "BOOKS", "GALLERY", "FLORIST", "TAILOR", "CHOCOLATIER", "OPTICIAN", "BAKERY", "JEWELLER", "PATISSERIE",
         "STATIONER", "PERFUMERY", "WINE MERCHANT", "BOUTIQUE", "SHOES"]


class Wall:
    """A street-facing wall from p0 to p1 (outside on the right), with helpers that place modules on it."""

    def __init__(self, M, p0, p1, base, top):
        self.M = M
        self.p0 = p0
        self.L = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        self.t = ((p1[0] - p0[0]) / self.L, (p1[1] - p0[1]) / self.L)
        self.n = (self.t[1], -self.t[0])  # outward
        self.base, self.top = base, top

    def P(self, u, w, z):
        return (self.p0[0] + self.t[0] * u + self.n[0] * w, self.p0[1] + self.t[1] * u + self.n[1] * w, z)

    def face(self, mat, u0, u1, z0, z1, w, color, uv=None):
        """A rectangle parallel to the wall at offset w, facing out; uv: tile (repeat in metres) or explicit."""
        a, b, c, d = self.P(u0, w, z0), self.P(u1, w, z0), self.P(u1, w, z1), self.P(u0, w, z1)
        b_ = self.M[mat]
        if uv is None:
            k = 1.0 / b_.tile
            uv = ((u0 * k, z0 * k), (u1 * k, z0 * k), (u1 * k, z1 * k), (u0 * k, z1 * k))
        b_.quad(a, b, c, d, color, uv)

    def slab(self, mat, u0, u1, z0, z1, w0, w1, color, ends=True):
        """A projecting block: front at w1, top, bottom and ends back to w0."""
        self.face(mat, u0, u1, z0, z1, w1, color)
        b_ = self.M[mat]
        k = 1.0 / b_.tile
        P = self.P
        b_.quad(P(u0, w1, z1), P(u1, w1, z1), P(u1, w0, z1), P(u0, w0, z1), color, ((u0 * k, w1 * k), (u1 * k, w1 * k), (u1 * k, w0 * k), (u0 * k, w0 * k)))
        b_.quad(P(u0, w0, z0), P(u1, w0, z0), P(u1, w1, z0), P(u0, w1, z0), color, ((u0 * k, 0), (u1 * k, 0), (u1 * k, w1 * k), (u0 * k, w1 * k)))
        if ends:
            b_.quad(P(u0, w0, z0), P(u0, w1, z0), P(u0, w1, z1), P(u0, w0, z1), color, ((0, z0 * k), (w1 * k, z0 * k), (w1 * k, z1 * k), (0, z1 * k)))
            b_.quad(P(u1, w1, z0), P(u1, w0, z0), P(u1, w0, z1), P(u1, w1, z1), color, ((0, z0 * k), (w1 * k, z0 * k), (w1 * k, z1 * k), (0, z1 * k)))

    def window(self, uc, z0, w, h, frame=TRIM, sill=True, hood=None, mat_frame="trim", glass_tint=GLASS):
        self.face(mat_frame, uc - 0.5 * w - 0.12, uc + 0.5 * w + 0.12, z0 - 0.10, z0 + h + 0.12, 0.025, frame)
        self.face("glass", uc - 0.5 * w, uc + 0.5 * w, z0, z0 + h, 0.045, glass_tint, uv=((0, 0), (1, 0), (1, 1), (0, 1)))
        if sill:
            self.slab(mat_frame, uc - 0.5 * w - 0.18, uc + 0.5 * w + 0.18, z0 - 0.16, z0 - 0.04, 0.0, 0.14, frame)
        if hood:
            self.slab(mat_frame, uc - 0.5 * w - 0.24, uc + 0.5 * w + 0.24, z0 + h + 0.14, z0 + h + 0.32, 0.0, hood, frame)

    def column(self, mat, uc, wc, z0, z1, r, color, sides=8):
        b_ = self.M[mat]
        pts = [self.P(uc + r * math.cos(2 * math.pi * k / sides), wc + r * math.sin(2 * math.pi * k / sides), 0.0) for k in range(sides)]
        for k in range(sides):
            a, b = pts[k], pts[(k + 1) % sides]
            # The polygon runs anticlockwise or clockwise depending on the wall's handedness: face outwards.
            cx, cy, _ = self.P(uc, wc, 0.0)
            mx, my = 0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])
            ex, ey = b[0] - a[0], b[1] - a[1]
            if ey * (mx - cx) - ex * (my - cy) < 0:
                a, b = b, a
            b_.wall((a[0], a[1]), (b[0], b[1]), z0, z1, color)

    def railing(self, u0, u1, w, z0, h):
        if u1 - u0 > 0.05:
            self.face("railing", u0, u1, z0, z0 + h, w, WHITE, uv=((0, 0), (u1 - u0, 0), (u1 - u0, 1), (0, 1)))


def bays(L, width):
    n = max(1, int(round(L / width)))
    bw = L / n
    return [bw * (i + 0.5) for i in range(n)], bw


def stucco_terrace(wall, storeys, rng, front_room, ground_h=3.6):
    """Kensington's Italianate stucco terraces: rusticated ground floor raised over a basement area, a
    portico at the door, a continuous first-floor balcony with iron railings, tall first-floor windows under
    cornice hoods, smaller windows above, a main cornice and parapet; railings round the basement area."""
    L, base, top = wall.L, wall.base, wall.top
    if L < 3.0:
        return
    raised = 0.9
    n = max(2, storeys)
    first_h = 4.0
    upper = max(2.7, (top - base - raised - ground_h - first_h - 1.0) / max(1, n - 2))
    wall.face("stucco_rustic", 0.0, L, base, base + raised + ground_h, 0.012, TRIM)
    centres, bw = bays(L, 2.7)
    door_bay = 0 if rng.random() < 0.5 else len(centres) - 1
    z1 = base + raised + ground_h
    for i, uc in enumerate(centres):
        if i == door_bay and L >= 4.5:
            dw = min(1.25, bw - 0.6)
            wall.face("trim", uc - 0.5 * dw, uc + 0.5 * dw, base + raised, base + raised + 2.7, 0.03, rng.choice(DOORS))
            wall.face("glass", uc - 0.5 * dw, uc + 0.5 * dw, base + raised + 2.8, base + raised + 3.3, 0.04, GLASS, uv=((0, 0), (1, 0), (1, 0.5), (0, 0.5)))
            if front_room > 1.4:  # portico: two columns carrying the balcony
                depth = min(1.7, front_room - 0.4)
                for du in (-0.5 * dw - 0.25, 0.5 * dw + 0.25):
                    wall.column("trim", uc + du, depth - 0.25, base + raised, z1 - 0.05, 0.2, TRIM)
                wall.slab("trim", uc - 0.5 * dw - 0.55, uc + 0.5 * dw + 0.55, z1 - 0.05, z1 + 0.5, 0.0, depth, TRIM)
                for k in range(5):  # steps up from the pavement, across the area
                    zz = base + raised * (k + 1) / 5.0
                    wall.slab("stone", uc - 0.5 * dw - 0.2, uc + 0.5 * dw + 0.2, base, zz, 0.0, depth + 0.35 * (5 - k) / 5.0 + 0.05,
                              (0.80, 0.79, 0.75, 1.0))
        else:
            wall.window(uc, base + raised + 0.7, min(1.2, bw - 0.9), 2.2, hood=None)
    zb = z1 + 0.5
    wall.slab("trim", 0.0, L, z1, zb, 0.0, 0.75, TRIM)  # first-floor balcony, with its balustrade
    wall.slab("trim", 0.0, L, zb, zb + 0.82, 0.60, 0.75, TRIM, ends=False)
    wall.slab("trim", 0.0, L, zb + 0.82, zb + 0.95, 0.56, 0.79, TRIM, ends=False)
    for uc in centres:
        wall.window(uc, zb + 0.15, min(1.25, bw - 0.9), 2.65, hood=0.22)  # corniced, first floor
    z = zb + first_h - 0.5
    for k in range(n - 2):
        h = 1.9 if k < n - 3 else 1.5
        for uc in centres:
            wall.window(uc, z + 0.55, min(1.1, bw - 1.0), h, sill=True, hood=0.16 if k == 0 else None)  # corniced, second floor
        z += upper
        if k == n - 4 or (n <= 3 and k == 0):
            wall.slab("trim", 0.0, L, z - 0.05, z + 0.12, 0.0, 0.10, TRIM)  # string course under the attic storey
    zc = min(top - 0.8, z + 0.1)
    wall.slab("trim", 0.0, L, zc, zc + 0.45, 0.0, 0.45, TRIM)  # main cornice
    if front_room > 1.0:  # basement area and its railings
        depth = min(1.8, front_room - 0.6)
        wall.M["dark"].quad(wall.P(0, 0.0, base + 0.004), wall.P(L, 0.0, base + 0.004), wall.P(L, depth, base + 0.004), wall.P(0, depth, base + 0.004),
                            (0.10, 0.10, 0.10, 1.0))
        cut = (centres[door_bay] - 1.2, centres[door_bay] + 1.2) if L >= 4.5 else (0.0, 0.0)
        wall.railing(0.0, max(0.0, cut[0]), depth, base, 1.05)
        wall.railing(min(L, cut[1]), L, depth, base, 1.05)


def mansion_block(wall, storeys, rng, front_room, brick="brick_red"):
    """Red-brick mansion blocks (Queen Anne / Pont Street Dutch): stone bands at each floor, white-framed sash
    windows, two-storey bay windows every third bay, an arched entrance, Dutch gables on the skyline."""
    L, base, top = wall.L, wall.base, wall.top
    if L < 3.0:
        return
    n = max(2, storeys)
    sh = (top - base - 0.8) / n
    centres, bw = bays(L, 2.6)
    door = len(centres) // 2
    for k in range(n):
        z = base + k * sh
        if k > 0:
            wall.slab("trim", 0.0, L, z - 0.12, z + 0.12, 0.0, 0.07, (0.90, 0.86, 0.78, 1.0))
        for i, uc in enumerate(centres):
            if k == 0 and i == door and L > 6:
                wall.face("dark", uc - 0.75, uc + 0.75, base, base + 2.9, 0.03, (0.12, 0.10, 0.09, 1.0))
                wall.slab("stone", uc - 1.05, uc + 1.05, base + 2.9, base + 3.4, 0.0, 0.25, (0.86, 0.83, 0.76, 1.0))
                continue
            if i % 3 == 1 and 0 < k < n - 1 and bw > 1.9:
                wall.slab(brick, uc - 0.5 * bw + 0.15, uc + 0.5 * bw - 0.15, z + 0.2, z + sh - 0.1, 0.0, 0.55, WHITE)
                wall.face("glass", uc - 0.45, uc + 0.45, z + 0.8, z + 0.8 + min(2.0, sh - 1.1), 0.60, GLASS, uv=((0, 0), (1, 0), (1, 1), (0, 1)))
            else:
                wall.window(uc, z + 0.8, min(1.1, bw - 0.8), min(2.0, sh - 1.1), frame=TRIM)
    for g0 in range(0, len(centres), 3):  # Dutch gables
        u0 = max(0.0, centres[g0] - 0.5 * bw + 0.1)
        u1 = min(L, centres[min(len(centres) - 1, g0 + 2)] + 0.5 * bw - 0.1)
        if u1 - u0 < 3.0 or rng.random() < 0.3:
            continue
        mid = 0.5 * (u0 + u1)
        b_ = wall.M[brick]
        P = wall.P
        b_.tri(P(u0, 0.02, top), P(u1, 0.02, top), P(mid, 0.02, top + 2.6), WHITE, (0, 0), ((u1 - u0), 0), (0.5 * (u1 - u0), 2.6))
        wall.window(mid, top + 0.3, 0.8, 1.2, frame=TRIM, sill=False)


def commercial(wall, storeys, rng, front_room, upper_mat):
    """Brompton Road's shops: shopfronts with stall risers, mullions and fascias (some under awnings) below
    stone or brick upper floors with regular windows, string courses and a cornice."""
    L, base, top = wall.L, wall.base, wall.top
    if L < 3.0:
        return
    gh = 4.4
    units = max(1, int(round(L / 6.5)))
    uw = L / units
    signs = []
    for i in range(units):
        u0, u1 = i * uw + 0.25, (i + 1) * uw - 0.25
        wall.face("dark", u0, u1, base, base + 0.5, 0.03, (0.13, 0.13, 0.14, 1.0))
        wall.face("glass", u0, u1, base + 0.5, base + 3.3, 0.04, (0.95, 0.93, 0.88, 1.0), uv=((0, 0), (max(1, round((u1 - u0) / 1.6)), 0),
                                                                                             (max(1, round((u1 - u0) / 1.6)), 1), (0, 1)))
        col = rng.choice(FASCIA)
        wall.slab("trim", u0, u1, base + 3.35, base + 4.15, 0.0, 0.16, col)
        if rng.random() < 0.55:
            c2 = rng.choice(AWNING)
            b_ = wall.M["trim"]
            P = wall.P
            b_.quad(P(u0, 0.16, base + 3.3), P(u1, 0.16, base + 3.3), P(u1, 1.6, base + 2.7), P(u0, 1.6, base + 2.7), c2)
            b_.quad(P(u0, 1.6, base + 2.7), P(u1, 1.6, base + 2.7), P(u1, 1.6, base + 2.45), P(u0, 1.6, base + 2.45), c2)
        if u1 - u0 > 2.6:
            signs.append((0.5 * (u0 + u1), base + 3.75, col, rng.choice(SIGNS)))
    wall.slab("trim", 0.0, L, base + gh - 0.1, base + gh + 0.15, 0.0, 0.18, (0.88, 0.86, 0.80, 1.0))
    n = max(1, storeys - 1)
    sh = (top - base - gh - 0.8) / n
    centres, bw = bays(L, 3.0)
    for k in range(n):
        z = base + gh + k * sh
        for uc in centres:
            wall.window(uc, z + 0.7, min(1.3, bw - 1.0), min(2.1, sh - 1.0), frame=(0.90, 0.88, 0.82, 1.0), hood=0.12 if k == 0 else None)
    wall.slab("trim", 0.0, L, top - 0.9, top - 0.5, 0.0, 0.40, (0.90, 0.88, 0.82, 1.0))
    return signs


def harrods(wall, storeys, rng, front_room):
    """The Brompton Road frontage of the Harrods building (1901-05, C. W. Stephens; pinky-buff Doulton
    terracotta; listed grade II*, Historic England 1294346): two lower storeys of large display windows, here
    behind dark-green awnings, four storeys above with giant pilasters between the bays, cornice bands, and
    a deep main cornice under the attic. Proportions only; no lettering, relief or emblem is reproduced."""
    L, base, top = wall.L, wall.base, wall.top
    if L < 3.0:
        return
    gh = 5.2
    first = 4.0  # the second storey of display windows
    centres, bw = bays(L, 4.4)
    tc = (1.0, 1.0, 1.0, 1.0)
    for uc in centres:
        wall.face("glass", uc - 0.5 * bw + 0.45, uc + 0.5 * bw - 0.45, base + 0.6, base + 4.0, 0.05, (1.0, 0.93, 0.78, 1.0),
                  uv=((0, 0), (2, 0), (2, 1), (0, 1)))
        b_ = wall.M["trim"]
        P = wall.P
        u0, u1 = uc - 0.5 * bw + 0.35, uc + 0.5 * bw - 0.35
        b_.quad(P(u0, 0.05, base + 4.6), P(u1, 0.05, base + 4.6), P(u1, 1.5, base + 3.95), P(u0, 1.5, base + 3.95), HARRODS_GREEN)
        b_.quad(P(u0, 1.5, base + 3.95), P(u1, 1.5, base + 3.95), P(u1, 1.5, base + 3.7), P(u0, 1.5, base + 3.7), HARRODS_GREEN)
        wall.slab("terracotta", uc + 0.5 * bw - 0.35, uc + 0.5 * bw + 0.35, base, top - 2.6, 0.0, 0.28, tc, ends=True)  # pilaster
    wall.slab("terracotta", 0.0, L, base + gh, base + gh + 0.5, 0.0, 0.35, tc)
    for uc in centres:  # first floor: display windows divided by mullions and transoms
        wall.face("glass", uc - 0.5 * bw + 0.5, uc + 0.5 * bw - 0.5, base + gh + 0.7, base + gh + first - 0.4, 0.05, (1.0, 0.94, 0.82, 1.0),
                  uv=((0, 0), (3, 0), (3, 2), (0, 2)))
    wall.slab("terracotta", 0.0, L, base + gh + first, base + gh + first + 0.45, 0.0, 0.3, tc)
    gh += first
    n = max(2, storeys - 3)
    sh = (top - base - gh - 3.0) / n
    for k in range(n):
        z = base + gh + 0.5 + k * sh
        for uc in centres:
            wall.window(uc, z + 0.6, min(2.0, bw - 1.4), min(2.6, sh - 1.1), frame=(0.88, 0.62, 0.48, 1.0), mat_frame="terracotta",
                        hood=0.18 if k % 2 == 0 else None)
    wall.slab("terracotta", 0.0, L, top - 2.6, top - 1.9, 0.0, 0.9, tc)  # main cornice
    for uc in centres:
        wall.window(uc, top - 1.6, min(1.2, bw - 2.0), 1.1, frame=(0.88, 0.62, 0.48, 1.0), mat_frame="terracotta", sill=False)


def institutional(wall, storeys, rng, front_room, mat):
    """Modern institutional blocks (Imperial College, the Royal College of Art): ribbon windows on every floor."""
    L, base, top = wall.L, wall.base, wall.top
    if L < 3.0:
        return
    n = max(1, storeys)
    sh = (top - base - 0.6) / n
    for k in range(n):
        z = base + k * sh
        wall.face("glass", 0.4, L - 0.4, z + 0.9, z + sh - 0.5, 0.04, (0.92, 0.94, 0.96, 1.0),
                  uv=((0, 0), (max(1, round(L / 1.5)), 0), (max(1, round(L / 1.5)), 1), (0, 1)))
        wall.slab(mat, 0.0, L, z + sh - 0.5, z + sh - 0.3, 0.0, 0.12, WHITE)


def plain(wall, storey, rng):
    """A side or rear wall that can be seen: a sparse grid of windows."""
    L, base, top = wall.L, wall.base, wall.top
    if L < 4.0:
        return
    centres, bw = bays(L, 3.2)
    z = base + 1.0
    while z + 1.6 < top - 0.5:
        for uc in centres:
            if rng.random() < 0.75:
                wall.face("glass", uc - 0.5, uc + 0.5, z, z + 1.5, 0.03, (0.85, 0.87, 0.90, 1.0), uv=((0, 0), (1, 0), (1, 1), (0, 1)))
        z += storey
