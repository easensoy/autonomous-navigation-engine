"""Scene geometry from the simulation's own map, in pure Python (no Panda3D), so it can be tested anywhere.

Everything the simulation measures against comes from the map message the backend sends: the kerb is the
exact set of kerb lines and corner arcs that `LaneNetwork::roadClearance` and the kerb scanner use. The
carriageway is the region on the carriageway side of those kerbs; it is triangulated later from the loops
built here. Decorative scenery (buildings, street furniture) is placed only on the pavement side, clear of
the kerb, and is never sent back to the simulation.

Frames: the map frame of the simulation (x east, y north, metres); z up.
"""
import math

KERB_HEIGHT = 0.12      # m, drawn kerb upstand (visual; the simulation's kerb is a line in the plane)
ARC_STEP = math.radians(6.0)


def _close(a, b, tol=1e-3):
    return abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol


def merged_kerb_lines(m):
    """The kerb lines with collinear overlapping pieces merged (an outer kerb is built per street and runs
    across the junction box, so neighbouring streets' pieces overlap there). Same geometry, fewer pieces."""
    groups = {}
    for ax, ay, bx, by, nx, ny in m["kerb_lines"]:
        key = (round(nx, 4) + 0.0, round(ny, 4) + 0.0, round(nx * ax + ny * ay, 3) + 0.0)
        dx, dy = ny, -nx  # direction with the carriageway on the left
        s0, s1 = sorted((ax * dx + ay * dy, bx * dx + by * dy))
        groups.setdefault(key, []).append((s0, s1, nx, ny))
    out = []
    for (_, _, c), items in groups.items():
        items.sort()
        nx, ny = items[0][2], items[0][3]
        dx, dy = ny, -nx
        cur0, cur1 = items[0][0], items[0][1]
        for s0, s1, _, _ in items[1:] + [(None, None, None, None)]:
            if s0 is not None and s0 <= cur1 + 1e-3:
                cur1 = max(cur1, s1)
                continue
            # Points on the line: p = c * n + s * d.
            out.append((c * nx + cur0 * dx, c * ny + cur0 * dy, c * nx + cur1 * dx, c * ny + cur1 * dy, nx, ny))
            if s0 is not None:
                cur0, cur1 = s0, s1
    return out


def kerb_primitives(m):
    """Each kerb line and arc as a polyline traversed with the carriageway on its left. A district network's
    kerb pieces meet end to end at any angle and never overlap, so they are used as they are (merging relies
    on the grids' exactly axis-aligned normals); a district's patch edges are included, as they close it."""
    prims = []
    if m.get("district"):
        for ax, ay, bx, by, nx, ny in m["kerb_lines"]:
            dx, dy = ny, -nx  # the direction with the carriageway on the left
            prims.append([(ax, ay), (bx, by)] if (bx - ax) * dx + (by - ay) * dy >= 0 else [(bx, by), (ax, ay)])
    else:
        for ax, ay, bx, by, nx, ny in merged_kerb_lines(m):
            prims.append([(ax, ay), (bx, by)])  # merged lines already run with the carriageway on the left
    for cx, cy, r, mid, half in m["kerb_arcs"]:
        # The carriageway lies outside the circle: clockwise around the centre keeps it on the left.
        n = max(2, int(math.ceil(2.0 * half / ARC_STEP)))
        a0, a1 = mid + half, mid - half
        prims.append([(cx + r * math.cos(a0 + (a1 - a0) * i / n), cy + r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)])
    return prims


def signed_area(pts):
    s = 0.0
    for i in range(len(pts)):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % len(pts)]
        s += x0 * y1 - x1 * y0
    return 0.5 * s


def kerb_loops(m):
    """Chains the kerb into closed loops, carriageway on the left.

    Returns (outer, holes): loops with positive area bound carriageway (the network's outer kerb), loops
    with negative area bound pavement islands (city blocks). Raises ValueError if the kerb does not close.
    """
    prims = kerb_primitives(m)
    unused = list(range(len(prims)))
    loops = []
    while unused:
        first = unused.pop(0)
        loop = list(prims[first][:-1])
        end = prims[first][-1]
        start = prims[first][0]
        guard = 0
        while not _close(end, start):
            guard += 1
            if guard > len(prims) + 1:
                raise ValueError("kerb does not close near (%.2f, %.2f)" % end)
            nxt = next((j for j in unused if _close(prims[j][0], end)), None)
            if nxt is None:
                raise ValueError("kerb has a gap at (%.2f, %.2f)" % end)
            unused.remove(nxt)
            loop.extend(prims[nxt][:-1])
            end = prims[nxt][-1]
        loops.append(loop)
    outer = [lp for lp in loops if signed_area(lp) > 0]
    holes = [lp for lp in loops if signed_area(lp) < 0]
    return outer, holes


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


def on_carriageway(x, y, outer, holes):
    """Inside the network's outer kerb and outside every block: on the road."""
    return any(point_in_polygon(x, y, lp) for lp in outer) and not any(point_in_polygon(x, y, h) for h in holes)


def distance_to_kerb(x, y, m):
    """Unsigned distance from a point to the nearest kerb line or arc (the map's exact kerb)."""
    best = 1e18
    for ax, ay, bx, by, _, _ in m["kerb_lines"]:
        vx, vy = bx - ax, by - ay
        L2 = vx * vx + vy * vy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / L2))
        best = min(best, math.hypot(x - ax - t * vx, y - ay - t * vy))
    for cx, cy, r, mid, half in m["kerb_arcs"]:
        a = math.atan2(y - cy, x - cx)
        d = (a - mid + math.pi) % (2.0 * math.pi) - math.pi
        if abs(d) <= half:
            best = min(best, abs(math.hypot(x - cx, y - cy) - r))
        else:
            for e in (mid - half, mid + half):
                best = min(best, math.hypot(x - cx - r * math.cos(e), y - cy - r * math.sin(e)))
    return best


def offset_loop(loop, dist):
    """The loop moved `dist` to its right (the pavement side), with mitred corners."""
    n = len(loop)
    out = []
    for i in range(n):
        p0, p1, p2 = loop[i - 1], loop[i], loop[(i + 1) % n]
        d0 = (p1[0] - p0[0], p1[1] - p0[1])
        d1 = (p2[0] - p1[0], p2[1] - p1[1])
        l0 = math.hypot(*d0) or 1.0
        l1 = math.hypot(*d1) or 1.0
        n0 = (d0[1] / l0, -d0[0] / l0)  # right normal
        n1 = (d1[1] / l1, -d1[0] / l1)
        bx, by = n0[0] + n1[0], n0[1] + n1[1]
        bl = math.hypot(bx, by)
        if bl < 1e-9:
            bx, by, scale = n1[0], n1[1], 1.0
        else:
            bx, by = bx / bl, by / bl
            cos_half = max(0.3, bx * n1[0] + by * n1[1])
            scale = 1.0 / cos_half
        out.append((p1[0] + bx * dist * scale, p1[1] + by * dist * scale))
    return out


def bounds(m, margin=60.0):
    xs = [j["x"] for j in m["junctions"]]
    ys = [j["y"] for j in m["junctions"]]
    for ax, ay, bx, by, _, _ in m["kerb_lines"]:
        xs += [ax, bx]
        ys += [ay, by]
    return min(xs) - margin, min(ys) - margin, max(xs) + margin, max(ys) + margin


def two_way(m):
    """Street index -> True when lanes run both ways along it."""
    pairs = {(l["from"], l["to"]) for l in m["lanes"]}
    return [((s[0], s[1]) in pairs and (s[1], s[0]) in pairs) for s in m["streets"]]


def centre_dashes(m, dash=3.0, gap=6.0):
    """White centre-line dashes on two-way streets, between the junction mouths (segments in the plane)."""
    mouth = m["mouth"]
    out = []
    for street, both in zip(m["streets"], two_way(m)):
        if not both:
            continue
        _, b, ax, ay, bx, by = street
        L = math.hypot(bx - ax, by - ay)
        if L < 1e-6:
            continue
        ux, uy = (bx - ax) / L, (by - ay) / L
        s = mouth + 2.0
        end = L - (mouth + 2.0 if b >= 0 else 4.0)
        while s + dash <= end:
            out.append(((ax + ux * s, ay + uy * s), (ax + ux * (s + dash), ay + uy * (s + dash))))
            s += dash + gap
    return out


def one_way_arrows(m, spacing=40.0):
    """Arrow positions (x, y, heading) along one-way lanes, in their direction of travel."""
    mouth = m["mouth"]
    out = []
    both = two_way(m)
    oneway_pairs = {(s[0], s[1]) for s, b in zip(m["streets"], both) if not b} | {(s[1], s[0]) for s, b in zip(m["streets"], both) if not b}
    for l in m["lanes"]:
        if (l["from"], l["to"]) not in oneway_pairs or l["from"] < 0 or l["to"] < 0:
            continue
        L = math.hypot(l["x1"] - l["x0"], l["y1"] - l["y0"])
        ux, uy = (l["x1"] - l["x0"]) / L, (l["y1"] - l["y0"]) / L
        s = 12.0
        while s < L - 10.0:
            out.append((l["x0"] + ux * s, l["y0"] + uy * s, math.atan2(uy, ux)))
            s += spacing
    return out


def _hash01(*vals):
    h = 2166136261
    for v in vals:
        for ch in ("%.2f" % v).encode():
            h = ((h ^ ch) * 16777619) & 0xFFFFFFFF
    return (h & 0xFFFFFF) / float(0xFFFFFF)


def frontage(m, pavement=3.2, depth_range=(9.0, 12.0)):
    """Terraced building plots along every straight kerb, on the pavement side, clear of corners.

    Returns dicts: x, y (front-centre), heading (facing the street, i.e. the kerb normal), width, depth,
    storeys, style. Plots end before a corner by at least their depth, so buildings on two sides of a block
    never overlap; any plot whose footprint would come within `pavement` of another kerb is dropped.
    """
    plots = []
    for ax, ay, bx, by, nx, ny in m["kerb_lines"]:
        L = math.hypot(bx - ax, by - ay)
        if L < 20.0:
            continue
        ux, uy = (bx - ax) / L, (by - ay) / L
        px, py = -nx, -ny  # into the pavement
        margin = depth_range[1] + 2.0
        s = margin
        while s < L - margin:
            w = 5.0 + 2.5 * _hash01(ax, ay, s)
            if s + w > L - margin:
                break
            d = depth_range[0] + (depth_range[1] - depth_range[0]) * _hash01(bx, by, s)
            cx = ax + ux * (s + 0.5 * w) + px * pavement
            cy = ay + uy * (s + 0.5 * w) + py * pavement
            corners = [(cx + ux * 0.5 * w * a + px * d * b, cy + uy * 0.5 * w * a + py * d * b) for a in (-1, 1) for b in (0, 1)]
            if all(distance_to_kerb(x, y, m) > pavement - 0.05 for x, y in corners):
                plots.append({"x": cx, "y": cy, "heading": math.atan2(ny, nx), "width": w, "depth": d,
                              "storeys": 2 + int(3 * _hash01(cx, cy)), "style": int(4 * _hash01(cy, cx)) % 4})
            s += w
    return plots


def furniture(m, spacing=28.0):
    """Lamp posts and trees along the pavements, bollards at corners: (kind, x, y, heading)."""
    items = []
    for ax, ay, bx, by, nx, ny in m["kerb_lines"]:
        L = math.hypot(bx - ax, by - ay)
        if L < 24.0:
            continue
        ux, uy = (bx - ax) / L, (by - ay) / L
        k = 0
        s = 12.0
        while s < L - 12.0:
            items.append(("lamp", ax + ux * s - nx * 0.6, ay + uy * s - ny * 0.6, math.atan2(ny, nx)))
            if k % 2 == 1:
                items.append(("tree", ax + ux * (s + 0.5 * spacing) - nx * 1.6, ay + uy * (s + 0.5 * spacing) - ny * 1.6, 0.0))
            s += spacing
            k += 1
    for cx, cy, r, mid, half in m["kerb_arcs"]:
        for a in (mid - 0.5 * half, mid + 0.5 * half):
            items.append(("bollard", cx + (r - 0.5) * math.cos(a), cy + (r - 0.5) * math.sin(a), 0.0))
    return [it for it in items if distance_to_kerb(it[1], it[2], m) > 0.3]


def footprint(x, y, yaw, length, width, rear):
    """The vehicle's rectangle (rear-axle pose), counter-clockwise."""
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for fx, fy in ((-rear, -0.5 * width), (length - rear, -0.5 * width), (length - rear, 0.5 * width), (-rear, 0.5 * width)):
        pts.append((x + c * fx - s * fy, y + s * fx + c * fy))
    return pts


def covariance_ellipse(pxx, pxy, pyy, scale, n=48):
    """Points of the ellipse x^T P^-1 x = scale^2, relative to the mean."""
    tr = pxx + pyy
    det = pxx * pyy - pxy * pxy
    disc = math.sqrt(max(0.0, 0.25 * tr * tr - det))
    l1, l2 = 0.5 * tr + disc, max(0.0, 0.5 * tr - disc)
    ang = 0.5 * math.atan2(2.0 * pxy, pxx - pyy)
    a, b = scale * math.sqrt(max(l1, 0.0)), scale * math.sqrt(l2)
    ca, sa = math.cos(ang), math.sin(ang)
    return [(a * math.cos(t) * ca - b * math.sin(t) * sa, a * math.cos(t) * sa + b * math.sin(t) * ca) for t in (2.0 * math.pi * i / n for i in range(n + 1))]


def scan_points(scan, pose=None):
    """End points of a kerb scan's returns from a rear-axle pose (default: the true pose it was taken at)."""
    x, y, yaw = pose if pose is not None else scan["true_pose"]
    sx, sy = x + scan["mount_x"] * math.cos(yaw), y + scan["mount_x"] * math.sin(yaw)
    pts = []
    for i, r in enumerate(scan["ranges"]):
        if r is None or r <= 0:
            continue
        th = yaw + scan["first_bearing"] + i * scan["bearing_step"]
        pts.append((sx + r * math.cos(th), sy + r * math.sin(th)))
    return (sx, sy), pts
