"""Render a recorded road_driving episode as an animated GIF and a poster PNG.

Reads only files written by road_drive: the map export (road_drive map R1 --json map.json), and an
episode folder written by `road_drive drive ... --out DIR` in estimated mode, after
`road_drive localise DIR` has added the offline estimator traces (trace_<estimator>.csv).

    python render_demo.py --map map.json --episode DIR --out test-drive.gif --poster test-drive.png [--compare ekf]

Needs matplotlib, numpy and Pillow. Everything drawn comes from the logs: the footprint is the
vehicle's simulated pose, the estimate is what the stack drove on, the ellipse is the estimator's own
position covariance, the kerb returns are the scanner's ranges drawn from the true scanner pose, and the
comparison trace is another estimator replayed offline on the same sensor log.
"""
import argparse
import csv
import io
import json
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import Circle, Ellipse, Polygon, Rectangle  # noqa: E402
from PIL import Image  # noqa: E402

BG = "#0f1419"
ROAD = "#3a4049"
KERB = "#5b6470"
TEXT = "#e6e9ee"
MUTED = "#9aa4b1"
TRUTH = "#f2f4f7"
EKF = "#3fb6b2"
DR = "#e0a43c"
GNSS = "#b48cf2"
ALERT = "#e5534b"
KERBPT = "#f7d774"
MULTI = "#d9822b"
LENGTH, WIDTH, MIRRORS, REAR = 4.682, 2.011, 2.139, 0.842


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def read_manifest(path):
    kv = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            if "=" in line:
                k, v = line.rstrip("\n").split("=", 1)
                kv[k] = v
    return kv


def zones(text):
    out = []
    for item in filter(None, text.split(";")):
        x0, y0, x1, y1 = (float(v) for v in item.split(":"))
        out.append((x0, y0, x1, y1))
    return out


def draw_roads(ax, net):
    hw = net["lane_width"]
    for a, b, ax0, ay0, bx, by in net["streets"]:
        L = math.hypot(bx - ax0, by - ay0)
        ux, uy = (bx - ax0) / L, (by - ay0) / L
        nx, ny = -uy, ux
        s0 = -hw if a >= 0 else 0.0
        s1 = L + hw if b >= 0 else L
        pts = [(ax0 + ux * s + nx * w, ay0 + uy * s + ny * w) for s, w in ((s0, -hw), (s1, -hw), (s1, hw), (s0, hw))]
        ax.add_patch(Polygon(pts, closed=True, facecolor=ROAD, edgecolor="none", zorder=1))
    # Corner kerbs: fill the fillet between perpendicular arms.
    for j, jn in enumerate(net["junctions"]):
        arms = []
        for a, b, ax0, ay0, bx, by in net["streets"]:
            if a == j or b == j:
                ox, oy = (bx, by) if a == j else (ax0, ay0)
                L = math.hypot(ox - jn["x"], oy - jn["y"])
                arms.append(((ox - jn["x"]) / L, (oy - jn["y"]) / L))
        r = jn["kerb"]
        for i in range(len(arms)):
            for k in range(i + 1, len(arms)):
                (a0, a1), (b0, b1) = arms[i], arms[k]
                if abs(a0 * b0 + a1 * b1) > 0.1:
                    continue
                cx, cy = jn["x"] + hw * (a0 + b0), jn["y"] + hw * (a1 + b1)
                fx, fy = cx + r * (a0 + b0), cy + r * (a1 + b1)
                arc = [(fx - r * math.cos(t) * a0 - r * math.sin(t) * b0, fy - r * math.cos(t) * a1 - r * math.sin(t) * b1)
                       for t in np.linspace(0, math.pi / 2, 12)]
                ax.add_patch(Polygon([(cx, cy)] + arc, closed=True, facecolor=ROAD, edgecolor="none", zorder=1))
    # Lane centre dashes for two-way streets, kerb-side edge lines.
    lanes = {l["name"]: l for l in net["lanes"]}
    for l in net["lanes"]:
        a, b = l["name"].split(">")
        if f"{b}>{a}" in lanes:
            mx0, my0 = (l["x0"] + lanes[f"{b}>{a}"]["x1"]) / 2, (l["y0"] + lanes[f"{b}>{a}"]["y1"]) / 2
            mx1, my1 = (l["x1"] + lanes[f"{b}>{a}"]["x0"]) / 2, (l["y1"] + lanes[f"{b}>{a}"]["y0"]) / 2
            ax.plot([mx0, mx1], [my0, my1], color="#c9cfd6", lw=0.6, dashes=(6, 6), zorder=2)


def footprint(x, y, yaw, length=LENGTH, width=WIDTH):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for u, w in ((-REAR, -width / 2), (length - REAR, -width / 2), (length - REAR, width / 2), (-REAR, width / 2)):
        pts.append((x + c * u - s * w, y + s * u + c * w))
    return pts


def draw_car(ax, x, y, yaw, est=None):
    c, s = math.cos(yaw), math.sin(yaw)
    body = Polygon(footprint(x, y, yaw), closed=True, facecolor="#2b3440", edgecolor=TRUTH, lw=1.2, zorder=8)
    ax.add_patch(body)
    for side in (-1, 1):  # mirrors
        u, w = 2.2, side * (MIRRORS / 2)
        ax.add_patch(Circle((x + c * u - s * w, y + s * u + c * w), 0.12, color=TRUTH, zorder=9))
    # Roof sensor dome and corner pods (styling only; the simulated sensors are GNSS, IMU, wheels).
    u = 1.4
    ax.add_patch(Circle((x + c * u, y + s * u), 0.42, facecolor=EKF, edgecolor="#d9f2f1", lw=0.8, zorder=10))
    for u, w in ((LENGTH - REAR - 0.15, 0.8), (LENGTH - REAR - 0.15, -0.8), (-REAR + 0.15, 0.8), (-REAR + 0.15, -0.8)):
        ax.add_patch(Circle((x + c * u - s * w, y + s * u + c * w), 0.13, color="#7fd3cf", zorder=10))
    if est is not None:
        ex, ey, eyaw = est
        ax.add_patch(Polygon(footprint(ex, ey, eyaw), closed=True, fill=False, edgecolor=EKF, lw=1.4, ls=(0, (3, 2)), zorder=11))


def ellipse(ax, x, y, pxx, pxy, pyy, scale, color):
    vals, vecs = np.linalg.eigh(np.array([[pxx, pxy], [pxy, pyy]]))
    vals = np.maximum(vals, 1e-12)
    ang = math.degrees(math.atan2(vecs[1, 1], vecs[0, 1]))
    k = math.sqrt(5.991)  # 95% for 2 dof
    ax.add_patch(Ellipse((x, y), 2 * k * scale * math.sqrt(vals[1]), 2 * k * scale * math.sqrt(vals[0]), angle=ang, fill=False,
                         edgecolor=color, lw=1.4, zorder=12))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True)
    ap.add_argument("--episode", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--poster")
    ap.add_argument("--poster-time", type=float, default=-1.0)
    ap.add_argument("--fps", type=float, default=8.0)
    ap.add_argument("--speedup", type=float, default=4.0)
    ap.add_argument("--idle-speedup", type=float, default=12.0)
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--compare", default="dr", help="offline trace to draw for comparison (trace_<name>.csv)")
    args = ap.parse_args()

    net = json.load(open(args.map, encoding="utf-8"))
    d = args.episode
    man = read_manifest(d + "/manifest.txt")
    truth = read_csv(d + "/truth.csv")
    est = read_csv(d + "/estimates.csv")
    events = read_csv(d + "/events.csv")
    all_sensors = read_csv(d + "/sensors.csv")
    sensors = [r for r in all_sensors if r["sensor"] == "GNSS"]
    kerb_scans = [r for r in all_sensors if r["sensor"] == "KERB"]
    KT = np.array([float(r["t_meas"]) for r in kerb_scans]) if kerb_scans else np.array([])
    try:
        dr = read_csv(d + "/trace_%s.csv" % args.compare)
    except FileNotFoundError:
        dr = []
    names = {"dr": "dead reckoning", "ekf": "previous EKF (10 s reset)", "ekf_int": "EKF with GNSS integrity",
             "gnss_cv": "GNSS-only filter", "gnss_hold": "GNSS fix held"}
    compare_label = names.get(args.compare, args.compare) + ", same log"
    driver = man.get("estimator", "ekf")
    driver_label = {"ekf": "a GNSS + IMU + wheel EKF", "ekf_int": "a GNSS + IMU + wheel EKF with GNSS integrity",
                    "ekf_kerb": "a GNSS + IMU + wheel + kerb-ranging EKF"}.get(driver, driver)
    T = np.array([float(r["t"]) for r in truth])
    TX = np.array([float(r["x"]) for r in truth])
    TY = np.array([float(r["y"]) for r in truth])
    TYAW = np.array([float(r["yaw"]) for r in truth])
    TV = np.array([float(r["v"]) for r in truth])
    EX = np.array([float(r["x"]) for r in est])
    EY = np.array([float(r["y"]) for r in est])
    EYAW = np.array([float(r["yaw"]) for r in est])
    ESIG = np.array([float(r["sigma_pos"]) for r in est])
    EERR = np.hypot(EX - TX[: len(EX)], EY - TY[: len(EY)])
    DX = np.array([float(r["x"]) for r in dr]) if dr else None
    DY = np.array([float(r["y"]) for r in dr]) if dr else None
    DERR = np.hypot(DX - TX[: len(DX)], DY - TY[: len(DY)]) if dr else None
    outages = []
    for item in filter(None, man.get("outage_windows", "").split(",")):
        a, b = item.split(":")
        outages.append((float(a), float(b)))
    zone_out = zones(man.get("zones_outage", ""))
    zone_mp = zones(man.get("zones_multipath", "")) + zones(man.get("zones_ramp", ""))
    gx = np.array([float(r["a"]) for r in sensors])
    gy = np.array([float(r["b"]) for r in sensors])
    gt = np.array([float(r["t_meas"]) for r in sensors])
    closure_lane = man.get("closure_lane", "")
    lane = next((l for l in net["lanes"] if l["name"] == closure_lane), None)
    barrier_s = float(man.get("barrier_lane_s", "0"))
    works = float(man.get("works_length", "0"))
    t_notified = float(man.get("t_notified", "-1"))
    t_lift = float(man.get("t_lift", "-1"))
    has_closure = man.get("closure", "0") == "1" and lane is not None

    # Frame times: real-time multiple while moving, faster while parked.
    times, t = [], T[0]
    while t <= T[-1]:
        times.append(t)
        i = min(len(T) - 1, int(round(t / 0.02)))
        idle = TV[i] < 0.05 and t > 3.0
        t += (args.idle_speedup if idle else args.speedup) / args.fps
    if times[-1] < T[-1]:
        times.append(T[-1])

    W = args.width
    H = int(W * 9 / 16)
    dpi = 100
    frames = []
    xs = [jn["x"] for jn in net["junctions"]]
    ys = [jn["y"] for jn in net["junctions"]]
    # Pre-render the overview background once.
    ov_fig = plt.figure(figsize=(3.2, 3.2 * (max(ys) - min(ys) + 40) / (max(xs) - min(xs) + 120)), dpi=dpi, facecolor=BG)

    def render(t, poster=False):
        i = min(len(T) - 1, int(round(t / 0.02)))
        fig = plt.figure(figsize=(W / dpi, H / dpi), dpi=dpi, facecolor=BG)
        # Follow view.
        ax = fig.add_axes([0.01, 0.06, 0.60, 0.84])
        ax.set_facecolor(BG)
        draw_roads(ax, net)
        for (x0, y0, x1, y1) in zone_out:
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor="#6d4bc2", alpha=0.16, edgecolor="none", zorder=3))
        for (x0, y0, x1, y1) in zone_mp:
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor=MULTI, alpha=0.10, edgecolor="none", zorder=3))
        present = has_closure and (t_lift < 0 or t < t_lift)
        if present:
            dx, dy = lane["x1"] - lane["x0"], lane["y1"] - lane["y0"]
            L = math.hypot(dx, dy)
            ux, uy = dx / L, dy / L
            bx, by = lane["x0"] + ux * barrier_s, lane["y0"] + uy * barrier_s
            nx, ny = -uy, ux
            pts = [(bx + nx * 1.5, by + ny * 1.5), (bx - nx * 1.5, by - ny * 1.5), (bx - nx * 1.5 + ux * works, by - ny * 1.5 + uy * works),
                   (bx + nx * 1.5 + ux * works, by + ny * 1.5 + uy * works)]
            ax.add_patch(Polygon(pts, closed=True, facecolor="#d9822b", alpha=0.35, edgecolor="none", zorder=4))
            ax.plot([bx + nx * 1.5, bx - nx * 1.5], [by + ny * 1.5, by - ny * 1.5], color=ALERT, lw=4, solid_capstyle="butt", zorder=5)
        k = max(0, i - 1)
        ax.plot(TX[: k + 1], TY[: k + 1], color=TRUTH, lw=1.2, zorder=6)
        if DX is not None:
            ax.plot(DX[: k + 1], DY[: k + 1], color=DR, lw=1.0, ls=(0, (2, 2)), zorder=6)
        ax.plot(EX[: k + 1], EY[: k + 1], color=EKF, lw=1.3, zorder=7)
        recent = (gt <= t) & (gt > t - 3.0)
        ax.scatter(gx[recent], gy[recent], s=6, color=GNSS, zorder=7)
        if len(KT):
            j = int(np.searchsorted(KT, t, side="right")) - 1
            if j >= 0:
                kr = kerb_scans[j]
                ti = min(len(T) - 1, int(round(float(kr["t_meas"]) / 0.02)))
                mx, b0, db = float(kr["c"]), float(kr["a"]), float(kr["b"])
                sxp, syp = TX[ti] + mx * math.cos(TYAW[ti]), TY[ti] + mx * math.sin(TYAW[ti])
                pts = []
                for n_, rv in enumerate(kr["ranges"].split()):
                    r_ = float(rv)
                    if r_ > 0:
                        th = TYAW[ti] + b0 + n_ * db
                        pts.append((sxp + r_ * math.cos(th), syp + r_ * math.sin(th)))
                if pts:
                    ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=7, color=KERBPT, zorder=13)
        draw_car(ax, TX[i], TY[i], TYAW[i], (EX[i], EY[i], EYAW[i]))
        e = est[i]
        ellipse(ax, EX[i], EY[i], float(e["pxx"]), float(e["pxy"]), float(e["pyy"]), 4.0, EKF)
        half = 20.0
        ax.set_xlim(TX[i] - half, TX[i] + half)
        ax.set_ylim(TY[i] - half * 0.84 * H / (0.60 * W), TY[i] + half * 0.84 * H / (0.60 * W))
        sx, sy = TX[i] - half + 2.0, TY[i] - half * 0.84 * H / (0.60 * W) + 2.0
        ax.plot([sx, sx + 10.0], [sy, sy], color=MUTED, lw=2, zorder=20)
        ax.text(sx + 5.0, sy + 0.8, "10 m", color=MUTED, fontsize=7, ha="center", zorder=20)
        ax.set_aspect("equal")
        ax.axis("off")
        # Overview.
        ov = fig.add_axes([0.62, 0.55, 0.37, 0.35])
        ov.set_facecolor(BG)
        draw_roads(ov, net)
        for (x0, y0, x1, y1) in zone_out:
            ov.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor="#6d4bc2", alpha=0.3, edgecolor="none", zorder=3))
        ov.plot(TX[: i + 1], TY[: i + 1], color=TRUTH, lw=0.8, zorder=4)
        ov.add_patch(Circle((TX[i], TY[i]), 5, color=EKF, zorder=5))
        if present:
            ov.add_patch(Circle((bx, by), 4, color=ALERT, zorder=5))
        ov.set_xlim(min(xs) - 15, max(xs) + 70)
        ov.set_ylim(min(ys) - 15, max(ys) + 15)
        ov.set_aspect("equal")
        ov.axis("off")
        # Error plot.
        ep = fig.add_axes([0.66, 0.08, 0.32, 0.165])
        ep.set_facecolor(BG)
        for a, b in outages:
            ep.axvspan(a, min(b, T[-1]), color="#6d4bc2", alpha=0.25, lw=0)
        upto = slice(0, i + 1)
        ep.plot(T[upto], np.maximum(EERR[upto], 0.005), color=EKF, lw=1.2, label="error")
        ep.plot(T[upto], np.maximum(ESIG[upto], 0.005), color=EKF, lw=0.8, ls=":", label="1-sigma")
        if DERR is not None:
            ep.plot(T[upto], np.maximum(DERR[upto], 0.005), color=DR, lw=1.0, ls=(0, (2, 2)), label=names.get(args.compare, args.compare))
        ep.set_xlim(0, T[-1])
        ep.set_yscale("log")
        ep.set_ylim(0.01, 20.0)
        ep.set_ylabel("position error (m)", color=MUTED, fontsize=7)
        ep.tick_params(colors=MUTED, labelsize=6)
        for sp in ep.spines.values():
            sp.set_color("#2e3640")
        ep.legend(loc="upper left", fontsize=6, frameon=False, labelcolor=MUTED, ncol=3)
        # Status.
        in_outage = any(a <= t <= b for a, b in outages)
        status = e["status"]
        last = [ev for ev in events if float(ev["t"]) <= t and ev["type"] in (
            "CLOSURE_NOTIFIED", "STOP_PLANNED", "STOPPED", "CLOSURE_LIFTED", "RECOVERED", "GOAL_REACHED", "LOCALISATION_DEGRADED",
            "LOCALISATION_NOMINAL", "LOCALISATION_UNRELIABLE", "LOCALISATION_RECOVERED", "REPLAN", "NO_CLEAR_CONTINUATION",
            "CLEAR_CONTINUATION", "MONITOR_TRIGGER")]
        lines = [
            ("t", f"{t:5.1f} s"),
            ("speed", f"{TV[i] * 3.6:5.1f} km/h"),
            ("error", f"{EERR[i]:5.2f} m"),
            ("1-sigma", f"{ESIG[i]:5.2f} m"),
            ("GNSS integrity", e.get("gnss", "-").replace("_", " ").lower()),
            ("kerb match", e.get("kerb", "-").lower()),
            ("path shift", f"{float(e.get('shift', '0') or 0):+5.2f} m"),
        ]
        y0 = 0.505
        for kk, (label, value) in enumerate(lines):
            fig.text(0.63, y0 - kk * 0.031, label, color=MUTED, fontsize=8)
            fig.text(0.76, y0 - kk * 0.031, value, color=TEXT, fontsize=8, family="monospace")
        gnss_ok = not in_outage and e.get("gnss", "TRUSTED") in ("TRUSTED", "-")
        for kk, (name, ok) in enumerate((("GNSS", gnss_ok), ("IMU", True), ("WHEEL", True), ("KERB", e.get("kerb", "-") != "NONE"))):
            fig.text(0.87, 0.505 - kk * 0.031, ("● " if ok else "○ ") + name, color=(EKF if ok else ALERT), fontsize=8)
        if last:
            ev = last[-1]
            fig.text(0.63, 0.272, f"{float(ev['t']):.1f} s  {ev['type'].replace('_', ' ').lower()}", color=TEXT, fontsize=7.5)
        # Titles and labels.
        fig.text(0.01, 0.955, "Autonomous Navigation Engine · road_driving", color=TEXT, fontsize=12, weight="bold")
        fig.text(0.01, 0.92, "Independent research simulation · simulated I-PACE-sized proxy on synthetic left-hand-traffic roads · "
                             "the stack drives on " + driver_label, color=MUTED, fontsize=8)
        fig.text(0.01, 0.02, "white: true footprint \u00b7 teal: estimate (dashed) and 95% ellipse x4 \u00b7 amber: " + compare_label +
                 " \u00b7 violet: GNSS fixes, outage \u00b7 orange: GNSS fault zone \u00b7 yellow: kerb returns \u00b7 red: barrier",
                 color=MUTED, fontsize=6.3)
        buf = io.BytesIO()
        fig.savefig(buf, format="png", dpi=dpi, facecolor=BG)
        plt.close(fig)
        buf.seek(0)
        return Image.open(buf).convert("RGB")

    plt.close(ov_fig)
    for n, t in enumerate(times):
        frames.append(render(t))
    pal = frames[len(frames) // 2].quantize(colors=96, method=Image.Quantize.MEDIANCUT)
    gif = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in frames]
    gif[0].save(args.out, save_all=True, append_images=gif[1:], duration=int(1000 / args.fps), loop=0, optimize=True)
    if args.poster:
        pt = args.poster_time if args.poster_time >= 0 else times[len(times) // 2]
        render(pt).save(args.poster, optimize=True)
    print(f"{len(frames)} frames, {T[-1]:.1f} s of simulation -> {args.out}")


if __name__ == "__main__":
    main()
