"""A diagnostic timeline of one estimated-pose episode: where the estimate's error lies against the filter's own
uncertainty, what GNSS and the kerbs were doing, and how close the body came to the kerb.

    python road_driving/tools/sf_timeline.py EPISODE_DIR SCENE_JSON OUT.png [--title TEXT] [--json OUT.json]

EPISODE_DIR is what `road_drive drive ... --mode estimated --out DIR` writes (truth.csv, estimates.csv, events.csv,
sensors.csv, manifest.txt); SCENE_JSON is `road_drive map SF-SOMA --scene FILE` (the map as the viewer gets it: the
world's kerb lines and arcs). Panels, against simulated time:
1. the error along the true heading and across it (lateral), with the filter's own 2-sigma band projected on the
   same axes (its covariance rotated into the true heading's frame); the GNSS fault window shaded;
2. the filter's GNSS integrity state and kerb-match state, and the stack's pose corrections and fallback stops;
3. the route's curvature (from the true path) and the true speed;
4. the body's clearance to the world's kerb (the evaluator's footprint, without mirrors), and its minimum.
Errors are projected on the true heading at each sample, so they are defined in junctions without any route
association: along-road error that the kerbs of a straight cannot see shows as the along trace, and becomes lateral
when the heading turns. Truth is used here only for this offline diagnosis.
Needs matplotlib and numpy.
"""
import csv
import json
import math
import pathlib
import sys

GNSS_LEVEL = {"TRUSTED": 0, "-": 0, "OUTAGE": 1, "INCONSISTENT": 2, "FAULTED": 3}
KERB_LEVEL = {"MATCHED": 0, "-": 1, "NONE": 1, "REJECTED": 2}


def kv(path):
    out = {}
    for line in pathlib.Path(path).read_text(encoding="utf-8").splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            out[k] = v
    return out


def footprint_clearance(g, m, loops, x, y, yaw, rear, front, width):
    """The evaluator's footprint clearance: the smallest signed distance to the kerb (patch edges included) over points
    along the body's edges, signed by an even-odd test against the closed kerb loops (the viewer's geometry module)."""
    c, s, hw = math.cos(yaw), math.sin(yaw), 0.5 * width
    best = float("inf")
    pts = [(-rear + (front + rear) * i / 8.0, w) for i in range(9) for w in (-hw, hw)]
    pts += [(u, -hw + width * i / 4.0) for i in range(1, 4) for u in (-rear, front)]
    for u, w in pts:
        px, py = x + c * u - s * w, y + s * u + c * w
        d = g.distance_to_kerb(px, py, m)
        best = min(best, d if g.on_carriageway(px, py, loops[0], loops[1]) else -d)
    return best


def main(argv):
    args = [a for a in argv[1:] if not a.startswith("--")]
    opts = {argv[i]: argv[i + 1] for i in range(1, len(argv) - 1) if argv[i].startswith("--")}
    if len(args) < 3:
        print(__doc__)
        return 2
    args = [a for a in args if a not in opts.values()]
    ep, scene_path, out = pathlib.Path(args[0]), pathlib.Path(args[1]), pathlib.Path(args[2])
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "viewer"))
    from ane_viewer import geometry as g  # pure Python: the kerb loops and distances the viewer draws from

    man = kv(ep / "manifest.txt")
    truth = list(csv.DictReader(open(ep / "truth.csv", encoding="utf-8")))
    est = list(csv.DictReader(open(ep / "estimates.csv", encoding="utf-8")))
    events = list(csv.DictReader(open(ep / "events.csv", encoding="utf-8")))
    m = json.loads(scene_path.read_text(encoding="utf-8"))
    m = m.get("map", m)
    vp = dict(item.split("=", 1) for item in man.get("vehicle_params", "").split(";") if "=" in item)
    rear, front = float(vp.get("rear_overhang", 0.842)), float(vp.get("wheelbase", 2.99)) + float(vp.get("front_overhang", 0.85))
    width = float(vp.get("width", 2.011))
    n = min(len(truth), len(est))
    t = np.array([float(truth[i]["t"]) for i in range(n)])
    tx = np.array([float(truth[i]["x"]) for i in range(n)])
    ty = np.array([float(truth[i]["y"]) for i in range(n)])
    tyaw = np.array([float(truth[i]["yaw"]) for i in range(n)])
    tv = np.array([float(truth[i]["v"]) for i in range(n)])
    valid = np.array([est[i]["status"] != "INIT" and float(est[i]["sigma_pos"]) >= 0.0 for i in range(n)])
    ex = np.array([float(est[i]["x"]) for i in range(n)])
    ey = np.array([float(est[i]["y"]) for i in range(n)])
    pxx = np.array([float(est[i]["pxx"]) for i in range(n)])
    pxy = np.array([float(est[i]["pxy"]) for i in range(n)])
    pyy = np.array([float(est[i]["pyy"]) for i in range(n)])
    c, s = np.cos(tyaw), np.sin(tyaw)
    dx, dy = ex - tx, ey - ty
    along, lat = dx * c + dy * s, -dx * s + dy * c
    sig_along = np.sqrt(np.maximum(0.0, c * c * pxx + 2 * c * s * pxy + s * s * pyy))
    sig_lat = np.sqrt(np.maximum(0.0, s * s * pxx - 2 * c * s * pxy + c * c * pyy))
    gnss = np.array([GNSS_LEVEL.get(est[i]["gnss"], 0) for i in range(n)])
    kerb = np.array([KERB_LEVEL.get(est[i]["kerb"], 1) for i in range(n)])
    # Curvature of the true path: heading change per metre travelled, over 0.5 m or more.
    curv = np.zeros(n)
    for i in range(1, n):
        ds = math.hypot(tx[i] - tx[i - 1], ty[i] - ty[i - 1])
        if ds > 1e-3:
            curv[i] = ((tyaw[i] - tyaw[i - 1] + math.pi) % (2 * math.pi) - math.pi) / ds
    loops = g.kerb_loops(m)
    clearance = np.array([footprint_clearance(g, m, loops, tx[i], ty[i], tyaw[i], rear, front, width) if i % 5 == 0 else np.nan for i in range(n)])
    windows = [tuple(float(v) for v in w.split(":")) for w in man.get("outage_windows", "").split(",") if ":" in w]

    fig, ax = plt.subplots(4, 1, figsize=(12.0, 10.5), sharex=True, gridspec_kw={"height_ratios": [3.0, 1.2, 1.2, 1.6]})
    title = opts.get("--title") or "%s, %s, seed %s, %s, %s" % (man.get("scenario"), man.get("condition"), man.get("seed"), man.get("estimator"),
                                                                 man.get("vehicle_profile", "ipace_proxy_v1"))
    fig.suptitle(title + "  -  independent simulation", fontsize=11)
    for a in ax:
        for w0, w1 in windows:
            a.axvspan(w0, w1, color="#5566ff", alpha=0.08, lw=0)
    a = ax[0]
    a.plot(t[valid], along[valid], color="#1f77b4", lw=1.3, label="along-road error (true heading)")
    a.plot(t[valid], lat[valid], color="#d62728", lw=1.3, label="lateral error")
    a.fill_between(t[valid], -2 * sig_along[valid], 2 * sig_along[valid], color="#1f77b4", alpha=0.12, lw=0, label="filter's 2-sigma along")
    a.fill_between(t[valid], -2 * sig_lat[valid], 2 * sig_lat[valid], color="#d62728", alpha=0.12, lw=0, label="filter's 2-sigma across")
    a.axhline(0.0, color="k", lw=0.5)
    a.set_ylabel("error against truth (m)")
    a.legend(loc="lower left", fontsize=8, ncol=2)  # clear of the fault window's label at the top
    if windows:
        a.text(windows[0][0], a.get_ylim()[1] * 0.92, " GNSS outage", color="#3344cc", fontsize=8)
    a = ax[1]
    a.step(t, gnss, where="post", color="#7a3db8", lw=1.2, label="GNSS integrity")
    a.step(t, kerb - 4.5, where="post", color="#2ca02c", lw=1.2, label="kerb match")
    a.set_yticks([0, 1, 2, 3, -4.5, -3.5, -2.5])
    a.set_yticklabels(["trusted", "outage", "inconsistent", "faulted", "kerb matched", "no scan", "kerb rejected"], fontsize=7)
    for e in events:
        if e["type"] in ("POSE_CORRECTION", "FALLBACK_STOP", "LOCALISATION_UNRELIABLE", "NO_CLEAR_CONTINUATION"):
            col = {"POSE_CORRECTION": "#ff7f0e", "FALLBACK_STOP": "#000000"}.get(e["type"], "#8c564b")
            for b_ in ax:
                b_.axvline(float(e["t"]), color=col, lw=0.8, ls="--", alpha=0.7)
            a.text(float(e["t"]), 3.3, e["type"].lower().replace("_", " "), fontsize=6, rotation=90, va="bottom", color=col)
    a = ax[2]
    a.plot(t, curv, color="#444444", lw=1.0)
    a.set_ylabel("curvature (1/m)")
    a2 = a.twinx()
    a2.plot(t, tv, color="#17becf", lw=1.0)
    a2.set_ylabel("speed (m/s)", color="#17becf")
    a = ax[3]
    ok = ~np.isnan(clearance)
    a.plot(t[ok], clearance[ok], color="#9467bd", lw=1.2, label="body clearance to the world's kerb")
    a.axhline(0.0, color="r", lw=0.8)
    i_min = int(np.nanargmin(clearance))
    a.annotate("min %.2f m" % clearance[i_min], (t[i_min], clearance[i_min]), fontsize=8, xytext=(5, 5), textcoords="offset points")
    a.set_ylabel("clearance (m)")
    a.set_xlabel("simulated time (s)")
    a.legend(loc="upper right", fontsize=8)
    for a in ax:
        a.grid(alpha=0.25)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    if "--json" in opts:
        summary = {"episode": str(ep), "outcome": man.get("outcome"), "min_clearance_body": round(float(np.nanmin(clearance)), 3),
                   "max_abs_along": round(float(np.max(np.abs(along[valid]))), 3), "max_abs_lateral": round(float(np.max(np.abs(lat[valid]))), 3),
                   "outage_windows": windows}
        pathlib.Path(opts["--json"]).write_text(json.dumps(summary, indent=1) + "\n", encoding="utf-8")
    print("wrote %s" % out)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
