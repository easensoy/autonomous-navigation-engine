"""Two figures for the San Francisco evidence, from the campaign's records (benchmark/PROTOCOL_SF.md).

    python road_driving/tools/sf_plots.py EVIDENCE_DIR

EVIDENCE_DIR is the folder sf_analysis.py reads (sf/ and sf-ipace/). Writes EVIDENCE_DIR/plots/:
- outage_end_error.png: P1 and P7. Every estimator's position error when the fixes return after gnss_outage_sf,
  one dot per run (SF-A and SF-B, seeds 1 to 15), for each vehicle, on a log scale, with each set's median.
- drift_mechanism.png: P5. For ekf_kerb2 under gnss_drift_sf, each run's along-road and lateral error at the
  drift window's end (open loop), the predeclared region (along over 1.0 m, lateral under 0.3 m), and the
  closed-loop episode of the same route and seed: completed, or a fallback stop for lateral error.
Colour follows the vehicle; the closed-loop outcome is a marker shape. Needs matplotlib.
"""
import json
import pathlib
import statistics
import sys

ESTIMATORS = ["dr", "gnss_hold", "gnss_cv", "ekf", "ekf_int", "ekf_int2", "ekf_kerb", "ekf_kerb2"]
VEHICLES = [("sf", "Cybercab-inspired proxy", "#2a78d6"), ("sf-ipace", "I-PACE proxy", "#eb6834")]
SURFACE, INK, INK2, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#e1e0d9", "#c3c2b7"


def load(path):
    p = pathlib.Path(path)
    return [json.loads(line) for line in p.read_text(encoding="utf-8").splitlines() if line.strip()] if p.is_file() else []


def style(ax):
    ax.set_facecolor(SURFACE)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
    ax.tick_params(colors=INK2, labelsize=9)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def outage_end_error(ev, out, plt):
    fig, ax = plt.subplots(figsize=(9.0, 5.2), dpi=120)
    fig.patch.set_facecolor(SURFACE)
    style(ax)
    ax.grid(False, axis="y")
    rows = list(reversed(ESTIMATORS))  # ekf_kerb2 at the bottom, nearest the axis
    for k, (folder, label, colour) in enumerate(VEHICLES):
        runs = [r for r in load(ev / folder / "runs.jsonl") if r["condition"] == "gnss_outage_sf"]
        offset = 0.17 if k == 0 else -0.17
        for i, est in enumerate(rows):
            vals = [r["fault_end_error"] for r in runs if r["estimator"] == est and r["fault_end_error"] >= 0.0]
            if not vals:
                continue
            ax.scatter([max(v, 1e-3) for v in vals], [i + offset] * len(vals), s=26, color=colour, alpha=0.85, linewidths=0.6,
                       edgecolors=SURFACE, label=label if i == 0 else None, zorder=3)
            m = statistics.median(vals)
            ax.plot([m, m], [i + offset - 0.13, i + offset + 0.13], color=INK, linewidth=1.6, zorder=4)
    ax.axvline(0.5, color=INK2, linewidth=1.0, linestyle=(0, (4, 3)), zorder=2)
    ax.text(0.53, len(rows) - 0.45, "P1, P7: ekf_kerb2's median at most 0.50 m", color=INK2, fontsize=8.5, va="center")
    ax.set_xscale("log")
    ax.set_xlim(0.004, 400)
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels(rows, fontsize=9.5, color=INK)
    ax.set_ylim(-0.6, len(rows) - 0.2)
    ax.set_xlabel("position error when the fixes return (m, log scale); one dot per run, a bar at each median", color=INK2, fontsize=9)
    ax.set_title("SF, gnss_outage_sf: the error when GNSS returns, 30 runs per estimator and vehicle (SF-A, SF-B; seeds 1-15)",
                 color=INK, fontsize=10.5, loc="left")
    leg = ax.legend(loc="lower right", fontsize=8.5, frameon=True, framealpha=0.95, edgecolor=GRID)
    for t in leg.get_texts():
        t.set_color(INK)
    fig.text(0.01, 0.01, "Independent simulation; simulated vehicle proxies, not Tesla or Jaguar vehicles.", color=INK2, fontsize=7.5)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(out, facecolor=SURFACE)
    plt.close(fig)


def drift_mechanism(ev, out, plt):
    folder, label, colour = VEHICLES[0]
    runs = [r for r in load(ev / folder / "runs.jsonl") if r["condition"] == "gnss_drift_sf" and r["estimator"] == "ekf_kerb2"]
    episodes = {(e["scenario"], e["seed"]): e for e in load(ev / folder / "closed_loop.jsonl")
                if e["condition"] == "gnss_drift_sf" and e["estimator"] == "ekf_kerb2"}
    fig, ax = plt.subplots(figsize=(8.0, 5.2), dpi=120)
    fig.patch.set_facecolor(SURFACE)
    style(ax)
    xmax = max(4.5, max(abs(r["fault_end_along"]) for r in runs) + 0.3)
    ymax = max(0.4, max(abs(r["fault_end_lateral"]) for r in runs) + 0.05)
    ax.fill_between([1.0, xmax], 0.0, 0.3, color=GRID, alpha=0.55, linewidth=0, zorder=1)
    ax.text(xmax - 0.05, 0.305, "P5's region: along over 1.0 m, lateral under 0.3 m", color=INK2, fontsize=8.5, ha="right", va="bottom")
    shapes = {"completed": ("o", "closed loop: completed"), "lateral": ("X", "closed loop: fallback stop for lateral error"),
              "other": ("s", "closed loop: other stop")}
    seen = set()
    counts = {True: {"completed": 0, "lateral": 0, "other": 0}, False: {"completed": 0, "lateral": 0, "other": 0}}
    for r in runs:
        e = episodes.get((r["route"], r["seed"]))
        kind = "completed" if e and e["outcome"] == "COMPLETED" else ("lateral" if e and "lateral error" in e["fallback_reason"] else "other")
        inside = abs(r["fault_end_along"]) > 1.0 and abs(r["fault_end_lateral"]) < 0.3
        counts[inside][kind] += 1
        marker, text = shapes[kind]
        ax.scatter([abs(r["fault_end_along"])], [abs(r["fault_end_lateral"])], s=70 if marker == "X" else 46, marker=marker, color=colour,
                   alpha=0.9, linewidths=0.8, edgecolors=SURFACE, zorder=3, label=None if kind in seen else text)
        seen.add(kind)

    def tally(c):
        parts = ["%d completed" % c["completed"], "%d fallback stop%s for lateral error" % (c["lateral"], "" if c["lateral"] == 1 else "s")]
        if c["other"]:
            parts.append("%d other stop%s" % (c["other"], "" if c["other"] == 1 else "s"))
        return ", ".join(parts)
    n_in, n_out = sum(counts[True].values()), sum(counts[False].values())
    ax.text(xmax - 0.05, 0.2, "%d runs inside: %s" % (n_in, tally(counts[True])), color=INK, fontsize=8.5, ha="right", va="center")
    ax.text(0.05, 0.2, "%d runs outside:\n%s" % (n_out, tally(counts[False]).replace(", ", ",\n").replace(" for lateral", "\nfor lateral")),
            color=INK, fontsize=8.5, ha="left", va="center")
    ax.set_xlim(0.0, xmax)
    ax.set_ylim(0.0, ymax)
    ax.set_xlabel("|along-road error| at the drift window's end (m), open loop", color=INK2, fontsize=9)
    ax.set_ylabel("|lateral error| there (m)", color=INK2, fontsize=9)
    ax.set_title("SF, gnss_drift_sf, ekf_kerb2 (%s): 30 runs, SF-A and SF-B, seeds 1-15" % label, color=INK, fontsize=10.5, loc="left")
    leg = ax.legend(loc="upper left", fontsize=8.5, frameon=True, framealpha=0.95, edgecolor=GRID)
    for t in leg.get_texts():
        t.set_color(INK)
    fig.text(0.01, 0.01, "Each dot: a run's open-loop errors; its shape: the closed-loop episode of the same route and seed. "
             "Independent simulation.", color=INK2, fontsize=7.5)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(out, facecolor=SURFACE)
    plt.close(fig)


def main(argv):
    if len(argv) != 2:
        sys.exit(__doc__)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ev = pathlib.Path(argv[1])
    plots = ev / "plots"
    plots.mkdir(exist_ok=True)
    outage_end_error(ev, plots / "outage_end_error.png", plt)
    drift_mechanism(ev, plots / "drift_mechanism.png", plt)
    print("wrote %s and %s" % (plots / "outage_end_error.png", plots / "drift_mechanism.png"))


if __name__ == "__main__":
    main(sys.argv)
