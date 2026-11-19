"""Record a showcase: the London scenes (a Kensington chapter and a Knightsbridge chapter) or the San Francisco
scene (one SF-A chapter). Each chapter is one uninterrupted lock-step run rendered offscreen at 1920 x 1080 and
encoded at 50 frames per second (two 10 ms ticks per frame, H.264, yuv420p, CRF 18, fast start); the chapters
are joined with chapter markers and verified, with the metadata, a poster frame and contact sheets.

    python tools/make_showcase.py OUT_DIR [--showcase london|sf] [--chapters K,N] [--crf 18] [--preset slow] [--quality high]

Run from road_driving/viewer with its environment (launch.ps1 -Setup). The backend is found as the viewer
finds it (build/road-msvc first). Nothing here changes what the simulation does: each chapter is the
episode exactly as `road_drive live` runs it, with no operator commands.

The San Francisco chapter drives the evaluation seed that road_driving/benchmark/PROTOCOL_SF.md's rule picks
from the stored results: the SF-A gnss_outage_sf episode on ekf_kerb2 whose position RMSE is the median of
the 15 (ties: the lower seed). The rule is applied here, not chosen by watching runs.
"""
import argparse
import datetime
import json
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
VIEWER = HERE.parent
REPO = VIEWER.parents[1]
sys.path.insert(0, str(VIEWER))

CHAPTERS = {
    "K": {"plan": "KEN-A", "file": "chapter1_kensington.mp4", "title": "Chapter 1 - Kensington",
          "subtitle": "Kensington Gore, Queen's Gate and Elvaston Place",
          "note": "Source-informed reconstruction from OpenStreetMap (ODbL); details approximated.  KEN-A, gnss_outage_ldn, seed 1, "
                  "ekf_kerb2, unique re-join: GNSS drops out on Queen's Gate."},
    "N": {"plan": "KNB-A", "file": "chapter2_knightsbridge.mp4", "title": "Chapter 2 - Knightsbridge",
          "subtitle": "Brompton Road, Hans Road and Basil Street",
          "note": "Source-informed reconstruction from OpenStreetMap (ODbL); details approximated.  KNB-A, multipath_ldn, seed 1, "
                  "ekf_kerb2, unique re-join: GNSS fixes 6 m off between the tall frontages of Brompton Road."},
    "SF": {"plan": "SF-A", "file": "sf_soma.mp4", "title": "San Francisco - SoMa",
           "subtitle": "Townsend Street, 5th Street and Brannan Street",
           "note": "Source-informed reconstruction from OpenStreetMap (ODbL); details approximated.  SF-A, gnss_outage_sf, seed {seed} "
                   "(the evaluation's median run), ekf_kerb2, unique re-join, Cybercab-inspired proxy: GNSS drops out on Townsend Street."},
}

SHOWCASES = {
    "london": {"chapters": "K,N", "master": "ane-london-showcase-1080p50", "poster_at": 38.0,
               "title": "Autonomous Navigation Engine - London district scenes (independent research simulation)",
               "provenance": "Source-informed research reconstructions of Kensington and Knightsbridge with approximated details, not "
                             "surveyed digital twins. Street geometry, names, one-way rules and turn restrictions from OpenStreetMap (ODbL, "
                             "(c) OpenStreetMap contributors); widths, kerbs and turning paths inferred; facades decorative. Independent "
                             "research simulation: an I-PACE-sized proxy, not a Jaguar; not Waymo software or data. See "
                             "road_driving/viewer/districts/PROVENANCE.md."},
    "sf": {"chapters": "SF", "master": "ane-sf-showcase-1080p50", "poster_at": 20.0,
           "title": "Autonomous Navigation Engine - San Francisco SoMa scene (independent research simulation)",
           "provenance": "Source-informed research reconstruction of South of Market, San Francisco, by the Caltrain station at 4th and "
                         "Townsend, with approximated details, not a surveyed digital twin. Street geometry, names, one-way rules and turn "
                         "restrictions from OpenStreetMap (ODbL, (c) OpenStreetMap contributors); widths, kerbs and turning paths "
                         "inferred; facades decorative; signals and stop control not modelled. Independent simulation: a "
                         "Cybercab-inspired proxy (cybercab_proxy_v1) whose outer dimensions and sensor mounts are research "
                         "assumptions, with the I-PACE proxy's simulated actuator limits; not a Tesla vehicle; no Tesla or Waymo "
                         "software or data. See "
                         "road_driving/viewer/districts/PROVENANCE.md and road_driving/benchmark/PROTOCOL_SF.md."},
}
SF_RESULTS = REPO / "road_driving" / "benchmark" / "sf" / "sf" / "closed_loop.jsonl"


def sf_seed(results=SF_RESULTS):
    """PROTOCOL_SF.md's rule: the evaluation seed whose closed-loop ekf_kerb2 episode on SF-A under gnss_outage_sf
    has the median position RMSE of the 15; of seeds tied at the median, the lowest. Returns (seed, RMSE)."""
    rows = [json.loads(line) for line in open(results, encoding="utf-8") if line.strip()]
    runs = sorted((r["est_rmse"], r["seed"]) for r in rows if r["scenario"] == "SF-A" and r["condition"] == "gnss_outage_sf"
                  and r["estimator"] == "ekf_kerb2" and r["mode"] == "estimated" and r["vehicle"] == "cybercab_proxy_v1")
    if len(runs) != 15:
        sys.exit("%s: expected 15 SF-A gnss_outage_sf ekf_kerb2 episodes, found %d" % (results, len(runs)))
    median = runs[7][0]
    return min(s for v, s in runs if v == median), median


def run(cmd, **kw):
    print("+ " + " ".join(str(c) for c in cmd), flush=True)
    return subprocess.run([str(c) for c in cmd], check=True, **kw)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--showcase", default="london", choices=sorted(SHOWCASES))
    ap.add_argument("--chapters", help="chapter keys, in order (default: the showcase's)")
    ap.add_argument("--poster-at", type=float, help="the poster frame's time in the last chapter, in seconds (default: the showcase's)")
    ap.add_argument("--crf", type=int, default=18)
    ap.add_argument("--preset", default="slow")
    ap.add_argument("--quality", default="high")
    ap.add_argument("--skip-record", action="store_true", help="reuse chapter files already in OUT")
    a = ap.parse_args(argv)
    from ane_viewer.capture import find_ffmpeg
    ffmpeg = find_ffmpeg()
    out = pathlib.Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    show = SHOWCASES[a.showcase]
    keys = [k.strip() for k in (a.chapters or show["chapters"]).split(",") if k.strip()]
    selection = None
    if "SF" in keys:
        seed, rmse = sf_seed()
        selection = {"rule": "PROTOCOL_SF.md: the evaluation seed whose closed-loop ekf_kerb2 episode on SF-A under gnss_outage_sf has the "
                             "median position RMSE of the 15 (ties: the lower seed)",
                     "results": SF_RESULTS.relative_to(REPO).as_posix(), "seed": seed, "est_rmse": rmse}
        print("SF-A gnss_outage_sf: seed %d, the median closed-loop ekf_kerb2 RMSE (%.4f m)" % (seed, rmse), flush=True)
    py = sys.executable
    for k in keys:
        ch = CHAPTERS[k]
        if a.skip_record and (out / ch["file"]).is_file():
            continue
        seed_args = ["--seed", selection["seed"]] if k == "SF" else []
        run([py, "-m", "ane_viewer", "--demo", k] + seed_args + ["--hud", "present", "--director", ch["plan"], "--quality", a.quality,
             "--size", "1280x720", "--record-video", out / ch["file"], "--record-size", "1920x1080", "--record-fps", "50", "--crf", a.crf,
             "--preset", a.preset, "--frame-log", out / (ch["file"][:-4] + ".frames.jsonl"), "--title", ch["title"], "--subtitle", ch["subtitle"],
             "--card-note", ch["note"].format(seed=selection["seed"] if selection else 0), "--card-seconds", "3", "--end-seconds", "3",
             "--exit-at-end", "--log", out / (ch["file"][:-4] + ".backend.log")], cwd=VIEWER)
    # Chapter markers, then the chapters joined without re-encoding (they share every encoder setting).
    meta = [";FFMETADATA1", "title=" + show["title"]]
    t = 0.0
    for k in keys:
        log = out / (CHAPTERS[k]["file"][:-4] + ".frames.jsonl")
        n = sum(1 for line in open(log, encoding="utf-8") if '"note"' not in line[:8])
        dur = n / 50.0
        meta += ["[CHAPTER]", "TIMEBASE=1/1000", "START=%d" % round(t * 1000), "END=%d" % round((t + dur) * 1000), "title=" + CHAPTERS[k]["title"]]
        t += dur
    (out / "chapters.txt").write_text("\n".join(meta) + "\n", encoding="utf-8")
    (out / "concat.txt").write_text("".join("file '%s'\n" % (out / CHAPTERS[k]["file"]).as_posix() for k in keys), encoding="utf-8")
    master = out / (show["master"] + ".mp4")
    run([ffmpeg, "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", out / "concat.txt", "-i", out / "chapters.txt", "-map", "0:v",
         "-map_metadata", "1", "-map_chapters", "1", "-c", "copy", "-movflags", "+faststart", master])
    # Poster (full resolution) and contact sheets: one tile every 2 s per chapter, and the frames either side of each cut.
    poster_at = show["poster_at"] if a.poster_at is None else a.poster_at
    run([ffmpeg, "-y", "-loglevel", "error", "-ss", "%g" % poster_at, "-i", out / CHAPTERS[keys[-1]]["file"], "-frames:v", "1", "-q:v", "2",
         out / "poster.jpg"])
    for k in keys:
        f = out / CHAPTERS[k]["file"]
        run([ffmpeg, "-y", "-loglevel", "error", "-i", f, "-vf", "select='not(mod(n\\,100))',scale=480:-1,tile=6x9", "-frames:v", "1", "-vsync", "vfr",
             out / (f.stem + "_sheet.png")])
        rows = [json.loads(line) for line in open(out / (f.stem + ".frames.jsonl"), encoding="utf-8") if '"note"' not in line[:8]]
        cuts = [r["frame"] for a_, r in zip(rows, rows[1:]) if a_["shot"] != r["shot"]]
        sel = "+".join("eq(n\\,%d)+eq(n\\,%d)" % (c - 1, c) for c in cuts) or "eq(n\\,0)"
        run([ffmpeg, "-y", "-loglevel", "error", "-i", f, "-vf", "select='%s',scale=480:-1,tile=4x%d" % (sel, max(1, (2 * len(cuts) + 3) // 4)),
             "-frames:v", "1", "-vsync", "vfr", out / (f.stem + "_cuts.png")])
    head = subprocess.run(["git", "-C", str(REPO), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    dirty = subprocess.run(["git", "-C", str(REPO), "status", "--porcelain", "--", "road_driving"], capture_output=True, text=True).stdout.strip()
    extra = {"source": {"repository": "easensoy/autonomous-navigation-engine", "commit": head, "road_driving_modified": bool(dirty)},
             "recording": {"render": "offscreen 1920x1080, rendered at that size (no scaling), multisampled", "fps": 50, "ticks_per_frame": 2,
                           "lockstep": "the backend is paused and stepped exactly 2 ticks of 10 ms per captured frame",
                           "encoder": "libx264 %s CRF %d, High profile, yuv420p, 1 s GOP, fast start" % (a.preset, a.crf), "quality": a.quality,
                           "hud": "presentation overlay", "poster_at_s": poster_at,
                           "cameras": "director plans %s (road_driving/viewer/ane_viewer/director.py)" % ", ".join(CHAPTERS[k]["plan"] for k in keys)},
             "created": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
             "provenance": show["provenance"]}
    if selection:
        extra["selection"] = selection
    (out / "extra.json").write_text(json.dumps(extra, indent=2) + "\n", encoding="utf-8")
    from check_recording import main as check
    logs = []
    for k in keys:
        logs += ["--log", str(out / (CHAPTERS[k]["file"][:-4] + ".frames.jsonl"))]
    return check([str(master)] + logs + ["--out", str(out / (show["master"] + ".json")), "--extra", str(out / "extra.json")])


if __name__ == "__main__":
    sys.exit(main())
