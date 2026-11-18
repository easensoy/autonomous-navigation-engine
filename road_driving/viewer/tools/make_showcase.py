"""Record the London showcase: a Kensington chapter and a Knightsbridge chapter, each one uninterrupted
lock-step run rendered offscreen at 1920 x 1080 and encoded at 50 frames per second (two 10 ms ticks per
frame, H.264, yuv420p, CRF 18, fast start); joined with chapter markers; verified; with its metadata, a
poster frame and contact sheets.

    python tools/make_showcase.py OUT_DIR [--chapters K,N] [--crf 18] [--preset slow] [--quality high]

Run from road_driving/viewer with its environment (launch.ps1 -Setup). The backend is found as the viewer
finds it (build/road-msvc first). Nothing here changes what the simulation does: each chapter is the
episode exactly as `road_drive live` runs it, with no operator commands.
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
}


def run(cmd, **kw):
    print("+ " + " ".join(str(c) for c in cmd), flush=True)
    return subprocess.run([str(c) for c in cmd], check=True, **kw)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--chapters", default="K,N")
    ap.add_argument("--crf", type=int, default=18)
    ap.add_argument("--preset", default="slow")
    ap.add_argument("--quality", default="high")
    ap.add_argument("--skip-record", action="store_true", help="reuse chapter files already in OUT")
    a = ap.parse_args(argv)
    from ane_viewer.capture import find_ffmpeg
    ffmpeg = find_ffmpeg()
    out = pathlib.Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    keys = [k.strip() for k in a.chapters.split(",") if k.strip()]
    py = sys.executable
    for k in keys:
        ch = CHAPTERS[k]
        if a.skip_record and (out / ch["file"]).is_file():
            continue
        run([py, "-m", "ane_viewer", "--demo", k, "--hud", "present", "--director", ch["plan"], "--quality", a.quality, "--size", "1280x720",
             "--record-video", out / ch["file"], "--record-size", "1920x1080", "--record-fps", "50", "--crf", a.crf, "--preset", a.preset,
             "--frame-log", out / (ch["file"][:-4] + ".frames.jsonl"), "--title", ch["title"], "--subtitle", ch["subtitle"], "--card-note", ch["note"],
             "--card-seconds", "3", "--end-seconds", "3", "--exit-at-end", "--log", out / (ch["file"][:-4] + ".backend.log")], cwd=VIEWER)
    # Chapter markers, then the chapters joined without re-encoding (they share every encoder setting).
    meta = [";FFMETADATA1", "title=Autonomous Navigation Engine - London district scenes (independent research simulation)"]
    t = 0.0
    for k in keys:
        log = out / (CHAPTERS[k]["file"][:-4] + ".frames.jsonl")
        n = sum(1 for line in open(log, encoding="utf-8") if '"note"' not in line[:8])
        dur = n / 50.0
        meta += ["[CHAPTER]", "TIMEBASE=1/1000", "START=%d" % round(t * 1000), "END=%d" % round((t + dur) * 1000), "title=" + CHAPTERS[k]["title"]]
        t += dur
    (out / "chapters.txt").write_text("\n".join(meta) + "\n", encoding="utf-8")
    (out / "concat.txt").write_text("".join("file '%s'\n" % (out / CHAPTERS[k]["file"]).as_posix() for k in keys), encoding="utf-8")
    master = out / "ane-london-showcase-1080p50.mp4"
    run([ffmpeg, "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", out / "concat.txt", "-i", out / "chapters.txt", "-map", "0:v",
         "-map_metadata", "1", "-map_chapters", "1", "-c", "copy", "-movflags", "+faststart", master])
    # Poster (full resolution) and contact sheets: one tile every 2 s per chapter, and the frames either side of each cut.
    first = CHAPTERS[keys[0]]
    run([ffmpeg, "-y", "-loglevel", "error", "-ss", "38", "-i", out / CHAPTERS[keys[-1]]["file"], "-frames:v", "1", "-q:v", "2", out / "poster.jpg"])
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
                           "hud": "presentation overlay", "cameras": "director plans KEN-A, KNB-A (road_driving/viewer/ane_viewer/director.py)"},
             "created": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
             "provenance": "Source-informed research reconstructions of Kensington and Knightsbridge with approximated details, not surveyed "
                           "digital twins. Street geometry, names, one-way rules and turn restrictions from OpenStreetMap (ODbL, (c) OpenStreetMap "
                           "contributors); widths, kerbs and turning paths inferred; facades decorative. Independent research simulation: an "
                           "I-PACE-sized proxy, not a Jaguar; not Waymo software or data. See road_driving/viewer/districts/PROVENANCE.md."}
    (out / "extra.json").write_text(json.dumps(extra, indent=2) + "\n", encoding="utf-8")
    from check_recording import main as check
    logs = []
    for k in keys:
        logs += ["--log", str(out / (CHAPTERS[k]["file"][:-4] + ".frames.jsonl"))]
    return check([str(master)] + logs + ["--out", str(out / "ane-london-showcase-1080p50.json"), "--extra", str(out / "extra.json")])


if __name__ == "__main__":
    sys.exit(main())
