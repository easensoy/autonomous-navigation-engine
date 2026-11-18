"""Verify a recording and write its metadata: the encoded stream (ffmpeg standing in for ffprobe), fast start,
the chapters, and every chapter's frame log.

    python check_recording.py VIDEO --log CHAPTER.frames.jsonl [--log ...] [--fps 50] [--size 1920x1080] [--out META.json]

Frame-log checks, per chapter: the run frames advance exactly ticks-per-frame simulation ticks each, from the
first tick to the episode's end without a gap; title and end cards hold one tick; the camera is never inside a
building or canopy; the car stays in view (at least three of its four sight points, except from overhead and
the driver's seat); and the clock the HUD drew is the frame's own tick. Exit status 1 when a check fails.
"""
import argparse
import hashlib
import json
import pathlib
import re
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))


def ffmpeg_exe():
    from ane_viewer.capture import find_ffmpeg
    exe = find_ffmpeg()
    if not exe:
        sys.exit("ffmpeg not found (set ANE_FFMPEG)")
    return exe


def probe(ffmpeg, path):
    """What ffprobe would report, from ffmpeg's own stream description and a stream-copy pass (frame count)."""
    err = subprocess.run([ffmpeg, "-hide_banner", "-i", str(path)], capture_output=True, text=True).stderr
    v = re.search(r"Stream #0:0[^\n]*Video: (\w+) \((\w+)\)[^\n]*?, (\w+)\(([^)]*)\), (\d+)x(\d+)[^\n]*?, (\d+) kb/s, ([\d.]+) fps", err)
    d = re.search(r"Duration: (\d+):(\d+):([\d.]+)", err)
    chapters = [{"start": float(a), "end": float(b), "title": t.strip()} for a, b, t in
                re.findall(r"Chapter #0:\d+: start ([\d.]+), end ([\d.]+)\s+Metadata:\s+title\s+:\s*([^\n]+)", err)]
    count = subprocess.run([ffmpeg, "-hide_banner", "-i", str(path), "-map", "0:v:0", "-c", "copy", "-f", "null", "-"], capture_output=True, text=True).stderr
    frames = [int(n) for n in re.findall(r"frame=\s*(\d+)", count)]
    out = {"codec": v.group(1) if v else None, "profile": v.group(2) if v else None, "pix_fmt": v.group(3) if v else None,
           "colour": v.group(4) if v else None, "width": int(v.group(5)) if v else None, "height": int(v.group(6)) if v else None,
           "bitrate_kbps": int(v.group(7)) if v else None, "fps": float(v.group(8)) if v else None, "frames": frames[-1] if frames else None,
           "duration_s": round(int(d.group(1)) * 3600 + int(d.group(2)) * 60 + float(d.group(3)), 3) if d else None, "chapters": chapters}
    return out


def top_level_boxes(path):
    """The MP4's top-level box types in order (fast start: moov before mdat)."""
    boxes = []
    with open(path, "rb") as f:
        while True:
            head = f.read(8)
            if len(head) < 8:
                break
            size = int.from_bytes(head[:4], "big")
            kind = head[4:8].decode("latin-1")
            boxes.append(kind)
            if size == 1:
                size = int.from_bytes(f.read(8), "big")
                f.seek(size - 16, 1)
            elif size == 0:
                break
            else:
                f.seek(size - 8, 1)
    return boxes


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def check_log(path, ticks_per_frame):
    rows, notes = [], []
    for line in open(path, encoding="utf-8"):
        r = json.loads(line)
        (notes if "note" in r else rows).append(r)
    problems = []
    run = [r for r in rows if r["phase"] == "run"]
    steps = [b["tick"] - a["tick"] for a, b in zip(run, run[1:])]
    # Every frame advances exactly k ticks; only the last may advance fewer, if the episode ended within it.
    if set(steps[:-1]) - {ticks_per_frame} or (steps and not 1 <= steps[-1] <= ticks_per_frame):
        problems.append("run frames advance by %s ticks, not exactly %d" % (sorted(set(steps)), ticks_per_frame))
    for phase in ("title", "end"):
        held = {r["tick"] for r in rows if r["phase"] == phase}
        if len(held) > 1:
            problems.append("the %s card spans ticks %s" % (phase, sorted(held)))
    inside = [r["frame"] for r in rows if r["inside"]]
    if inside:
        problems.append("camera inside the scenery in %d frames (first %d)" % (len(inside), inside[0]))
    hidden = [r["frame"] for r in rows if r["camera"] not in ("overhead", "driver") and r["share"] < 0.75]
    if hidden:
        problems.append("car out of view in %d frames (first %d)" % (len(hidden), hidden[0]))
    clock = [r["frame"] for r in rows if r["t"] is not None and r["clock"] != "t = %.2f s" % r["t"]]
    if clock:
        problems.append("HUD clock differs from the frame's tick in %d frames (first %d)" % (len(clock), clock[0]))
    if any(not r["ok"] for r in rows):
        problems.append("%d frames were not encoded" % sum(1 for r in rows if not r["ok"]))
    end = next((n["note"] for n in notes if n["note"].get("type") == "end"), None)
    cfg = next((n["note"]["config"] for n in notes if n["note"].get("type") == "config"), {})
    if end is None:
        problems.append("the episode's end is not in the log")
    elif run and run[-1]["tick"] != end["tick"]:
        problems.append("the last run frame is tick %d, the episode ended at tick %d" % (run[-1]["tick"], end["tick"]))
    shots = []
    for a, b in zip([None] + rows, rows):
        if a is None or a["shot"] != b["shot"]:
            shots.append({"frame": b["frame"], "t": b["t"], "shot": b["shot"]})
    summary = {"frames": len(rows), "title_frames": sum(1 for r in rows if r["phase"] == "title"), "run_frames": len(run),
               "tick_steps": sorted(set(steps)), "final_step": steps[-1] if steps else None,
               "end_frames": sum(1 for r in rows if r["phase"] == "end"), "first_tick": run[0]["tick"] if run else None,
               "last_tick": run[-1]["tick"] if run else None, "ticks_per_frame": ticks_per_frame, "frames_inside_geometry": len(inside),
               "min_car_share": min((r["share"] for r in rows if r["camera"] not in ("overhead", "driver")), default=1.0),
               "hud_clock_matches_tick": not clock, "shots": shots, "config": cfg, "end": end}
    return summary, problems


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--log", action="append", default=[], help="a chapter's frame log, in chapter order")
    ap.add_argument("--fps", type=int, default=50)
    ap.add_argument("--size", default="1920x1080")
    ap.add_argument("--out", help="write the metadata JSON here")
    ap.add_argument("--extra", help="a JSON file of fields to add to the metadata (source revision, settings)")
    a = ap.parse_args(argv)
    ffmpeg = ffmpeg_exe()
    path = pathlib.Path(a.video)
    info = probe(ffmpeg, path)
    boxes = top_level_boxes(path)
    w, h = (int(v) for v in a.size.lower().split("x"))
    problems = []
    if (info["width"], info["height"]) != (w, h):
        problems.append("resolution %sx%s, expected %dx%d" % (info["width"], info["height"], w, h))
    if info["fps"] != a.fps:
        problems.append("frame rate %s, expected %d" % (info["fps"], a.fps))
    if info["codec"] != "h264" or info["pix_fmt"] != "yuv420p":
        problems.append("codec %s %s, expected h264 yuv420p" % (info["codec"], info["pix_fmt"]))
    faststart = "moov" in boxes and "mdat" in boxes and boxes.index("moov") < boxes.index("mdat")
    if not faststart:
        problems.append("not fast start: top-level boxes %s" % boxes)
    k = 100 // a.fps
    chapters = []
    total = 0
    for i, log in enumerate(a.log):
        summary, probs = check_log(log, k)
        problems += ["chapter %d: %s" % (i + 1, p) for p in probs]
        total += summary["frames"]
        if i < len(info["chapters"]):
            summary["title"] = info["chapters"][i]["title"]
            summary["start_s"], summary["end_s"] = info["chapters"][i]["start"], info["chapters"][i]["end"]
        chapters.append(summary)
    if a.log and info["frames"] != total:
        problems.append("the video has %s frames; the chapter logs %d" % (info["frames"], total))
    if a.log and info["frames"] and abs(info["frames"] / float(a.fps) - info["duration_s"]) > 0.05:
        problems.append("duration %.3f s for %d frames at %d fps" % (info["duration_s"], info["frames"], a.fps))
    meta = {"file": path.name, "bytes": path.stat().st_size, "sha256": sha256(path), "video": dict(info, faststart=faststart, top_level_boxes=boxes),
            "chapters": chapters, "checks": {"passed": not problems, "problems": problems}}
    if a.extra:
        meta.update(json.loads(pathlib.Path(a.extra).read_text(encoding="utf-8")))
    if a.out:
        pathlib.Path(a.out).write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({k2: meta[k2] for k2 in ("file", "bytes", "sha256")}, indent=None))
    print(json.dumps(meta["video"], indent=None))
    for c in chapters:
        print("chapter: %s frames (%s title, %s run, %s end), ticks %s-%s, inside %s, min share %.2f, clock ok %s, %s" % (
            c["frames"], c["title_frames"], c["run_frames"], c["end_frames"], c["first_tick"], c["last_tick"], c["frames_inside_geometry"],
            c["min_car_share"], c["hud_clock_matches_tick"], (c["end"] or {}).get("outcome")))
    print("checks: %s" % ("passed" if not problems else "FAILED: " + "; ".join(problems)))
    return 0 if not problems else 1


if __name__ == "__main__":
    sys.exit(main())
