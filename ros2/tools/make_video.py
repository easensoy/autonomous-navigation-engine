#!/usr/bin/env python3
"""Builds the README animations, poster and full recordings from a captured trial.

  python3 ros2/tools/make_video.py --capture-dir <dir> --trial-dir <dir> --out-dir <dir> --story mission|failure

Inputs: the screen frames recorded by ros2/tools/container_capture.sh (Gazebo GUI and RViz2 side by side)
with frames.csv (each frame's monotonic time), capture_clock.csv (container monotonic time -> simulation
time), windows.txt (window geometry), and the
trial's trajectory.json (ground truth, events and the navigator's recorded status at 10 Hz), metrics.json
and manifest.json. Every output frame shows

  - the Gazebo scene and the RViz2 view, cropped from the same screen frame (nothing inside them is
    drawn by this script);
  - a telemetry panel whose values are the navigator's recorded status (ane/status) at that simulation
    time, the last recorded blockage report and the last recorded engine call;
  - a caption for the most recent recorded event, a timeline of the recorded events, and labels saying
    what is simulated, scripted and ground truth, and how fast the clip plays.

Output frames are taken at even steps of simulation time, so "1x" means simulation time. The animated
preview plays the quiet parts faster and the decisive seconds at 1x; every frame states its speed.

Outputs: <name>.mp4 (whole run, 1x simulation time), <name>-preview.gif (README width) and, for the
mission story, <name>-poster.png. Needs Pillow, numpy and ffmpeg.
"""

import argparse
import bisect
import json
import math
import re
import subprocess
import tempfile
from pathlib import Path

import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ane_sim" / "scripts"))
from scenario_lib import load_roadmap  # noqa: E402

ROADMAP = load_roadmap(Path(__file__).resolve().parents[1] / "ane_sim" / "maps" / "warehouse_roadmap.txt")
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
MONO = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
MONO_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"

W = 1280                       # canvas width; the Gazebo scene is shown at its captured size
HEADER, SCENE_H, ROW_H, FOOTER = 64, 720, 400, 96
H = HEADER + SCENE_H + ROW_H + FOOTER
RVIZ_SRC = (784, 490)          # centred crop of the RViz2 view (44 px/m; the hall fills it, the dock handles are cut)
BG, PANEL, INK, MUTED, RULE = (17, 21, 28), (27, 33, 43), (236, 240, 245), (150, 160, 175), (52, 60, 74)
ORANGE, GREEN, AMBER, RED, BLUE, PURPLE = (255, 160, 70), (80, 200, 120), (245, 190, 60), (240, 85, 75), (95, 160, 255), (200, 120, 230)
STATE_COLOUR = {"EXECUTING": GREEN, "HOLDING_FEEDBACK": AMBER, "COMPLETED": BLUE, "BLOCKED": RED,
                "FAILED_CONTROLLER": RED, "FAILED_LOCALIZATION": RED, "STOPPED": AMBER, "IDLE": MUTED}


def font(path, size):
    try:
        return ImageFont.truetype(path, size)
    except OSError:
        return ImageFont.load_default()


F = {"badge": font(FONT_BOLD, 22), "head": font(FONT, 19), "small": font(FONT, 16), "label": font(FONT, 17),
     "chip": font(FONT_BOLD, 16), "caption": font(FONT, 23), "time": font(MONO_BOLD, 19),
     "k": font(FONT, 18), "v": font(MONO, 20), "vb": font(MONO_BOLD, 20), "tick": font(FONT, 14)}


def wrap(draw, text, face, width):
    lines, line = [], ""
    for word in text.split():
        trial = f"{line} {word}".strip()
        if draw.textlength(trial, font=face) <= width:
            line = trial
        else:
            lines.append(line)
            line = word
    return lines + [line] if line else lines


def chip(draw, xy, text, fill=(0, 0, 0, 170), ink=INK, face=None, pad=(10, 5)):
    face = face or F["chip"]
    x, y = xy
    w = draw.textlength(text, font=face)
    draw.rounded_rectangle((x, y, x + w + 2 * pad[0], y + 26), radius=6, fill=fill)
    draw.text((x + pad[0], y + 3), text, font=face, fill=ink)
    return x + w + 2 * pad[0]


def window_geometry(path):
    """{'gazebo': (x, y, w, h), 'rviz': (x, y, w, h)} from xdotool --shell output."""
    out = {}
    for line in Path(path).read_text().splitlines():
        name, rest = line.split(" ", 1)
        values = dict(re.findall(r"(\w+)=(\d+)", rest))
        out[name] = tuple(int(values[k]) for k in ("X", "Y", "WIDTH", "HEIGHT"))
    return out


def rviz_render_box(frame, window):
    """The RViz2 render area inside its window: the longest run of rows in which some pixels have the
    view's background colour (252, 252, 252). Menu, tool and status bars never do; every row of the view
    does, at least in the margins around the hall."""
    x, y, w, h = window
    pixels = np.asarray(frame.crop((x, y, x + w, y + h)).convert("RGB")).astype(int)
    share = (np.abs(pixels - 252).max(axis=2) <= 3).mean(axis=1)
    best, start = (0, 0), None
    for i, ok in enumerate(list(share >= 0.03) + [False]):
        if ok and start is None:
            start = i
        elif not ok and start is not None:
            best = max(best, (i - start, start))
            start = None
    rows, top = best
    return x, y + top, w, rows


class Story:
    """Captions and timeline ticks from the recorded events, for the mission (B) or failure (C) story."""

    def __init__(self, kind, trajectory, metrics):
        m = metrics["metrics"]
        self.items, self.ticks = [], []
        req = trajectory["mission_request"]
        routes = trajectory["routes"]
        sensor = [b for b in trajectory["blockages"] if b["source"] == "SENSOR"]
        operator = [b for b in trajectory["blockages"] if b["source"] != "SENSOR"]
        obstacles = trajectory["obstacles"]
        replaced = [r for r in routes if r["type"] == "ROUTE_REPLACED"]
        blocked = [r for r in routes if r["type"] == "ROUTE_BLOCKED"]
        terminal = trajectory.get("terminal")
        calls = {e["type"]: e["value"] for e in trajectory["event_log"] if e["type"].startswith("ROUTE_")}
        if kind == "failure" and operator:
            self.add(operator[0]["t"], "Operator closures (scripted notifications, no physical obstacle): the north "
                     "and south aisles are closed; purple nodes in RViz2.", "closures")
        self.add(req, "Mission dispatch → delivery: the engine (NavigationCore, A*) plans the least-cost route, "
                 + ("through the middle aisle." if kind == "mission" else "through the middle aisle, the only one open."),
                 "plan")
        for o in obstacles[:1]:
            self.add(o["t"], "Scripted world event: the scenario places a pallet in the middle aisle. The navigator "
                     "is not told.", "pallet")
        if sensor:
            self.add(sensor[0]["t"], f"Lidar: returns the static map does not explain block {len(sensor[0]['nodes'])} "
                     f"roadmap nodes (red), {m['detection_latency_s']:.2f} s after the pallet appeared.", "lidar")
        if kind == "mission" and replaced:
            t = replaced[0]["t"]
            self.add(t, f"handleObstacles: the engine replaces the route ahead in {calls.get('ROUTE_REPLACED', 0):.2f} ms: "
                     "back to the west cross aisle and along the south aisle (the north aisle's picking zone costs more).",
                     "replan")
            self.add(t + 4.0, "The robot returns to the last node it reached, turns on it, and follows the engine's "
                     "new route.", None)
            south = [e["t"] for e in trajectory["event_log"] if e["type"] == "NODE_REACHED" and e["t"] > t
                     and abs(ROADMAP["nodes"][e["nodes"][0]][1] + 3.0) < 0.01]
            if south:
                self.add(south[0], "Detour on the engine's replacement route through the south aisle; the north "
                         "aisle is as long but crosses the picking zone (edge cost ×1.5).", "south aisle")
            if terminal is not None:
                self.add(terminal, f"COMPLETED: {m['final_position_error_m']:.3f} m from the goal (ground truth), "
                         f"no collision, closest approach to an obstacle {m['min_clearance_m']:.2f} m.", "arrive")
        if kind == "failure" and blocked:
            t = blocked[0]["t"]
            self.add(t, f"handleObstacles finds no route ({calls.get('ROUTE_BLOCKED', 0):.2f} ms): engine state BLOCKED. "
                     "The navigator commands zero velocity.", "BLOCKED")
            stop = m.get("stop_time_after_terminal_s")
            drift = m.get("drift_after_stop_m")
            if stop is not None:
                self.add(t + stop, f"Robot stationary {stop:.2f} s after BLOCKED (ground truth); largest displacement "
                         f"afterwards {drift * 1000:.0f} mm, until the recording ends.", "stopped")
        self.items.sort(key=lambda item: item[0])
        self.times = [t for t, _ in self.items]

    def add(self, t, text, tick):
        self.items.append((t, text))
        if tick:
            self.ticks.append((t, tick))

    def caption(self, t):
        k = bisect.bisect_right(self.times, t) - 1
        return self.items[k] if k >= 0 else (None, "")


class Telemetry:
    """The navigator's recorded status (trajectory.json 'status', 10 Hz) and recorded events at time t."""

    def __init__(self, trajectory):
        cols = trajectory["status_columns"]
        self.rows = [dict(zip(cols, r)) for r in trajectory["status"]]
        self.times = [r["t"] for r in self.rows]
        self.blockages = sorted(trajectory["blockages"], key=lambda b: b["t"])
        self.calls = [e for e in trajectory["event_log"] if e["type"].startswith("ROUTE_")]

    def at(self, t):
        k = max(0, bisect.bisect_right(self.times, t) - 1)
        row = self.rows[k] if self.rows and self.times[0] <= t else None
        blockage = next((b for b in reversed(self.blockages) if b["t"] <= t), None)
        call = next((c for c in reversed(self.calls) if c["t"] <= t), None)
        return row, blockage, call


def draw_telemetry(canvas, box, t, telemetry):
    x0, y0, w, h = box
    d = ImageDraw.Draw(canvas)
    d.rectangle((x0, y0, x0 + w, y0 + h), fill=PANEL)
    d.text((x0 + 22, y0 + 14), "NAVIGATOR TELEMETRY", font=F["chip"], fill=MUTED)
    d.text((x0 + 22, y0 + 34), "recorded ane/status (20 Hz) and events", font=F["small"], fill=MUTED)
    row, blockage, call = telemetry.at(t)
    lines = [("simulation time", f"{t:7.2f} s", INK)]
    if row is None:
        lines.append(("mission state", "waiting for mission", MUTED))
    else:
        lines.append(("mission state", row["mission_state"], STATE_COLOUR.get(row["mission_state"], INK)))
        lines.append(("engine state", row["engine_state"], INK))
        if row["returning_to_node"]:
            waypoint = f"back to node {row['current_node']}"
        elif row["target_node"] >= 0:
            waypoint = f"node {row['current_node']} → {row['target_node']}"
        else:
            waypoint = f"at node {row['current_node']}"
        lines.append(("waypoint", waypoint, INK))
        cost = "∞ (no route)" if row["remaining_cost"] is None else f"{row['remaining_cost']:.2f}"
        lines.append(("route ahead, cost", f"{cost}  (v{row['route_version']})", INK))
        nonzero = lambda v, digits: 0.0 if round(v, digits) == 0 else v  # noqa: E731  (no "-0.0")
        lines.append(("tracking error", f"{nonzero(row['cross_track_m'] * 100, 1):+5.1f} cm", INK))
        lines.append(("command", f"{nonzero(row['cmd_linear_mps'], 2):.2f} m/s  "
                                 f"{nonzero(row['cmd_angular_radps'], 2):+.2f} rad/s", INK))
    if blockage is not None:
        source = "lidar (SENSOR)" if blockage["source"] == "SENSOR" else blockage["source"].lower()
        lines.append(("last blockage", f"{source}, {len(blockage['nodes'])} nodes", RED if blockage["source"] == "SENSOR" else PURPLE))
    if call is not None:
        verb = {"ROUTE_PLANNED": "planned", "ROUTE_REPLACED": "replaced", "ROUTE_KEPT": "kept",
                "ROUTE_BLOCKED": "no route"}.get(call["type"], call["type"])
        lines.append(("last engine call", f"{verb}, {call['value']:.3f} ms", INK))
    y = y0 + 70
    for key, value, colour in lines:
        d.text((x0 + 22, y + 2), key, font=F["k"], fill=MUTED)
        d.text((x0 + 232, y), value, font=F["vb"] if key == "mission state" else F["v"], fill=colour)
        y += 32
    d.text((x0 + 22, y0 + h - 26), "engine call: wall time on the container's steady clock", font=F["small"], fill=MUTED)


def rviz_panel(crop, width, height, legend):
    """The RViz2 crop scaled into a panel, with aisle labels at fixed map positions (the view is a fixed
    top-down projection centred on the map origin) and a legend strip underneath."""
    strip = 30
    view_h = height - strip
    view_w = round(view_h * RVIZ_SRC[0] / RVIZ_SRC[1])
    panel = Image.new("RGB", (width, height), (252, 252, 252))
    view = crop.resize((view_w, view_h), Image.LANCZOS)
    left = (width - view_w) // 2
    panel.paste(view, (left, 0))
    d = ImageDraw.Draw(panel, "RGBA")
    scale = 44 * view_w / RVIZ_SRC[0]
    to_px = lambda x, y: (left + view_w / 2 + x * scale, view_h / 2 - y * scale)  # noqa: E731
    for text, (x, y) in (("north aisle · picking zone, cost ×1.5", (-4.75, 3.62)), ("middle aisle", (-4.75, 0.62)),
                         ("south aisle", (-4.75, -2.38))):
        px, py = to_px(x, y)
        d.text((px, py), text, font=F["tick"], fill=(64, 72, 88, 255))
    y, x = height - strip + 7, 12
    for colour, text, kind in legend:
        if kind == "line":
            d.rounded_rectangle((x, y + 5, x + 18, y + 11), radius=3, fill=colour + (255,))
        else:
            d.ellipse((x + 4, y + 3, x + 14, y + 13), fill=colour + (255,))
        d.text((x + 23, y - 1), text, font=F["tick"], fill=(50, 56, 68, 255))
        x += 34 + d.textlength(text, font=F["tick"])
    return panel


def draw_timeline(d, box, t, t0, t1, ticks):
    """Recorded events on a line. Events closer together than their labels share one label
    ("pallet · lidar · replan"), so nothing overlaps."""
    x0, y0, w, _ = box
    span = max(t1 - t0, 1e-6)
    to_x = lambda tt: x0 + w * min(max((tt - t0) / span, 0.0), 1.0)  # noqa: E731
    d.line((x0, y0, x0 + w, y0), fill=RULE, width=4)
    px = to_x(t)
    d.line((x0, y0, px, y0), fill=ORANGE, width=4)
    clusters = []
    for tt, label in ticks:
        if clusters and to_x(tt) - to_x(clusters[-1][-1][0]) < 70:
            clusters[-1].append((tt, label))
        else:
            clusters.append([(tt, label)])
    for cluster in clusters:
        for tt, _ in cluster:
            d.line((to_x(tt), y0 - 6, to_x(tt), y0 + 6), fill=INK if tt <= t else MUTED, width=2)
        text = " · ".join(label for _, label in cluster)
        centre = sum(to_x(tt) for tt, _ in cluster) / len(cluster)
        width = d.textlength(text, font=F["tick"])
        left = min(max(centre - width / 2, x0 - 20), x0 + w + 20 - width)
        d.text((left, y0 + 8), text, font=F["tick"], fill=INK if cluster[0][0] <= t else MUTED)
    d.ellipse((px - 7, y0 - 7, px + 7, y0 + 7), fill=ORANGE)


def compose(frame, boxes, t, speed, story, telemetry, meta, span):
    canvas = Image.new("RGB", (W, H), BG)
    gx, gy, gw, gh = boxes["gazebo"]
    canvas.paste(frame.crop((gx, gy, gx + gw, gy + gh)), (0, HEADER))
    rx, ry, rw, rh = boxes["rviz"]
    crop = frame.crop((rx, ry, rx + rw, ry + rh)).convert("RGB")
    canvas.paste(rviz_panel(crop, W // 2, ROW_H, meta["legend"]), (0, HEADER + SCENE_H))
    draw_telemetry(canvas, (W // 2, HEADER + SCENE_H, W // 2, ROW_H), t, telemetry)

    d = ImageDraw.Draw(canvas, "RGBA")
    # Header: what this is.
    right = chip(d, (18, 12), "SIMULATION", fill=ORANGE + (255,), ink=BG, face=F["badge"], pad=(10, 2))
    d.text((right + 16, 8), "ROS 2 Jazzy + Gazebo Harmonic, software rendering. " + meta["title"], font=F["head"], fill=INK)
    d.text((right + 16, 36), "Robot pose: simulator ground truth (no localization). " + meta["scripted"],
           font=F["small"], fill=MUTED)
    # Panel labels.
    chip(d, (14, HEADER + 12), "Gazebo Harmonic: simulated robot and warehouse")
    speed_text = "playback 1× simulation time" if abs(speed - 1) < 1e-6 else f"playback {speed:g}× simulation time"
    chip(d, (W - 14 - d.textlength(speed_text, font=F["chip"]) - 20, HEADER + 12), speed_text,
         fill=(ORANGE + (235,)) if speed > 1 else (0, 0, 0, 170), ink=BG if speed > 1 else INK)
    chip(d, (14, HEADER + SCENE_H + 10), "RViz2: ROS data, top-down", fill=(30, 36, 46, 200))
    d.line((W // 2, HEADER + SCENE_H, W // 2, HEADER + SCENE_H + ROW_H), fill=RULE, width=2)
    # Footer: caption and timeline.
    y = HEADER + SCENE_H + ROW_H
    d.rectangle((0, y, W, H), fill=BG)
    when, text = story.caption(t)
    d.text((18, y + 10), f"t = {when:6.2f} s" if when is not None else "", font=F["time"], fill=ORANGE)
    for n, line in enumerate(wrap(d, text, F["caption"], W - 170 - 18)[:2]):
        d.text((170, y + 8 + 28 * n), line, font=F["caption"], fill=INK)
    draw_timeline(d, (40, y + 70, W - 80, 0), t, span[0], span[1], story.ticks)
    return canvas


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--capture-dir", type=Path, required=True)
    parser.add_argument("--trial-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--story", choices=("mission", "failure"), required=True)
    parser.add_argument("--name", default=None)
    parser.add_argument("--fps", type=int, default=10, help="output frames per second")
    parser.add_argument("--fast", type=float, default=6.0, help="preview speed outside the decisive seconds")
    parser.add_argument("--gif-width", type=int, default=800)
    parser.add_argument("--gif-fps", type=int, default=8)
    args = parser.parse_args()
    name = args.name or ("simulation-mission" if args.story == "mission" else "simulation-failure")

    clock = np.loadtxt(args.capture_dir / "capture_clock.csv", delimiter=",", skiprows=1)
    stamps = np.loadtxt(args.capture_dir / "frames.csv", delimiter=",", skiprows=1, ndmin=2)
    frames = [args.capture_dir / "frames" / f"{int(i):05d}.jpg" for i in stamps[:, 0]]
    frame_sim = np.interp(stamps[:, 1], clock[:, 0], clock[:, 1])  # each frame's monotonic stamp -> sim time
    trajectory = json.loads((args.trial_dir / "trajectory.json").read_text())
    metrics = json.loads((args.trial_dir / "metrics.json").read_text())
    manifest = json.loads((args.trial_dir / "manifest.json").read_text())
    story = Story(args.story, trajectory, metrics)
    telemetry = Telemetry(trajectory)
    windows = window_geometry(args.capture_dir / "windows.txt")
    gx, gy, gw, gh = windows["gazebo"]
    probe = Image.open(frames[len(frames) // 2])
    rx, ry, rw, rh = rviz_render_box(probe, windows["rviz"])
    rcx, rcy = rx + rw / 2, ry + rh / 2
    boxes = {"gazebo": (gx, gy + gh - SCENE_H, W, SCENE_H),
             "rviz": (int(rcx - RVIZ_SRC[0] / 2), int(rcy - RVIZ_SRC[1] / 2), *RVIZ_SRC)}
    if args.story == "mission":
        legend = [((40, 150, 70), "engine route", "line"), ((40, 100, 225), "trajectory (ground truth)", "line"),
                  ((215, 30, 30), "blocked: lidar", "dot"), ((255, 40, 15), "unexplained returns", "dot")]
    else:
        legend = [((40, 150, 70), "route", "line"), ((40, 100, 225), "trajectory", "line"),
                  ((210, 30, 30), "invalidated route", "line"), ((215, 30, 30), "blocked: lidar", "dot"),
                  ((180, 50, 205), "blocked: operator", "dot")]
    scenario = manifest["scenario"].split("_", 1)
    meta = {"title": f"Scenario {scenario[0]}, seed {manifest['seed']}, engine {manifest['engine_commit'][:7]}.",
            "scripted": ("Pallet placed by the scenario script." if args.story == "mission" else
                         "Aisle closures and pallet are scripted by the scenario."),
            "legend": legend}

    t_first = story.times[0] - 1.5
    t_last = (trajectory.get("terminal") or float(frame_sim[-1])) + (3.0 if args.story == "mission" else 4.0)
    t_last = min(t_last, float(frame_sim[-1]))
    if args.story == "mission":
        decisive = (trajectory["obstacles"][0]["t"] - 1.0, next(r["t"] for r in trajectory["routes"] if r["type"] == "ROUTE_REPLACED") + 4.0)
    else:
        decisive = (trajectory["obstacles"][0]["t"] - 1.0, (trajectory.get("terminal") or t_last) + 2.0)
    span = (t_first, t_last)

    def frame_at(t):
        return frames[int(np.argmin(np.abs(frame_sim - t)))]

    args.out_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        # Full recording: 1x simulation time.
        (tmp / "full").mkdir()
        times = np.arange(t_first, t_last, 1.0 / args.fps)
        for i, t in enumerate(times):
            canvas = compose(Image.open(frame_at(t)), boxes, float(t), 1.0, story, telemetry, meta, span)
            canvas.save(tmp / "full" / f"{i:05d}.png")
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(args.fps), "-i", str(tmp / "full" / "%05d.png"),
                        "-c:v", "libx264", "-preset", "slow", "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                        str(args.out_dir / f"{name}.mp4")], check=True)
        # Preview: decisive seconds at 1x, the rest faster; a two-second hold at the end.
        (tmp / "gif").mkdir()
        t, k, step = t_first, 0, 1.0 / args.gif_fps
        while t < t_last:
            speed = 1.0 if decisive[0] <= t <= decisive[1] else args.fast
            canvas = compose(Image.open(frame_at(t)), boxes, t, speed, story, telemetry, meta, span)
            canvas.resize((args.gif_width, round(H * args.gif_width / W)), Image.LANCZOS).save(tmp / "gif" / f"{k:05d}.png")
            k += 1
            t += step * speed
        for _ in range(2 * args.gif_fps):
            Image.open(tmp / "gif" / f"{k - 1:05d}.png").save(tmp / "gif" / f"{k:05d}.png")
            k += 1
        palette = ("split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];"
                   "[b][p]paletteuse=dither=bayer:bayer_scale=3:diff_mode=rectangle")
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(args.gif_fps), "-i", str(tmp / "gif" / "%05d.png"),
                        "-vf", palette, "-loop", "0", str(args.out_dir / f"{name}-preview.gif")], check=True)
        if args.story == "mission":
            replaced = next(r["t"] for r in trajectory["routes"] if r["type"] == "ROUTE_REPLACED")
            poster_t = min(replaced + 3.0, t_last)
            compose(Image.open(frame_at(poster_t)), boxes, poster_t, 1.0, story, telemetry, meta, span).save(
                args.out_dir / f"{name}-poster.png", optimize=True)
    info = {"frames_captured": len(frames), "frames_mp4": len(times), "span_sim_s": [round(t_first, 2), round(t_last, 2)],
            "decisive_sim_s": [round(decisive[0], 2), round(decisive[1], 2)], "gazebo_box": boxes["gazebo"],
            "rviz_box": boxes["rviz"], "rviz_render_area": [rx, ry, rw, rh]}
    print(json.dumps(info))
    write_provenance(args, name, manifest, info)
    for path in sorted(args.out_dir.glob(f"{name}*")):
        print(f"{path.name}: {path.stat().st_size / 1e6:.2f} MB")


def sha256(path):
    import hashlib
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_provenance(args, name, manifest, info):
    """<name>.json next to the media: what was recorded, by which code, and what this script drew."""
    campaign_file = args.trial_dir.parents[1] / "campaign.json"
    campaign = json.loads(campaign_file.read_text()) if campaign_file.exists() else {}
    outputs = {p.name: {"bytes": p.stat().st_size, "sha256": sha256(p)}
               for p in sorted(args.out_dir.glob(f"{name}*")) if p.suffix in (".mp4", ".gif", ".png")}
    record = {
        "schema": "ane-sim/media/1",
        "story": args.story,
        "outputs": outputs,
        "recorded_trial": {k: manifest.get(k) for k in ("campaign", "scenario", "scenario_file", "scenario_sha256", "trial",
                                                         "seed", "initial_pose", "engine_commit", "image_id", "mode")},
        "integration_tree_sha256": campaign.get("integration_tree_sha256"),
        "image": {k: campaign.get("image", {}).get(k) for k in ("tag", "id")},
        "trial_files": {f: sha256(args.trial_dir / f) for f in ("trajectory.json", "metrics.json", "camera_log.csv")
                        if (args.trial_dir / f).exists()},
        "capture": info | {"screen_frames": "up to 10 per wall-clock second, each stamped with the container's "
                                            "monotonic clock (frames.csv) and placed on the simulation clock "
                                            "through capture_clock.csv",
                           "preview_playback": {"decisive_window_sim_s": info["decisive_sim_s"], "decisive_speed": 1.0,
                                                "other_speed": args.fast, "gif_fps": args.gif_fps, "gif_width": args.gif_width},
                           "full_recording_playback": "1x simulation time, 10 frames per simulated second"},
        "drawn_by_this_script": "header, panel labels, telemetry panel (values from trajectory.json 'status' and 'event_log'), "
                                "captions and timeline (from recorded events and metrics.json), RViz2 aisle labels and legend. "
                                "The Gazebo and RViz2 pixels are screen captures of the running trial.",
    }
    (args.out_dir / f"{name}.json").write_text(json.dumps(record, indent=1) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
