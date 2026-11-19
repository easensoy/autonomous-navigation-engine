"""The 3D viewer: draws what `road_drive live` reports, and sends the operator's commands.

The backend owns the simulation: the vehicle, sensors, estimators, routing, controller and evaluator. This
process never moves the car itself; every pose it draws is one the backend reported for a tick. Rendering
runs at its own rate, and the simulation's fixed 10 ms tick never depends on it.
"""
import argparse
import datetime
import json
import math
import pathlib
import sys
import time

from panda3d.core import loadPrcFileData

VIEWER = pathlib.Path(__file__).resolve().parents[1]
QUALITY = {
    "low": {"msaa": 0, "shadow": 0},
    "medium": {"msaa": 4, "shadow": 1024},
    "high": {"msaa": 8, "shadow": 2048},
}


def parse_args(argv=None):
    p = argparse.ArgumentParser(prog="ane_viewer", description="Live 3D driving and localisation viewer for road_driving.")
    p.add_argument("--backend", help="path to road_drive (default: the build folders, or $ANE_BACKEND)")
    p.add_argument("--demo", default="C", choices=["A", "B", "C", "S", "K", "N", "SF", "SFB", "SFD", "SFM"],
                   help="demonstration to start with (default C; K Kensington, N Knightsbridge)")
    p.add_argument("--scenario", help="start a custom configuration instead of a demonstration")
    p.add_argument("--condition", default="nominal_r7")
    p.add_argument("--estimator", default="ekf_kerb2")
    p.add_argument("--seed", type=int, help="the seed (default 1); with --demo, it replaces the demonstration's")
    p.add_argument("--mode", default="estimated", choices=["estimated", "ground_truth"])
    p.add_argument("--rejoin", default="unique", choices=["unique", "verified", "on", "off"])
    p.add_argument("--vehicle", default="", choices=["", "ipace_proxy_v1", "cybercab_proxy_v1"],
                   help="the vehicle profile of a custom configuration (default: the backend's, ipace_proxy_v1)")
    p.add_argument("--replay", help="start by replaying this session file (REPLAY mode)")
    p.add_argument("--speed", type=float, default=1.0, help="simulated seconds per second (default 1)")
    p.add_argument("--quality", default="medium", choices=list(QUALITY))
    p.add_argument("--size", default="1600x900", help="window size, e.g. 1920x1080")
    p.add_argument("--fullscreen", action="store_true")
    p.add_argument("--no-vsync", action="store_true", help="render as fast as possible (for measuring)")
    p.add_argument("--camera", default="chase", choices=["chase", "overhead", "driver", "orbit"])
    p.add_argument("--vehicle-model", help="a vehicle JSON (see models/README.md) to draw an imported glTF instead of the proxy")
    p.add_argument("--paused", action="store_true", help="start paused")
    p.add_argument("--hide-panels", action="store_true")
    p.add_argument("--legend", action="store_true", help="show the legend at start")
    p.add_argument("--record-video", help="record the episode to this MP4 (lock-step, real time) and stop at its end")
    p.add_argument("--record-fps", type=int, default=50, help="video frame rate; it must divide the 100 Hz tick rate (default 50: 2 ticks a frame)")
    p.add_argument("--record-size", help="render the recording offscreen at this size, e.g. 1920x1080 (default: the window's)")
    p.add_argument("--crf", type=int, default=18, help="x264 quality (default 18)")
    p.add_argument("--preset", default="slow", help="x264 preset (default slow)")
    p.add_argument("--frame-log", help="log every recorded frame (tick, camera, HUD clock) to this JSON-lines file")
    p.add_argument("--hud", default="diagnostic", choices=["diagnostic", "present"], help="full diagnostic panels, or the compact presentation overlay")
    p.add_argument("--director", default="", help="a camera sequence for recordings: a plan name (KEN-A, KNB-A, default) or 'auto'")
    p.add_argument("--title", default="", help="a title card before the episode (recordings)")
    p.add_argument("--subtitle", default="")
    p.add_argument("--card-note", default="")
    p.add_argument("--card-seconds", type=float, default=3.0)
    p.add_argument("--end-seconds", type=float, default=2.5)
    p.add_argument("--stop-at", type=float, default=0.0, help="end a recording at this simulated time (s), for trial runs")
    p.add_argument("--camera-at", default="", help="camera changes at simulated times, e.g. '0:chase,40:orbit@25,50:overhead@60'")
    p.add_argument("--inject-at", default="", help="operator commands at simulated times, e.g. '15:gnss_bias,30:close,45:reopen,50:save'")
    p.add_argument("--save-session", help="save the session to this file when the episode ends")
    p.add_argument("--screenshots", default="", help="simulated times to save screenshots at, e.g. '20,54.5,60'")
    p.add_argument("--shot-dir", default=str(VIEWER / "captures"))
    p.add_argument("--shot-prefix", default="shot")
    p.add_argument("--exit-at-end", action="store_true", help="quit when the episode ends (after any recording)")
    p.add_argument("--benchmark", type=float, default=0.0, help="measure frame rate for this many wall seconds, after a second of warm-up; print it and quit")
    p.add_argument("--log", default=str(VIEWER / "captures" / "backend.log"), help="the backend's diagnostics (stderr)")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    w, h = (int(v) for v in args.size.lower().split("x"))
    q = QUALITY[args.quality]
    loadPrcFileData("", "window-title Autonomous Navigation Engine - live driving and localisation (independent research simulation)")
    loadPrcFileData("", "win-size %d %d" % (w, h))
    loadPrcFileData("", "fullscreen %s" % ("#t" if args.fullscreen else "#f"))
    loadPrcFileData("", "sync-video %s" % ("#f" if args.no_vsync else "#t"))
    loadPrcFileData("", "framebuffer-multisample %s" % ("#t" if q["msaa"] else "#f"))
    loadPrcFileData("", "multisamples %d" % q["msaa"])
    loadPrcFileData("", "textures-power-2 none")
    loadPrcFileData("", "notify-level-display error")
    from .backend import BUILD_HELP, find_backend
    exe = find_backend(args.backend)
    if exe is None:
        print(BUILD_HELP, file=sys.stderr)
        return 2
    viewer = Viewer(args, exe)
    viewer.run()
    return viewer.exit_code


from direct.showbase.ShowBase import ShowBase  # noqa: E402


class Viewer(ShowBase):
    def __init__(self, args, exe):
        ShowBase.__init__(self)
        from panda3d.core import AmbientLight, AntialiasAttrib, DirectionalLight, Fog, Vec4

        from . import demos, protocol
        from .backend import Backend
        from .cameras import Cameras
        from .hud import Hud
        from .overlays import Overlays
        from .state import SimState

        self.args = args
        self.protocol = protocol
        self.demos = demos
        self.exit_code = 0
        self.quality = args.quality
        self.disableMouse()
        self.setBackgroundColor(0.71, 0.77, 0.83, 1.0)
        self.render.setShaderAuto()
        if QUALITY[self.quality]["msaa"]:
            self.render.setAntialias(AntialiasAttrib.MMultisample)
        amb = AmbientLight("ambient")
        amb.setColor(Vec4(0.50, 0.52, 0.56, 1))
        self.render.setLight(self.render.attachNewNode(amb))
        sun = DirectionalLight("sun")
        sun.setColor(Vec4(0.80, 0.78, 0.72, 1))
        self.sun = self.render.attachNewNode(sun)
        self.sun.setHpr(35, -55, 0)
        self.render.setLight(self.sun)
        size = QUALITY[self.quality]["shadow"]
        if size:
            sun.setShadowCaster(True, size, size)
            sun.getLens().setFilmSize(140, 140)
            sun.getLens().setNearFar(10, 400)
        fog = Fog("haze")
        fog.setColor(0.71, 0.77, 0.83)
        fog.setLinearRange(220.0, 900.0)
        self.render.setFog(fog)

        self.font, self.bold = self._fonts()
        if self.font:
            from panda3d.core import TextNode
            TextNode.setDefaultFont(self.font)
        self.amb = amb
        self.state = SimState()
        self.overlays = Overlays(self.render)
        self.cameras = Cameras(self)
        self.cameras.set_mode(args.camera)
        self.world = None
        self.vehicle = None
        self.ghost = None
        self.model_cfg = None
        if args.vehicle_model:
            from .vehicle3d import load_model_config
            self.model_cfg = load_model_config(args.vehicle_model)
        self._scene_built = 0
        self._session_seen = 0
        self._frame = 0
        self._last_tick_t = None
        self._hud_t = 0.0
        self.narrator = None
        self.demo = None
        self.pending = {}
        self.paused = bool(args.paused)
        self.last_saved = None
        self._replay_path = str(pathlib.Path(args.replay).resolve()) if args.replay else None
        self.video = None
        self._lock_target = None
        self._capture_next = False
        self.fps_samples = []
        self._bench_t0 = None
        self.shots = sorted(float(t) for t in args.screenshots.split(",") if t.strip())
        self.camera_plan = sorted((float(t), m) for t, m in (item.split(":") for item in args.camera_at.split(",") if item.strip()))
        self.inject_plan = sorted((float(t), c) for t, c in (item.split(":") for item in args.inject_at.split(",") if item.strip()))
        self._end_saved = False
        pathlib.Path(args.shot_dir).mkdir(parents=True, exist_ok=True)

        actions = {
            "pause": self.toggle_pause, "step": lambda: self.step_ticks(10), "reset": self.restart_episode, "speed": self.set_speed,
            "demo": self.start_demo, "cycle": self.cycle, "apply": self.apply_config, "gnss_outage": self.toggle_gnss_outage,
            "gnss_bias": self.toggle_gnss_bias, "kerb_blackout": self.toggle_kerb_blackout, "close": self.close_road, "reopen": self.reopen_road,
            "save": self.save_session, "replay": self.replay_last, "camera": self.set_camera, "legend": self.toggle_legend, "help": self.toggle_help,
            "quality": self.cycle_quality, "record": self.toggle_record,
        }
        self.hud = Hud(self, actions)
        if args.hide_panels:
            self.hud.toggle_visible()
        if args.legend:
            self.hud.legend.show()
        from .presentation import PresentHud
        self.present = PresentHud(self, self.font, self.bold)
        self.hud_mode = None
        self.set_hud_mode(args.hud)
        self.kerb = None
        self.director = None
        self.target = None
        self.frame_log = None
        self.rec_phase = None
        self.rec_cards = 0
        self.k_ticks = 2
        self.clock_text = ""

        backend_args = ["--speed", "%g" % args.speed]
        if args.paused or args.record_video:
            backend_args += ["--paused", "on"]
        if args.replay:
            backend_args += ["--replay", str(pathlib.Path(args.replay).resolve())]
        pathlib.Path(args.log).parent.mkdir(parents=True, exist_ok=True)
        self.backend = Backend(exe, backend_args, log_path=args.log).start()
        if not args.replay:
            if args.scenario:
                self.pending = {"scenario": args.scenario, "condition": args.condition, "mode": args.mode, "estimator": args.estimator,
                                "seed": 1 if args.seed is None else args.seed, "rejoin": args.rejoin}
                if args.vehicle:
                    self.pending["vehicle"] = args.vehicle
                self.backend.send(protocol.reset(self.pending))
            else:
                self.start_demo(args.demo, args.seed)
        self._bind_keys()
        self.taskMgr.add(self.update, "update")
        self.exitFunc = self.stop_backend

    # ---- Input ------------------------------------------------------------------------------------
    def _bind_keys(self):
        k = self.accept
        k("escape", self.userExit)
        k("space", self.toggle_pause)
        k("n", self.step_ticks, [10])
        k("shift-n", self.step_ticks, [1])
        k("r", self.restart_episode)
        for key, d in (("f1", "A"), ("f2", "B"), ("f3", "C"), ("f4", "S")):
            k(key, self.start_demo, [d])
        for key, mode in (("1", "chase"), ("2", "overhead"), ("3", "driver"), ("4", "orbit")):
            k(key, self.set_camera, [mode])
        k("g", self.toggle_gnss_outage)
        k("b", self.toggle_gnss_bias)
        k("k", self.toggle_kerb_blackout)
        k("c", self.close_road)
        k("o", self.reopen_road)
        k("s", self.save_session)
        k("p", self.replay_last)
        k("l", self.toggle_legend)
        k("h", self.toggle_help)
        k("q", self.cycle_quality)
        k("t", self.overlays.toggle, ["truth_trail"])
        k("e", self.toggle_estimate)
        k("y", self.overlays.toggle, ["placed"])
        k("z", self.toggle_zones)
        k("d", self.detach_orbit)
        k("tab", self.hud.toggle_visible)
        k("m", self.toggle_hud_mode)
        k("v", self.toggle_provenance)
        for key, d in (("f5", "K"), ("f6", "N"), ("f7", "SF")):
            k(key, self.start_demo, [d])
        k("f12", self.screenshot_now)
        k("f9", self.toggle_record)
        k("[", self.speed_step, [0.5])
        k("]", self.speed_step, [2.0])
        k("wheel_up", self.cameras.zoom, [0.88])
        k("wheel_down", self.cameras.zoom, [1.14])

    def send_cmd(self, cmd):
        if not self.backend.send(cmd):
            self.state.errors.append({"message": "backend not running"})

    def toggle_pause(self):
        self.paused = not self.paused
        self.send_cmd(self.protocol.pause() if self.paused else self.protocol.resume())

    def step_ticks(self, ticks):
        self.paused = True
        self.send_cmd(self.protocol.step(ticks))

    def restart_episode(self):
        if self.state.mode == "replay" and self.state.session:
            self.send_cmd(self.protocol.replay(self._replay_path))
            return
        cfg = dict(self.state.config) if self.state.config else self.pending
        self.send_cmd(self.protocol.reset({k: cfg[k] for k in ("scenario", "condition", "mode", "estimator", "seed", "rejoin", "vehicle") if k in cfg}))

    def set_speed(self, v):
        self.send_cmd(self.protocol.speed(v))

    def speed_step(self, f):
        self.set_speed(max(0.125, min(16.0, self.state.status.get("speed", 1.0) * f)))

    def start_demo(self, key, seed=None):
        self.demo = self.demos.by_key(key)
        self.narrator = self.demos.Narrator(self.demo)
        self.pending = dict(self.demo.config)
        if seed is not None:
            self.pending["seed"] = seed
        self.paused = bool(self.args.paused)
        self.send_cmd(self.protocol.reset(self.pending))

    def cycle(self, field, d):
        demos = self.demos
        cfg = self.pending or dict(self.state.config)
        if field == "scenario":
            opts = demos.ROUTES
        elif field == "condition":
            opts = demos.CONDITIONS[demos.network_of(cfg.get("scenario", "R7-A"))]
        elif field == "estimator":
            opts = demos.ESTIMATORS
        elif field == "seed":
            opts = list(range(1, 11))
        elif field == "mode":
            opts = demos.MODES
        else:
            opts = demos.REJOINS
        cur = cfg.get(field)
        i = opts.index(cur) if cur in opts else -d
        cfg[field] = opts[(i + d) % len(opts)]
        if field == "scenario":
            conds = demos.CONDITIONS[demos.network_of(cfg["scenario"])]
            if cfg.get("condition") not in conds:
                cfg["condition"] = conds[0]
        self.pending = cfg

    def apply_config(self):
        self.demo = None
        self.narrator = None
        self.send_cmd(self.protocol.reset(self.pending))

    def _injected(self):
        tk = self.state.tick or {}
        return ((tk.get("sensors") or {}).get("injected") or {})

    def toggle_gnss_outage(self):
        self.send_cmd(self.protocol.gnss_outage(not self._injected().get("gnss_outage", False)))

    def toggle_gnss_bias(self):
        on = not any(self._injected().get("gnss_bias", [0, 0]))
        self.send_cmd(self.protocol.gnss_bias(on, 5.0, 90.0))

    def toggle_kerb_blackout(self):
        self.send_cmd(self.protocol.kerb_blackout(not self._injected().get("kerb_blackout", False)))

    def close_road(self):
        self.send_cmd(self.protocol.closure_ahead(40.0))

    def reopen_road(self):
        self.send_cmd(self.protocol.closure_lift())

    def save_session(self):
        d = VIEWER / "sessions"
        d.mkdir(exist_ok=True)
        cfg = self.state.config
        name = "session_%s_%s_%s_s%s.json" % (datetime.datetime.now().strftime("%Y%m%d_%H%M%S"), cfg.get("scenario", "x"), cfg.get("condition", "x"),
                                              cfg.get("seed", "x"))
        self.last_saved = str(d / name)
        self.send_cmd(self.protocol.save_session(self.last_saved))

    def replay_last(self):
        path = self.last_saved
        if not path:
            files = sorted((VIEWER / "sessions").glob("*.json"), key=lambda p: p.stat().st_mtime)
            path = str(files[-1]) if files else None
        if not path:
            self.state.errors.append({"message": "no saved session to replay yet (press S to save one)"})
            return
        self._replay_path = path
        self.send_cmd(self.protocol.replay(path))

    def set_camera(self, mode):
        mode, _, value = mode.partition("@")
        tk = self.state.tick
        self.cameras.set_mode(mode, tk["truth"]["yaw"] if tk else None)
        if value:
            if mode == "overhead":
                self.cameras.overhead_height = float(value)
            elif mode == "orbit":
                self.cameras.orbit_dist = float(value)

    def inject(self, name):
        p = self.protocol
        cmd = {"gnss_outage": p.gnss_outage(True), "gnss_outage_off": p.gnss_outage(False), "gnss_bias": p.gnss_bias(True, 5.0, 90.0),
               "gnss_bias_off": p.gnss_bias(False), "kerb_blackout": p.kerb_blackout(True), "kerb_blackout_off": p.kerb_blackout(False),
               "close": p.closure_ahead(40.0), "reopen": p.closure_lift()}.get(name)
        if name == "save":
            self.save_session()
        elif cmd:
            self.send_cmd(cmd)

    def detach_orbit(self):
        tk = self.state.tick
        if tk and self.cameras.mode == "orbit":
            self.cameras.detach(tk["truth"]["x"], tk["truth"]["y"])

    def toggle_legend(self):
        (self.hud.legend.hide if not self.hud.legend.isHidden() else self.hud.legend.show)()

    def toggle_help(self):
        (self.hud.help.hide if not self.hud.help.isHidden() else self.hud.help.show)()

    def toggle_estimate(self):
        on = self.overlays.toggle("est_trail")
        self.overlays.toggle("ellipse")
        if self.ghost:
            (self.ghost.root.show if on else self.ghost.root.hide)()

    def _fonts(self):
        """A clear sans-serif for the panels where one is installed (Segoe UI on Windows, DejaVu elsewhere)."""
        for regular, bold in (("C:/Windows/Fonts/segoeui.ttf", "C:/Windows/Fonts/segoeuib.ttf"),
                              ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf")):
            if pathlib.Path(regular).is_file():
                from panda3d.core import Filename
                fonts = []
                try:
                    for path in (regular, bold if pathlib.Path(bold).is_file() else regular):
                        f = self.loader.loadFont(Filename.fromOsSpecific(path).getFullpath())
                        f.setPixelsPerUnit(72)
                        fonts.append(f)
                except (IOError, OSError, TypeError):
                    return None, None
                return fonts[0], fonts[1]
        return None, None

    def set_hud_mode(self, mode):
        self.hud_mode = mode
        present = mode == "present"
        self.present.show(present)
        for w in (self.hud.banner, self.hud.title, self.hud.disclaimer):
            (w.hide if present else w.show)()
        if present and self.hud.visible:
            self.hud.toggle_visible()
        elif not present and not self.hud.visible and not self.args.hide_panels:
            self.hud.toggle_visible()
        self.overlays.labels = not present
        self.overlays.compact = present

    def toggle_hud_mode(self):
        self.set_hud_mode("diagnostic" if self.hud_mode == "present" else "present")

    def toggle_provenance(self):
        if self.world:
            n = self.world.find("provenance")
            if not n.isEmpty():
                (n.hide if not n.isHidden() else n.show)()

    def toggle_zones(self):
        if self.world:
            z = self.world.find("zones")
            (z.hide if not z.isHidden() else z.show)()

    def cycle_quality(self):
        names = list(QUALITY)
        self.quality = names[(names.index(self.quality) + 1) % len(names)]
        self._scene_built = -1  # rebuild the scenery at the new density (shadows and MSAA are set at start)

    # ---- Capture ----------------------------------------------------------------------------------
    def screenshot_now(self, label=None):
        from panda3d.core import Filename
        tk = self.state.tick
        t = tk["t"] if tk else 0.0
        name = "%s_%s_t%06.2f.png" % (self.args.shot_prefix, label or datetime.datetime.now().strftime("%H%M%S"), t)
        path = pathlib.Path(self.args.shot_dir) / name
        self._update_hud()  # the panels at this tick, like the scene, then this frame rather than the last one
        self.graphicsEngine.renderFrame()
        self.graphicsEngine.renderFrame()
        self.win.saveScreenshot(Filename.fromOsSpecific(str(path)))
        return path

    def toggle_record(self, path=None):
        from .capture import FrameLog, OffscreenTarget, VideoWriter, ticks_per_frame
        if self.video:
            self._stop_recording()
            return
        if not path:
            path = pathlib.Path(self.args.shot_dir) / ("video_%s.mp4" % datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        fps = self.args.record_fps
        self.k_ticks = ticks_per_frame(fps)
        if self.args.record_size:
            w, h = (int(v) for v in self.args.record_size.lower().split("x"))
            self.target = OffscreenTarget(self, w, h, QUALITY[self.quality]["msaa"] or 8)
            print("recording offscreen at %dx%d, %dx MSAA" % (w, h, self.target.samples), flush=True)
        else:
            props = self.win.getProperties()
            w, h = props.getXSize(), props.getYSize()
        self.video = VideoWriter(path, w, h, fps=fps, crf=self.args.crf, preset=self.args.preset)
        if self.args.frame_log:
            self.frame_log = FrameLog(self.args.frame_log)
        self.send_cmd(self.protocol.pause())
        self.paused = True
        self._lock_target = None
        self._last_run_tick = None
        self.rec_phase = "title" if self.args.title else "run"
        self.rec_cards = 0
        self.hud.set_toggle("record", True, "Stop video")

    def _stop_recording(self):
        out = self.video.close()
        print("video: %s (%d frames at %d fps)" % (out, self.video.frames, self.video.fps), flush=True)
        if self.frame_log:
            self.frame_log.close()
            self.frame_log = None
        if self.target:
            self.target.close(self)
            self.target = None
        self.video = None
        self.rec_phase = None
        self.present.hide_card()
        self.hud.set_toggle("record", False, "Record video")
        if not self.state.end:
            self.paused = False
            self.send_cmd(self.protocol.resume())

    def _view(self, dt):
        """The camera (and the director's cut, the labels' scale, the sun) for the tick on screen."""
        tk = self.state.tick
        tr = tk["truth"]
        while self.camera_plan and tk["t"] >= self.camera_plan[0][0]:
            self.set_camera(self.camera_plan.pop(0)[1])
        if self.director:
            self.director.update(self.state, dt)
        self.cameras.update(dt, tr)
        self.overlays.scale_labels(self.camera.getPos(self.render))
        self.sun.setPos(tr["x"] + 60.0, tr["y"] - 80.0, 120.0)
        self.sun.lookAt(tr["x"], tr["y"], 0.0)

    def _capture(self, phase):
        """Draw this tick's frame (camera, HUD) and hand it to the encoder; log it. The camera advances by one
        frame of simulated time (k ticks), so its path is the same however fast frames render."""
        tk = self.state.tick
        self._view(self.k_ticks * 0.01)
        self._update_hud()
        self.graphicsEngine.renderFrame()
        ok = self.video.add_texture(self.target.tex) if self.target else self.video.add(self)
        if self.frame_log:
            info = self.cameras.info
            self.frame_log.write(tick=tk["tick"] if tk else None, t=tk["t"] if tk else None, phase=phase,
                                 shot=self.director.current if self.director else self.cameras.mode, camera=self.cameras.mode, pos=info.get("pos"),
                                 inside=bool(info.get("inside")), share=round(info.get("share", 1.0), 3), clock=self.clock_text, ok=ok)

    def _lockstep(self):
        """Video: the frame for each recorded tick, then advance exactly k ticks (2 at 50 fps)."""
        tk = self.state.tick
        if tk is None:
            # A recording starts the backend paused, before its first tick: ask for that tick (t = 0.01 s).
            if self.vehicle is not None and not getattr(self, "_first_step", False):
                self._first_step = True
                self.send_cmd(self.protocol.step(self.k_ticks))  # every frame then falls on a multiple of k ticks
            return
        if self.frame_log and not getattr(self, "_config_noted", False):
            self._config_noted = True
            self.frame_log.note({"type": "config", "config": self.state.config, "labels": (self.state.session or {}).get("labels"),
                                 "backend": (self.state.session or {}).get("backend"), "quality": self.quality,
                                 "size": [self.video.width, self.video.height], "fps": self.video.fps, "ticks_per_frame": self.k_ticks,
                                 "msaa": self.target.samples if self.target else None, "crf": self.args.crf, "preset": self.args.preset,
                                 "renderer": self.win.getGsg().getDriverRenderer(), "director": self.args.director})
        if self.vehicle is None:
            return
        fps = self.video.fps
        self.mainWinMinimized = 0
        if self.rec_phase == "title":
            if self.rec_cards == 0:
                self.present.show_card(self.args.title, self.args.subtitle, self.args.card_note)
            self._capture("title")
            self.rec_cards += 1
            if self.rec_cards >= int(round(self.args.card_seconds * fps)):
                self.present.hide_card()
                self.rec_phase = "run"
                self.rec_cards = 0
            return
        if self.rec_phase == "end":
            self._capture("end")
            self.rec_cards += 1
            if self.rec_cards >= int(round(self.args.end_seconds * fps)):
                self._stop_recording()
            return
        if self._lock_target is None:
            if tk["tick"] != self._last_run_tick:
                self._capture("run")
                self._last_run_tick = tk["tick"]
            # else the episode ended on the tick just recorded, and its end message came in after that frame
            if self.args.stop_at and tk["t"] >= self.args.stop_at:
                self._stop_recording()
                self.userExit()
                return
            if self.state.end is not None:
                self.rec_phase = "end"
                self.rec_cards = 0
                e = self.state.end
                if self.frame_log:
                    self.frame_log.note(dict(e, director=self.director.log if self.director else []))
                m = e.get("metrics", {})
                self.present.show_card(e["outcome"].replace("_", " ").title() + " at %.1f s" % e["t"],
                                       "position error RMSE %.2f m, largest %.2f m;  deepest kerb crossing %.2f m" % (
                                           m.get("est_rmse", -1.0), m.get("est_max", -1.0), m.get("max_crossing", 0.0)),
                                       "trace %s  |  the backend's own evaluation of this run, from simulator truth" % e.get("trace_hash", "-"))
                return
            self._lock_target = tk["tick"] + self.k_ticks
            self.send_cmd(self.protocol.step(self.k_ticks))
        elif tk["tick"] >= self._lock_target or self.state.end is not None:
            self._lock_target = None

    # ---- Frame ------------------------------------------------------------------------------------
    def _rebuild_world(self):
        from . import world
        from .vehicle3d import Vehicle
        if self.world is not None:
            self.world.removeNode()
        scene = self.state.scene
        self.world = world.build(scene, self.quality)
        self.world.reparentTo(self.render)
        self.cameras.set_occluders(self.world.getPythonTag("occluders"))
        from .district_layout import Kerb
        try:
            self.kerb = Kerb(scene["map"])
        except ValueError:
            self.kerb = None
        district = scene["map"].get("district")
        report = self.world.getPythonTag("district_report")
        if report:
            print("district scene: " + json.dumps(report), flush=True)
        # District scenes: a softer, warmer daylight with a cool fill from the opposite side, and a lighter
        # tint on the sensor-condition zones, which would otherwise hide the road surface.
        if district:
            self.amb.setColor((0.46, 0.49, 0.55, 1))
            self.sun.node().setColor((0.92, 0.88, 0.78, 1))
            z = self.world.find("zones")
            if not z.isEmpty():
                z.setAlphaScale(0.55)
                if self.hud_mode == "present":
                    z.hide()  # the presentation overlay reports the conditions; the zone tint would hide the road (key Z shows it)
        plan_name = self.args.director
        if plan_name:
            from .director import PLANS, Director
            key = plan_name if plan_name in PLANS else (self.state.config.get("scenario") if plan_name == "auto" else "default")
            self.director = Director(self, PLANS.get(key, PLANS["default"]))
        # The proxy for the scene's vehicle profile, with the GNSS antenna where the profile mounts it.
        params = dict(scene["vehicle"], antenna_x=((scene.get("sensors") or {}).get("gnss") or {}).get("antenna_x"))
        if self.vehicle is None or self.vehicle.params != params:
            if self.vehicle:
                self.vehicle.root.removeNode()
                self.ghost.root.removeNode()
            self.vehicle = Vehicle(self.loader, params, self.model_cfg)
            self.vehicle.root.reparentTo(self.render)
            if self.vehicle.report:
                print("imported vehicle alignment: " + json.dumps(self.vehicle.report), flush=True)
            self.ghost = Vehicle(self.loader, params, None, ghost=True)
            self.ghost.root.reparentTo(self.render)
        self._scene_built = self.state.scene_version

    def update(self, task):
        dt = globalClock.getDt()  # noqa: F821 - Panda3D builtin
        self._frame += 1
        for msg in self.backend.poll():
            self.state.apply(msg)
        if self.state.session_version != self._session_seen:
            self._session_seen = self.state.session_version
            self.overlays.clear()
            self._last_tick_t = None
            if self.narrator and self.state.mode == "live" and self.demo and self.state.config.get("scenario") == self.demo.config["scenario"]:
                self.narrator = self.demos.Narrator(self.demo)
            if self.state.mode == "replay":
                self.narrator = None
        if self.state.scene is not None and self._scene_built != self.state.scene_version:
            self._rebuild_world()
        if self.narrator:
            self.narrator.update(self.state)
        tk = self.state.tick
        if tk is not None and self.vehicle is not None:
            tr = tk["truth"]
            sim_dt = 0.0 if self._last_tick_t is None else max(0.0, tk["t"] - self._last_tick_t)
            self._last_tick_t = tk["t"]
            self.vehicle.set_pose(tr["x"], tr["y"], tr["yaw"], tr["steer"], tr["v"], sim_dt)
            est = tk.get("est")
            inside = self.cameras.mode == "driver"  # the camera sits in the car: draw neither car around it
            (self.vehicle.root.hide if inside else self.vehicle.root.show)()
            if est and est.get("valid") and self.overlays.show["est_trail"] and not inside:
                self.ghost.root.show()
                self.ghost.set_pose(est["x"], est["y"], est["yaw"])
            else:
                self.ghost.root.hide()
            self.overlays.update(self.state, self._frame if self.video is None else 0)
            self.overlays.footprints(self.state.scene["vehicle"], tr, est)
            self.cameras.mouse(self.mouseWatcherNode.isButtonDown("mouse3"), self.mouseWatcherNode.isButtonDown("mouse2"))
            if self.video is None:
                self._view(dt)
            while self.inject_plan and tk["t"] >= self.inject_plan[0][0] and self.state.mode == "live":
                self.inject(self.inject_plan.pop(0)[1])
            while self.shots and tk["t"] >= self.shots[0]:
                self.screenshot_now("%05.1f" % self.shots.pop(0))
        now = time.time()
        if now - self._hud_t > 0.1:
            self._hud_t = now
            self._update_hud()
        if dt > 0:
            self.fps_samples.append(dt)
        if (self.args.record_video and self.video is None and self.state.scene is not None and self.vehicle is not None
                and not getattr(self, "_recorded", False) and self.state.mode != "idle"):
            self._recorded = True
            self.toggle_record(self.args.record_video)
        if self.video:
            self._lockstep()
        if self.args.benchmark and tk is not None:
            if self._bench_t0 is None:
                self._bench_t0 = now + 1.0  # measured from a second after the first tick: the scene's shaders compile first
            elif now < self._bench_t0:
                self.fps_samples = []
            elif now - self._bench_t0 >= self.args.benchmark:
                self._report_benchmark(now - self._bench_t0)
                self.userExit()
        if self.args.save_session and self.state.end is not None and not self._end_saved:
            self._end_saved = True
            self.send_cmd(self.protocol.save_session(str(pathlib.Path(self.args.save_session).resolve())))
        if self.args.exit_at_end and self.state.end is not None and self.video is None:
            if not getattr(self, "_end_seen", None):
                self._end_seen = now
            elif now - self._end_seen > 1.0:
                self.userExit()
        if not self.backend.alive() and not getattr(self, "_dead_reported", False):
            self._dead_reported = True
            self.state.errors.append({"message": "the backend has stopped (see %s)" % self.args.log})
        return task.cont

    def _report_benchmark(self, wall):
        s = sorted(self.fps_samples)
        n = len(s)
        props = self.win.getProperties()
        out = {"wall_s": round(wall, 2), "frames": n, "fps_mean": round(n / sum(s), 1) if n else 0, "frame_ms_median": round(1000 * s[n // 2], 2) if n else 0,
               "frame_ms_p95": round(1000 * s[int(0.95 * (n - 1))], 2) if n else 0, "frame_ms_max": round(1000 * s[-1], 2) if n else 0,
               "rtf": self.state.status.get("rtf"), "lag_events": self.state.status.get("lag_events"), "quality": self.quality,
               "window": "%dx%d" % (props.getXSize(), props.getYSize()), "ticks_received": self.state.ticks_seen, "messages": self.backend.received,
               "gl_renderer": self.win.getGsg().getDriverRenderer(), "gl_vendor": self.win.getGsg().getDriverVendor()}
        print("benchmark: " + json.dumps(out), flush=True)

    def _update_hud(self):
        st = self.state
        h = self.hud
        mode = st.mode
        if self.hud_mode == "present":
            self.clock_text = self.present.update(st, "REPLAY" if mode == "replay" else "LIVE", recording=self.video is not None)
        run = (st.status.get("state") or "starting").upper()
        if mode == "replay":
            h.banner.setText("REPLAY  -  recorded session, %s" % run)
            h.banner["fg"] = (1.0, 0.62, 0.15, 1.0)
        elif self.video:
            h.banner.setText("LIVE  -  recording in lock-step: %d ticks per frame, %d frames per second" % (self.k_ticks, self.video.fps))
            h.banner["fg"] = (0.30, 0.95, 0.40, 1.0)
        else:
            h.banner.setText("LIVE  -  simulation %s" % run)
            h.banner["fg"] = (0.30, 0.95, 0.40, 1.0)
        fps = 1.0 / (sum(self.fps_samples[-60:]) / len(self.fps_samples[-60:])) if self.fps_samples else 0.0
        h.title.setText("%s   |   render %.0f fps   |   %s" % (self.demo.title if self.demo and mode == "live" else ("Custom configuration" if mode == "live" else "Replay"),
                                                         fps, "video: recording %d frames" % self.video.frames if self.video else "camera: " + self.cameras.mode))
        rows = st.readout()
        h.labels.setText("\n".join(r[0] for r in rows))
        h.values.setText("\n".join(r[1] for r in rows))
        h.fit_panel(len(rows))
        h.set_disclaimer(bool(((st.scene or {}).get("map") or {}).get("district")), ((st.scene or {}).get("vehicle") or {}).get("id", "ipace_proxy_v1"))
        evs = list(st.events)[-9:]
        lines = ["%7.2f  %-9s %-22s %s" % (e["t"], e["source"], e["event"], e["detail"][:46]) for e in evs]
        errs = list(st.errors)[-2:]
        lines += ["ERROR  " + e.get("message", "")[:90] for e in errs]
        if st.saved:
            lines.append("saved  %s (%d commands%s)" % (pathlib.Path(st.saved["path"]).name, st.saved["commands"], "" if st.saved["complete"] else ", episode not finished"))
        if st.end:
            rc = st.end.get("replay_check")
            check = ""
            if rc:
                check = "  replay %s the recorded trace" % ("REPRODUCED" if rc["hash_equal"] else "DIFFERS FROM")
            lines.append("END  %s at %.2f s, trace %s%s" % (st.end["outcome"], st.end["t"], st.end["trace_hash"], check))
        h.events.setText("\n".join(lines[-11:]))
        caption = self.narrator.text() if self.narrator else ("Replaying a recorded session: its commands take effect at their recorded ticks."
                                                              if mode == "replay" else "")
        h.caption.setText(caption)
        h.fit_caption()
        cfg = self.pending or st.config
        for field, node in h.cycle_text.items():
            node.setText("%s: %s" % (field, cfg.get(field, "-")))
        h.set_toggle("pause", self.paused, "Resume" if self.paused else "Pause")
        inj = self._injected()
        h.set_toggle("gnss_outage", inj.get("gnss_outage", False))
        h.set_toggle("gnss_bias", any(inj.get("gnss_bias", [0, 0])))
        h.set_toggle("kerb_blackout", inj.get("kerb_blackout", False))
        for k in ("chase", "overhead", "driver", "orbit"):
            h.set_toggle("cam_" + k, self.cameras.mode == k)
        for k in ("A", "B", "C", "S", "K", "N", "SF"):
            h.set_toggle("demo" + k, bool(self.demo) and self.demo.key == k and mode == "live")
        h.set_toggle("quality", False, self.quality.capitalize())

    def windowEvent(self, win):
        ShowBase.windowEvent(self, win)
        if self.args.record_video or getattr(self, "video", None) is not None:
            # ShowBase sleeps 100 ms a frame while the main window is minimized. A recording draws offscreen
            # and never needs the preview window on screen, so it keeps going at full speed.
            self.mainWinMinimized = 0

    def stop_backend(self):
        if self.video:
            self.video.close()
        code = self.backend.stop()
        print("backend exited with code %s" % code, flush=True)
