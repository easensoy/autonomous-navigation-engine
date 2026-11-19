"""On-screen panels: mode banner, readouts, events, demonstration captions, controls, legend and help."""
from direct.gui import DirectGuiGlobals as DGG
from direct.gui.DirectGui import DirectButton, DirectFrame
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import TextNode

PANEL = (0.04, 0.05, 0.07, 0.78)
BUTTON = (0.16, 0.18, 0.22, 0.95)
BUTTON_ON = (0.10, 0.45, 0.45, 0.95)
TEXT = (0.93, 0.94, 0.96, 1.0)
MUTED = (0.65, 0.69, 0.75, 1.0)
LIVE = (0.30, 0.95, 0.40, 1.0)
REPLAY = (1.00, 0.62, 0.15, 1.0)
DISCLAIMER = ("Independent research simulation  |  synthetic streets (not surveyed London locations)  |  "
              "simulated I-PACE-sized proxy, not a Jaguar  |  not Waymo software or data")
DISCLAIMER_DISTRICT = ("Independent research simulation  |  source-informed reconstruction, details approximated; map data © OpenStreetMap "
                       "contributors (ODbL)  |  simulated I-PACE-sized proxy, not a Jaguar  |  not Waymo software or data")
READOUT_SCALE = 0.029
LEGEND = [
    ((0.96, 0.97, 0.98, 1), "solid car / white trail: simulated truth (drawing and diagnostics only)"),
    ((0.20, 0.85, 0.85, 1), "teal outline and trail: the estimate the stack drives on"),
    ((0.20, 0.95, 0.95, 1), "teal ring: the filter's own 95% position ellipse; lines: heading spread"),
    ((1.00, 0.72, 0.15, 1), "amber dashes: the stack's planned reference (with shift and re-join)"),
    ((0.72, 0.45, 0.98, 1), "violet: raw GNSS fixes (newest largest)"),
    ((0.25, 0.95, 0.35, 1), "green / red returns: kerb scan used / rejected by the filter"),
    ((0.30, 0.90, 1.00, 1), "cyan (toggle): the same returns placed from the estimated pose"),
    ((1.00, 0.55, 0.08, 1), "orange stripe on the bumper: the active planar kerb scanner (37 beams, ±90°, 0.5-25 m)"),
    ((0.65, 0.69, 0.75, 1), "simulated: GNSS, IMU, wheel and steering sensors, kerb scanner; any roof rig is decorative, no lidar or camera"),
    ((0.95, 0.25, 0.25, 1), "shaded ground: sensor-condition zones (outage, multipath, ramp, kerb blackout, map error)"),
    ((1.00, 0.50, 0.05, 1), "orange translucent kerb: where the vehicle's map has a kerb the world does not (map error)"),
]
HELP = """Keys
  Space  pause / resume          N  step 0.1 s (Shift+N: one 10 ms tick)      R  restart this configuration
  F1 F2 F3 F4  demonstrations A, B, C, sandbox        F5 F6  London scenes: Kensington, Knightsbridge        [ ]  speed down / up
  M  presentation overlay / diagnostic panels         V  London scenes: OpenStreetMap centrelines (cyan) over the modelled lanes (amber)
  1 2 3 4  cameras: chase, overhead, driver, orbit (right-drag rotate, wheel zoom, middle-drag pan, D detach)
  G  GNSS outage on/off     B  GNSS offset 5 m on/off     K  kerb-scanner blackout on/off
  C  close the road ahead   O  reopen it                  S  save the session    P  replay the last saved session
  L  legend   T  truth trail   E  estimate   Y  scan returns from the estimate   Z  zones   Q  quality
  F12  screenshot           F9  record video (lock-step, real time)            Tab  hide panels   H  help   Esc  quit
Every command takes effect at a recorded simulation tick; a saved session replays headlessly:
  road_drive session-replay <session.json>"""


class Hud:
    def __init__(self, base, actions):
        self.base = base
        self.actions = actions
        self.visible = True
        self.banner = OnscreenText(parent=base.a2dTopLeft, pos=(0.04, -0.075), scale=0.058, align=TextNode.ALeft, fg=LIVE, shadow=(0, 0, 0, 1),
                                   mayChange=True, text="")
        self.title = OnscreenText(parent=base.a2dTopLeft, pos=(0.04, -0.13), scale=0.034, align=TextNode.ALeft, fg=TEXT, shadow=(0, 0, 0, 1),
                                  mayChange=True, text="")
        self.disclaimer = OnscreenText(parent=base.a2dBottomRight, pos=(-0.03, 0.025), scale=0.026, align=TextNode.ARight, fg=MUTED,
                                       shadow=(0, 0, 0, 1), mayChange=True, text=DISCLAIMER)
        self._disclaimer = DISCLAIMER
        self._panel_h = 0.40
        self._caption_h = 0.24
        self.panel = DirectFrame(parent=base.a2dTopLeft, frameColor=PANEL, frameSize=(0.0, 1.22, -0.40, 0.0), pos=(0.02, 0, -0.16))
        self.labels = OnscreenText(parent=self.panel, pos=(0.03, -0.045), scale=READOUT_SCALE, align=TextNode.ALeft, fg=MUTED, mayChange=True,
                                   text="")
        self.values = OnscreenText(parent=self.panel, pos=(0.28, -0.045), scale=READOUT_SCALE, align=TextNode.ALeft, fg=TEXT, mayChange=True,
                                   text="")
        self.events_frame = DirectFrame(parent=base.a2dBottomLeft, frameColor=PANEL, frameSize=(0.0, 1.22, 0.0, 0.36), pos=(0.02, 0, 0.06))
        self.events = OnscreenText(parent=self.events_frame, pos=(0.03, 0.32), scale=0.027, align=TextNode.ALeft, fg=TEXT, mayChange=True, text="")
        self.caption_frame = DirectFrame(parent=base.a2dBottomCenter, frameColor=PANEL, frameSize=(-0.55, 0.55, 0.0, 0.24), pos=(0.30, 0, 0.06))
        self.caption = OnscreenText(parent=self.caption_frame, pos=(-0.52, 0.19), scale=0.029, align=TextNode.ALeft, fg=TEXT, wordwrap=36,
                                    mayChange=True, text="")
        self.controls = DirectFrame(parent=base.a2dTopRight, frameColor=PANEL, frameSize=(-0.74, 0.0, -1.70, 0.0), pos=(-0.02, 0, -0.02))
        self.buttons = {}
        self.cycle_text = {}
        self._build_controls()
        self.legend = DirectFrame(parent=base.aspect2d, frameColor=(0.03, 0.04, 0.05, 0.9), frameSize=(-1.05, 1.05, -0.36, 0.36), pos=(0, 0, 0.1))
        for i, (col, text) in enumerate(LEGEND):
            OnscreenText(parent=self.legend, pos=(-1.0, 0.30 - i * 0.066), scale=0.032, align=TextNode.ALeft, fg=col, text="--  " + text)
        self.legend.hide()
        self.help = DirectFrame(parent=base.aspect2d, frameColor=(0.03, 0.04, 0.05, 0.92), frameSize=(-1.25, 1.25, -0.38, 0.38), pos=(0, 0, 0.1))
        OnscreenText(parent=self.help, pos=(-1.2, 0.31), scale=0.032, align=TextNode.ALeft, fg=TEXT, text=HELP)
        self.help.hide()

    def _button(self, key, text, x, y, w, command, toggle=False):
        b = DirectButton(parent=self.controls, text=text, text_scale=0.028, text_fg=TEXT, frameColor=BUTTON, relief=DGG.FLAT,
                         frameSize=(-w / 2, w / 2, -0.028, 0.042), pos=(x, 0, y), command=command, pressEffect=1)
        self.buttons[key] = b
        return b

    def _build_controls(self):
        a = self.actions
        y = -0.06
        OnscreenText(parent=self.controls, pos=(-0.71, y + 0.0), scale=0.03, align=TextNode.ALeft, fg=MUTED, text="Simulation")
        y -= 0.07
        self._button("pause", "Pause", -0.60, y, 0.21, a["pause"])
        self._button("step", "Step 0.1 s", -0.375, y, 0.21, a["step"])
        self._button("reset", "Restart", -0.15, y, 0.21, a["reset"])
        y -= 0.075
        for i, (label, v) in enumerate((("0.5x", 0.5), ("1x", 1.0), ("2x", 2.0), ("4x", 4.0))):
            self._button("speed%g" % v, label, -0.64 + i * 0.165, y, 0.15, lambda v=v: a["speed"](v))
        y -= 0.085
        OnscreenText(parent=self.controls, pos=(-0.71, y + 0.02), scale=0.03, align=TextNode.ALeft, fg=MUTED, text="Demonstrations")
        y -= 0.05
        for i, k in enumerate(("A", "B", "C", "S")):
            self._button("demo" + k, "Sandbox" if k == "S" else k, -0.64 + i * 0.165, y, 0.15, lambda k=k: a["demo"](k))
        y -= 0.07
        self._button("demoK", "Kensington", -0.545, y, 0.32, lambda: a["demo"]("K"))
        self._button("demoN", "Knightsbridge", -0.195, y, 0.32, lambda: a["demo"]("N"))
        y -= 0.085
        OnscreenText(parent=self.controls, pos=(-0.71, y + 0.02), scale=0.03, align=TextNode.ALeft, fg=MUTED, text="Configuration")
        for field in ("scenario", "condition", "estimator", "seed", "mode", "rejoin"):
            y -= 0.062
            self._button("prev_" + field, "<", -0.68, y, 0.06, lambda f=field: a["cycle"](f, -1))
            self.cycle_text[field] = OnscreenText(parent=self.controls, pos=(-0.38, y), scale=0.028, align=TextNode.ACenter, fg=TEXT, mayChange=True,
                                                  text=field)
            self._button("next_" + field, ">", -0.08, y, 0.06, lambda f=field: a["cycle"](f, 1))
        y -= 0.07
        self._button("apply", "Start this configuration", -0.37, y, 0.66, a["apply"])
        y -= 0.085
        OnscreenText(parent=self.controls, pos=(-0.71, y + 0.02), scale=0.03, align=TextNode.ALeft, fg=MUTED, text="Inject (at the next tick)")
        y -= 0.055
        self._button("gnss_outage", "GNSS outage", -0.545, y, 0.32, a["gnss_outage"])
        self._button("gnss_bias", "GNSS offset 5 m", -0.195, y, 0.32, a["gnss_bias"])
        y -= 0.07
        self._button("kerb_blackout", "Kerb blackout", -0.545, y, 0.32, a["kerb_blackout"])
        self._button("close", "Close road ahead", -0.195, y, 0.32, a["close"])
        y -= 0.07
        self._button("reopen", "Reopen road", -0.545, y, 0.32, a["reopen"])
        self._button("save", "Save session", -0.195, y, 0.32, a["save"])
        y -= 0.07
        self._button("replay", "Replay saved session", -0.37, y, 0.66, a["replay"])
        y -= 0.085
        OnscreenText(parent=self.controls, pos=(-0.71, y + 0.02), scale=0.03, align=TextNode.ALeft, fg=MUTED, text="Camera and overlays")
        y -= 0.055
        for i, (k, label) in enumerate((("chase", "Chase"), ("overhead", "Overhead"), ("driver", "Driver"), ("orbit", "Orbit"))):
            self._button("cam_" + k, label, -0.64 + i * 0.165, y, 0.15, lambda k=k: a["camera"](k))
        y -= 0.07
        for i, (k, label) in enumerate((("legend", "Legend"), ("help", "Help"), ("quality", "Quality"), ("record", "Record video"))):
            self._button(k, label, -0.64 + i * 0.165, y, 0.15, a[k])

    def set_disclaimer(self, district, vehicle_id="ipace_proxy_v1"):
        """The footer says what the streets are (synthetic, or a source-informed reconstruction of real ones) and what
        the vehicle is."""
        text = DISCLAIMER_DISTRICT if district else DISCLAIMER
        if vehicle_id == "cybercab_proxy_v1":
            text = ("Independent simulation  |  source-informed reconstruction, details approximated; map data © OpenStreetMap contributors (ODbL)  |  "
                    "Cybercab-inspired proxy, not a Tesla vehicle or Tesla software")
        if text != self._disclaimer:
            self._disclaimer = text
            self.disclaimer.setText(text)

    def fit_panel(self, rows):
        """Size the readout panel to its rows, from the font's own line height, so that no row falls outside it."""
        h = round(0.045 + (rows - 1) * self.labels.textNode.getLineHeight() * READOUT_SCALE + 0.035, 3)
        if h != self._panel_h:
            self._panel_h = h
            self.panel["frameSize"] = (0.0, 1.22, -h, 0.0)

    def fit_caption(self):
        """Grow the caption box (upwards from its foot) to the caption's rows, so that no line runs out of it."""
        tn = self.caption.textNode
        rows = tn.getNumRows() if self.caption["text"] else 0
        h = round(max(0.24, 0.05 + rows * tn.getLineHeight() * 0.029 + 0.015), 3)
        if h != self._caption_h:
            self._caption_h = h
            self.caption_frame["frameSize"] = (-0.55, 0.55, 0.0, h)
            self.caption.setTextPos(-0.52, h - 0.05)

    def set_toggle(self, key, on, text=None):
        b = self.buttons.get(key)
        if b is None:
            return
        b["frameColor"] = BUTTON_ON if on else BUTTON
        if text:
            b["text"] = text

    def toggle_visible(self):
        self.visible = not self.visible
        for w in (self.panel, self.events_frame, self.caption_frame, self.controls):
            (w.show if self.visible else w.hide)()
