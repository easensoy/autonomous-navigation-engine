"""The presentation overlay: a compact, readable HUD for showcase recordings (the full diagnostic panels stay one
key away, M). It shows the district and street, simulated time and speed, the estimator and its integrity
state, the localisation error against truth with the filter's own uncertainty, and a short list of sensor
and correction events. Every value is the backend's report for the tick on screen: nothing is smoothed.

Events are the backend's correction events and the transitions it reports in its tick stream (a GNSS outage
beginning and ending, the integrity monitor excluding or readmitting GNSS, the filter dead-reckoning, kerb
scans rejected or lost), each stamped with the tick at which it was reported.
"""
import math

PANEL = (0.03, 0.04, 0.05, 0.62)
TEXT = (0.95, 0.96, 0.97, 1.0)
MUTED = (0.70, 0.74, 0.79, 1.0)
GOOD = (0.40, 0.92, 0.50, 1.0)
WARN = (1.00, 0.76, 0.25, 1.0)
BAD = (1.00, 0.42, 0.38, 1.0)
LIVE = (0.30, 0.95, 0.40, 1.0)
REPLAY = (1.00, 0.62, 0.15, 1.0)
STATUS_COLOUR = {"OK": GOOD, "INIT": MUTED, "DEAD_RECKONING": WARN, "DEGRADED": WARN, "UNRELIABLE": BAD, "LOST": BAD}
STATUS_TEXT = {"OK": "integrity OK", "INIT": "initialising", "DEAD_RECKONING": "dead reckoning", "DEGRADED": "degraded", "UNRELIABLE": "unreliable",
               "LOST": "lost"}
GNSS_TEXT = {"TRUSTED": "GNSS trusted", "OUTAGE": "GNSS outage", "FAULTED": "GNSS excluded (fault)", "SUSPECT": "GNSS suspect",
             "INCONSISTENT": "GNSS inconsistent with the filter", "-": "GNSS -"}
KERB_TEXT = {"MATCHED": "kerb scans matched", "REJECTED": "kerb scans rejected", "NONE": "no kerb scans", "-": "kerb -"}


def vehicle_text(scene):
    """How the footer names the simulated vehicle, and what it does not claim."""
    vid = ((scene or {}).get("vehicle") or {}).get("id", "ipace_proxy_v1")
    if vid == "cybercab_proxy_v1":
        return "Independent simulation  |  Cybercab-inspired proxy, not a Tesla vehicle or Tesla software  |  sensors: GNSS, IMU, wheel, steering, planar kerb scanner"
    return "Independent research simulation  |  simulated I-PACE-sized proxy (not a Jaguar)  |  not Waymo software or data  |  planar kerb scanner only; the roof rig is decorative"


def compass(yaw):
    names = ["eastbound", "north-eastbound", "northbound", "north-westbound", "westbound", "south-westbound", "southbound", "south-eastbound"]
    return names[int(round((yaw % (2 * math.pi)) / (math.pi / 4))) % 8]


class Streets:
    """Element name -> the real street it lies on, from the map message (district networks name their streets)."""

    def __init__(self, m):
        info = m.get("street_info") or []
        self.lanes = {}
        for ln in m["lanes"]:
            # Each lane runs beside one street's centreline: the nearest one to its middle.
            mx, my = 0.5 * (ln["x0"] + ln["x1"]), 0.5 * (ln["y0"] + ln["y1"])
            best = None
            for (a, b, ax, ay, bx, by), si in zip(m["streets"], info):
                vx, vy = bx - ax, by - ay
                l2 = vx * vx + vy * vy
                t = max(0.0, min(1.0, ((mx - ax) * vx + (my - ay) * vy) / l2)) if l2 else 0.0
                d = math.hypot(mx - ax - t * vx, my - ay - t * vy)
                if best is None or d < best[0]:
                    best = (d, si["name"])
            self.lanes[ln["name"]] = best[1] if best else ""

    def street(self, element):
        if not element:
            return ""
        if ":" in element:  # a turning movement at a junction: from one street into another
            j, rest = element.split(":", 1)
            rest = rest.split("(")[0]
            a, b = rest.split(">")
            s0 = self.lanes.get(a + ">" + j, "")
            s1 = self.lanes.get(j + ">" + b, "")
            return s0 if s0 == s1 else ("%s into %s" % (s0, s1) if s0 and s1 else s0 or s1)
        return self.lanes.get(element.split("/")[0], "")


class PresentEvents:
    """Sensor and correction events for the overlay, from the tick stream's reported states."""

    WATCH = {"KERB_RELOCALISATION": "kerb re-localisation", "POSE_CORRECTION": "pose correction re-joined", "FALLBACK_STOP": "fallback stop",
             "LOCALISATION_UNRELIABLE": "localisation unreliable", "CLOSURE_NOTIFIED": "closure notified", "GOAL_REACHED": "goal reached"}

    def __init__(self):
        self.items = []
        self.last = {}
        self.seen = 0

    def update(self, state):
        tk = state.tick
        if tk is None:
            return
        started = tk["stack"]["started"]
        est = tk.get("est") or {}
        for key, val in (("gnss", est.get("gnss")), ("status", est.get("status")), ("kerb", est.get("kerb"))):
            if val is None:
                continue
            prev = self.last.get(key)
            self.last[key] = val
            if prev is None or prev == val or not started or prev in ("INIT", "-", "NONE"):
                continue
            text = self._transition(key, prev, val)
            if text:
                self.items.append((tk["t"], text[0], text[1]))
        evs = list(state.events)
        for e in evs[self.seen:]:
            label = self.WATCH.get(e["event"])
            if label and (started or e["event"] == "GOAL_REACHED"):
                jump = ""
                for part in e.get("detail", "").split(";"):
                    if part.startswith("jump="):
                        jump = " %.2f m" % abs(float(part[5:]))
                self.items.append((e["t"], label + jump, WARN if "fallback" in label or "unreliable" in label else GOOD))
        self.seen = len(evs)

    @staticmethod
    def _transition(key, prev, val):
        if key == "gnss":
            if val == "OUTAGE":
                return ("GNSS outage: no fixes", WARN)
            if val == "FAULTED":
                return ("GNSS fault detected: fixes excluded", WARN)
            if val == "TRUSTED":
                return ("GNSS restored and trusted", GOOD)
            return ("GNSS " + val.lower(), WARN)
        if key == "status":
            if val == "DEAD_RECKONING":
                return ("dead reckoning on IMU, wheels and kerb scans", WARN)
            if val == "OK" and prev in ("DEAD_RECKONING", "DEGRADED", "UNRELIABLE"):
                return ("integrity OK again", GOOD)
            return ("estimate " + val.lower().replace("_", " "), BAD if val in ("UNRELIABLE", "LOST") else WARN)
        if key == "kerb":
            if val == "REJECTED":
                return ("kerb scans rejected by the filter", WARN)
            if val == "MATCHED" and prev == "REJECTED":
                return ("kerb scans matched again", GOOD)
        return None


class PresentHud:
    def __init__(self, base, font=None, bold=None):
        from direct.gui.DirectGui import DirectFrame
        from direct.gui.OnscreenText import OnscreenText
        from panda3d.core import TextNode
        self.base = base
        self.font, self.bold = font, bold
        self.root = base.aspect2d.attachNewNode("present_hud")
        kw = dict(parent=base.a2dTopLeft, mayChange=True, align=TextNode.ALeft, shadow=(0, 0, 0, 0.8), shadowOffset=(0.04, 0.04))
        self.top = DirectFrame(parent=base.a2dTopLeft, frameColor=PANEL, frameSize=(0.0, 1.10, -0.235, 0.0), pos=(0.035, 0, -0.035))
        self.district = OnscreenText(text="", pos=(0.075, -0.105), scale=0.062, fg=TEXT, font=bold or font, **kw)
        self.street = OnscreenText(text="", pos=(0.075, -0.168), scale=0.040, fg=TEXT, font=font, **kw)
        self.subtitle = OnscreenText(text="", pos=(0.075, -0.222), scale=0.026, fg=MUTED, font=font, **kw)
        kr = dict(parent=base.a2dTopRight, mayChange=True, align=TextNode.ARight, shadow=(0, 0, 0, 0.8), shadowOffset=(0.04, 0.04))
        self.tr = DirectFrame(parent=base.a2dTopRight, frameColor=PANEL, frameSize=(-0.62, 0.0, -0.235, 0.0), pos=(-0.035, 0, -0.035))
        self.mode = OnscreenText(text="", pos=(-0.075, -0.098), scale=0.036, fg=LIVE, font=bold or font, **kr)
        self.clock = OnscreenText(text="", pos=(-0.075, -0.160), scale=0.042, fg=TEXT, font=font, **kr)
        self.speed = OnscreenText(text="", pos=(-0.075, -0.212), scale=0.030, fg=MUTED, font=font, **kr)
        kb = dict(parent=base.a2dBottomLeft, mayChange=True, align=TextNode.ALeft, shadow=(0, 0, 0, 0.8), shadowOffset=(0.04, 0.04))
        self.bl = DirectFrame(parent=base.a2dBottomLeft, frameColor=PANEL, frameSize=(0.0, 1.10, 0.0, 0.265), pos=(0.035, 0, 0.085))
        self.est = OnscreenText(text="", pos=(0.075, 0.29), scale=0.036, fg=TEXT, font=bold or font, **kb)
        self.sens = OnscreenText(text="", pos=(0.075, 0.236), scale=0.029, fg=MUTED, font=font, **kb)
        self.err = OnscreenText(text="", pos=(0.075, 0.183), scale=0.031, fg=TEXT, font=font, **kb)
        self.sig = OnscreenText(text="", pos=(0.075, 0.131), scale=0.027, fg=MUTED, font=font, **kb)
        kbr = dict(parent=base.a2dBottomRight, mayChange=True, align=TextNode.ALeft, shadow=(0, 0, 0, 0.8), shadowOffset=(0.04, 0.04))
        self.br = DirectFrame(parent=base.a2dBottomRight, frameColor=PANEL, frameSize=(-1.04, 0.0, 0.0, 0.265), pos=(-0.035, 0, 0.085))
        self.ev_title = OnscreenText(text="Sensor and correction events", pos=(-1.0, 0.29), scale=0.027, fg=MUTED, font=font, **kbr)
        self.ev = [OnscreenText(text="", pos=(-1.0, 0.240 - 0.050 * i), scale=0.029, fg=TEXT, font=font, **kbr) for i in range(4)]
        self.foot = OnscreenText(parent=base.a2dBottomCenter, text="", pos=(0.0, 0.032), scale=0.024, fg=MUTED, font=font, align=TextNode.ACenter,
                                 mayChange=True, shadow=(0, 0, 0, 0.8))
        self.card = DirectFrame(parent=base.aspect2d, frameColor=(0.02, 0.03, 0.04, 0.80), frameSize=(-1.30, 1.30, -0.34, 0.34), pos=(0, 0, 0.05))
        self.card_title = OnscreenText(parent=self.card, text="", pos=(0, 0.13), scale=0.090, fg=TEXT, font=bold or font, align=TextNode.ACenter,
                                       mayChange=True)
        self.card_sub = OnscreenText(parent=self.card, text="", pos=(0, 0.035), scale=0.045, fg=TEXT, font=font, align=TextNode.ACenter, mayChange=True)
        self.card_note = OnscreenText(parent=self.card, text="", pos=(0, -0.075), scale=0.030, fg=MUTED, font=font, align=TextNode.ACenter,
                                      mayChange=True, wordwrap=70)
        self.card.hide()
        self.widgets = [self.top, self.district, self.street, self.subtitle, self.tr, self.mode, self.clock, self.speed, self.bl, self.est, self.sens,
                        self.err, self.sig, self.br, self.ev_title, self.foot] + self.ev
        self.visible = True
        self.events = PresentEvents()
        self.streets = None
        self.time_text = ""

    def show(self, on):
        self.visible = on
        for w in self.widgets:
            (w.show if on else w.hide)()
        if not on:
            self.card.hide()

    def show_card(self, title, sub, note):
        self.card_title.setText(title)
        self.card_sub.setText(sub)
        self.card_note.setText(note)
        self.card.show()

    def hide_card(self):
        self.card.hide()

    def update(self, state, mode_label, recording=False):
        """The panels for the tick on screen. Returns the clock text shown (logged per recorded frame)."""
        tk = state.tick
        scene = state.scene or {}
        m = scene.get("map") or {}
        if self.streets is None and m:
            self.streets = Streets(m)
        dist = m.get("district") or {}
        cfg = state.config
        place = dist.get("district") or cfg.get("scenario", "")
        if dist.get("city"):
            place = "%s, %s" % (place, dist["city"])
        self.district.setText(place.upper())
        self.subtitle.setText(("Source-informed reconstruction, details approximated; " + ("right-hand traffic; " if dist.get("traffic") == "right" else "") +
                               "map data © OpenStreetMap contributors (ODbL)") if dist else "Synthetic streets")
        live = mode_label == "LIVE"
        self.mode.setText(mode_label + ("  |  1080p50 lock-step" if recording else ""))
        self.mode["fg"] = LIVE if live else REPLAY
        self.foot.setText(vehicle_text(scene))
        if tk is None:
            self.time_text = ""
            return self.time_text
        self.events.update(state)
        tr = tk["truth"]
        st = tk["stack"]
        name = self.streets.street(st.get("element")) if self.streets else ""
        self.street.setText("%s  -  %s" % (name, compass(tr["yaw"])) if name else cfg.get("scenario", ""))
        self.time_text = "t = %.2f s" % tk["t"]
        self.clock.setText(self.time_text)
        self.speed.setText("%.1f mph  (%.1f m/s)   tick %d" % (tr["v"] / 0.44704, tr["v"], tk["tick"]))
        est = tk.get("est")
        if est and est.get("valid"):
            status = est["status"]
            self.est.setText("%s   %s" % (cfg.get("estimator", "-"), STATUS_TEXT.get(status, status.lower())))
            self.est["fg"] = STATUS_COLOUR.get(status, TEXT)
            self.sens.setText("%s   |   %s" % (GNSS_TEXT.get(est["gnss"], est["gnss"]), KERB_TEXT.get(est["kerb"], est["kerb"].lower())))
            d = tk.get("diag") or {}
            self.err.setText("error against truth  %.2f m   (along %+.2f, lateral %+.2f)" % (d.get("position", 0.0), d.get("along", 0.0), d.get("lateral", 0.0)))
            self.sig.setText("filter's own 1-sigma  %.2f m   (along %.2f, across %.2f);  ring: 95%%" % (est["sigma"], est["sigma_along"], est["sigma_across"]))
        else:
            self.est.setText("driving on simulator truth" if cfg.get("mode") == "ground_truth" else "estimate initialising")
            self.est["fg"] = MUTED
            self.sens.setText("")
            self.err.setText("")
            self.sig.setText("")
        items = self.events.items[-4:]
        for i, node in enumerate(self.ev):
            if i < len(items):
                t, text, col = items[-1 - i]
                node.setText("%6.1f s   %s" % (t, text))
                node["fg"] = col
            else:
                node.setText("")
        return self.time_text
