"""What the viewer knows about the running episode, built only from backend messages. Pure Python.

The renderer reads this; nothing here is sent back except operator commands. Truth fields are kept for
drawing and diagnostics only.
"""
import collections
import math


def applied_detail(msg):
    """One line for an `applied` message: its tick and the resolved command."""
    c = msg.get("command", {})
    parts = ["tick %d" % msg["tick"]]
    if "on" in c:
        parts.append("on" if c["on"] else "off")
    if "dx" in c:
        dx, dy = (round(c[k], 2) + 0.0 for k in ("dx", "dy"))  # + 0.0: no "-0.00"
        parts.append("offset %+.2f, %+.2f m" % (dx, dy) if dx or dy else "off")
    if "lane" in c:
        parts.append("barrier on %s at %.1f m" % (c["lane"], c.get("barrier_s", 0.0)))
    if msg.get("late"):
        parts.append("late")
    return "; ".join(parts)


class SimState:
    def __init__(self, trail_seconds=120.0):
        self.trail_ticks = int(trail_seconds * 100)
        self.hello = None
        self.reset()

    def reset(self):
        self.session = None      # the "session" message: mode (live/replay), config, labels
        self.scene = None        # the "scene" message: map, vehicle, sensors, zones, obstacles
        self.tick = None         # the latest "tick" message
        self.status = {}
        self.end = None
        self.route = None
        self.path = None
        self.closure = None
        self.truth_trail = collections.deque(maxlen=4000)
        self.est_trail = collections.deque(maxlen=4000)
        self.gnss = collections.deque(maxlen=200)
        self.scans = collections.deque(maxlen=6)
        self.events = collections.deque(maxlen=200)
        self.markers = collections.deque(maxlen=40)  # correction events to draw where they happened
        self.applied = collections.deque(maxlen=50)
        self.errors = collections.deque(maxlen=20)
        self.saved = None
        self.scene_version = 0   # bumps when a new scene arrives (rebuild the world)
        self.session_version = 0
        self.ticks_seen = 0
        self.max_crossing = 0.0
        self.last_trail_tick = -1

    @property
    def mode(self):
        return (self.session or {}).get("mode", "idle")

    @property
    def config(self):
        return (self.session or {}).get("config", {})

    def apply(self, msg):
        kind = msg["type"]
        if kind == "hello":
            self.hello = msg
        elif kind == "session":
            hello = self.hello
            self.reset()
            self.hello = hello
            self.session = msg
            self.session_version += 1
        elif kind == "scene":
            self.scene = msg
            self.scene_version += 1
        elif kind == "tick":
            self.tick = msg
            self.ticks_seen += 1
            self.max_crossing = max(self.max_crossing, msg.get("crossing", 0.0))
            t = msg["truth"]
            if msg["tick"] - self.last_trail_tick >= 10:  # trails at 10 Hz
                self.last_trail_tick = msg["tick"]
                self.truth_trail.append((t["x"], t["y"]))
                est = msg.get("est")
                if est and est.get("valid"):
                    self.est_trail.append((est["x"], est["y"]))
        elif kind == "gnss":
            self.gnss.append(msg)
        elif kind == "scan":
            self.scans.append(msg)
        elif kind == "event":
            self.events.append(msg)
            if msg["event"] in ("POSE_CORRECTION", "KERB_CORRECTION", "KERB_RELOCALISATION", "MONITOR_TRIGGER", "FALLBACK_STOP",
                                "CLOSURE_NOTIFIED", "CLOSURE_LIFTED", "NO_CLEAR_CONTINUATION", "LOCALISATION_UNRELIABLE"):
                pos = None
                if self.tick and self.tick["stack"]["started"]:  # not the alignment before the mission starts
                    tr = self.tick["truth"]
                    pos = (tr["x"], tr["y"])
                self.markers.append(dict(msg, pos=pos))
        elif kind == "route":
            self.route = msg
        elif kind == "path":
            self.path = msg
        elif kind == "closure":
            self.closure = msg
        elif kind == "status":
            self.status = msg
        elif kind == "applied":
            self.applied.append(msg)
            # The events panel lists each operator command at the tick it took effect, beside the episode's events.
            self.events.append({"t": msg["t"], "source": "REPLAY" if msg.get("replay") else "OPERATOR",
                                "event": str(msg.get("command", {}).get("cmd", "?")).upper(), "detail": applied_detail(msg)})
        elif kind == "end":
            self.end = msg
        elif kind == "saved":
            self.saved = msg
        elif kind == "error":
            self.errors.append(msg)

    # ---- Derived readouts -------------------------------------------------------------------------
    def readout(self):
        """The panel's lines, as (label, value) pairs; '-' where unknown."""
        tk = self.tick
        cfg = self.config
        st = self.status
        rows = []
        rows.append(("scenario", "%s  %s" % (cfg.get("scenario", "-"), cfg.get("condition", "-"))))
        rows.append(("pose source", "%s  %s  re-join %s  seed %s" % (
            cfg.get("mode", "-"), cfg.get("estimator", "-") if cfg.get("mode") == "estimated" else "(truth)", cfg.get("rejoin", "-"), cfg.get("seed", "-"))))
        if tk is None:
            return rows
        tr = tk["truth"]
        rows.append(("time", "t %6.2f s   tick %d   RTF %.2f%s" % (tk["t"], tk["tick"], st.get("rtf", 0.0) or 0.0,
                                                                 "  (lagging)" if st.get("lagging") else "")))
        rows.append(("speed", "%5.2f m/s (%4.1f mph)   steer %+5.1f deg   accel %+4.1f" % (
            tr["v"], tr["v"] / 0.44704, math.degrees(tr["steer"]), tr["accel"])))
        stack = tk["stack"]
        rows.append(("stack", "%s   %s   shift %+.2f   re-join %+.2f" % (stack["mode"], stack["element"], stack["shift"], stack["rejoin"])))
        sens = tk.get("sensors") or {}
        est = tk.get("est")
        gnss = "available" if sens.get("gnss_available", True) else "NO FIXES"
        if sens.get("injected", {}).get("gnss_outage"):
            gnss = "NO FIXES (injected outage)"
        bias = sens.get("injected", {}).get("gnss_bias", [0, 0])
        fault = sens.get("gnss_fault_m", 0.0)
        integrity = est["gnss"] if est else "-"
        rows.append(("GNSS", "%s   integrity %s   fault %.1f m%s" % (gnss, integrity, fault, "  (injected %.1f m)" % math.hypot(*bias) if any(bias) else "")))
        kerb = "scanning" if sens.get("kerb_scanning", True) else "BLACKOUT"
        if sens.get("injected", {}).get("kerb_blackout"):
            kerb = "BLACKOUT (injected)"
        if sens.get("kerb_map_error"):
            kerb += "   (map error zone)"
        rows.append(("kerb scanner", "%s   match %s   used %s rejected %s" % (
            kerb, est["kerb"] if est else "-", est["kerb_used"] if est else "-", est["kerb_rejected"] if est else "-")))
        if est:
            rows.append(("estimate", "%s   sigma %.2f m (along %.2f, across %.2f)" % (est["status"], est["sigma"], est["sigma_along"], est["sigma_across"])))
            rows.append(("corrections", "verified %d   unique %d   re-localisations %d" % (est["verified"], est["unique"], est["relocalisations"])))
        d = tk.get("diag")
        if d:
            rows.append(("error (truth)", "along %+5.2f m   lateral %+5.2f m   heading %+5.2f deg" % (d["along"], d["lateral"], d["heading_deg"])))
        rows.append(("clearance", "%+.2f m with mirrors   body over kerb %.2f m" % (tk["clearance"], tk["crossing"])))
        return rows
