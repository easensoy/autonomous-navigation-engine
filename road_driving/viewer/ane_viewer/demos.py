"""Demonstrations and the configuration choices the viewer offers. Pure Python.

Each demonstration is an ordinary live configuration; its captions are triggered by what the backend
reports (events, sensor states, the truth diagnostics), and quote the live values. Nothing is scripted
against time, smoothed or moved: the demonstration shows the episode exactly as the simulation runs it.
"""

ROUTES = ["KEN-A", "KNB-A", "R7-A", "R7-B", "R7-C", "R6-A", "R6-B", "R6-C", "R5-A", "R5-B", "R5-C", "R4-A", "R4-B", "R4-C",
          "R3-A", "R3-B", "R3-C", "R2-A", "R2-B", "R2-C", "R1-A", "R1-B", "R1-C",
          "R7-B_late_closure", "R6-B_late_closure", "R3-B_late_closure"]
CONDITIONS = {
    "KEN": ["nominal_ldn", "gnss_outage_ldn", "multipath_ldn"],
    "KNB": ["nominal_ldn", "gnss_outage_ldn", "multipath_ldn"],
    "R7": ["nominal_r7", "gnss_outage_r7", "multipath_r7", "kerb_return_r7", "imu_fault_r7", "kerb_faults_r7", "combined_r7"],
    "R6": ["nominal_r6", "gnss_outage_r6", "multipath_r6", "gnss_ramp_r6", "kerb_return_r6", "imu_fault_r6", "combined_r6"],
    "R5": ["nominal_r5", "gnss_outage_r5", "multipath_r5", "gnss_ramp_r5", "imu_fault_r5", "kerb_faults_r5", "combined_r5"],
    "R4": ["nominal_r4", "gnss_outage_r4", "multipath_r4", "gnss_ramp_r4", "imu_fault_r4", "kerb_faults_r4", "combined_r4"],
    "R3": ["nominal_r3", "gnss_outage_r3", "multipath_r3", "gnss_jumps_r3", "gnss_ramp_r3", "kerb_clutter_r3", "combined_r3"],
    "R2": ["nominal_h", "gnss_outage_h", "multipath_h", "wheel_slip_h", "imu_bias_h", "delay_drop_h", "combined_h"],
    "R1": ["nominal", "gnss_outage", "multipath", "wheel_slip", "imu_bias", "delay_drop"],
}
ESTIMATORS = ["ekf_kerb2", "ekf_kerb", "ekf_int2", "ekf_int", "ekf", "gnss_cv", "dr"]
MODES = ["estimated", "ground_truth"]
REJOINS = ["unique", "verified", "on", "off"]


def network_of(scenario):
    head = scenario.split("-")[0]
    return head if head in CONDITIONS else "R1"


def _fmt(v, fallback="-"):
    return fallback if v is None else "%.2f" % v


class Stage:
    """A caption: shown when `trigger` first holds. Its text is fixed then, unless `live`: a live caption keeps
    quoting current values while it is the newest, and is fixed when the next one appears."""

    def __init__(self, trigger, text, live=False):
        self.trigger = trigger
        self.text = text
        self.live = live


class Demo:
    def __init__(self, key, title, config, intro, stages=()):
        self.key = key
        self.title = title
        self.config = config
        self.intro = intro
        self.stages = list(stages)


def _diag(state, name):
    d = (state.tick or {}).get("diag") or {}
    return d.get(name)


def _sensor(state, name, default=None):
    return ((state.tick or {}).get("sensors") or {}).get(name, default)


def _events(state, kind, after=0.0):
    return [e for e in state.events if e["event"] == kind and e["t"] > after]


def _jump(event):
    for part in event.get("detail", "").split(";"):
        if part.startswith("jump="):
            return abs(float(part[5:]))
    return None


DEMOS = [
    Demo("A", "A. Nominal estimated-state driving",
         {"scenario": "R7-A", "condition": "nominal_r7", "mode": "estimated", "estimator": "ekf_kerb2", "seed": 1, "rejoin": "unique"},
         "The stack drives on ekf_kerb2's estimate (GNSS, IMU, wheel speed, steering and the planar kerb scanner). "
         "Solid car: simulated truth. Teal outline: the estimate it drives on. Green kerb returns: scans the filter used.",
         [Stage(lambda s: s.tick and s.tick["t"] > 15.0,
                lambda s: "Lateral error now %s m against truth; the filter reports %s m (1 sigma)." % (
                    _fmt(_diag(s, "lateral")), _fmt((s.tick.get("est") or {}).get("sigma_across"))), live=True)]),
    Demo("B", "B. Slow GNSS excursion, kerb-aided recovery",
         {"scenario": "R3-B", "condition": "gnss_ramp_r3", "mode": "estimated", "estimator": "ekf_kerb2", "seed": 5, "rejoin": "unique"},
         "R3-B, slow GNSS excursion, seed 5 (development data): the fixes drift sideways at 0.25 m/s with no jump. "
         "On a straight, kerbs cannot see along-road error, so the filter follows GNSS along the road.",
         [Stage(lambda s: (s.tick or {}).get("est") and s.tick["est"]["kerb"] == "REJECTED" and s.tick["t"] > 20.0,
                lambda s: "At the turn the kerb scans stop fitting the estimate (rejected, red). Along-road error %s m." % _fmt(_diag(s, "along"))),
          Stage(lambda s: bool(_events(s, "KERB_RELOCALISATION", 5.0)),
                lambda s: "ekf_kerb2 searched a wider area and found the pose the scan fits uniquely: re-localised. "
                          "Error now %s m; the receiver must earn trust again." % _fmt(_diag(s, "position")))]),
    Demo("C", "C. R7-A kerb faults, seed 7: the remaining failure",
         {"scenario": "R7-A", "condition": "kerb_faults_r7", "mode": "estimated", "estimator": "ekf_kerb2", "seed": 7, "rejoin": "unique"},
         "The held-out R7 episode behind the sixth evaluation's one failure (X3), run live: R7-A, kerb faults, seed 7, "
         "ekf_kerb2, unique re-join. It is reproduced exactly, crossing included.",
         [Stage(lambda s: _sensor(s, "kerb_map_error"),
                lambda s: "A kerb section here is mapped 0.5 m wrong. Registrations move the estimate gradually; nothing is re-joined."),
          Stage(lambda s: _sensor(s, "kerb_scanning") is False and abs(_diag(s, "along") or 0.0) > 1.0,
                lambda s: "No kerb scans, while the GNSS error grows at 0.3 m/s. The filter follows GNSS and reports sigma %s m; "
                          "its true error along the road is %s m." % (_fmt((s.tick.get("est") or {}).get("sigma")), _fmt(_diag(s, "along"))), live=True),
          Stage(lambda s: bool(_events(s, "POSE_CORRECTION", 10.0)),
                lambda s: "Kerbs return: a unique re-localisation corrects %s m sideways and the stack re-joins it. The along-road error "
                          "(%s m) remains: kerbs on a straight cannot see it." % (_fmt(_jump(_events(s, "POSE_CORRECTION", 10.0)[0])), _fmt(_diag(s, "along")))),
          Stage(lambda s: (s.tick or {}).get("crossing", 0.0) > 0.0,
                lambda s: "Right turn at the tight J23: the along-road error has become lateral (%s m). The body is %s m over the kerb now, "
                          "%s m at most so far (truth)." % (_fmt(_diag(s, "lateral")), _fmt(s.tick["crossing"]), _fmt(s.max_crossing)), live=True),
          Stage(lambda s: len(_events(s, "POSE_CORRECTION", 10.0)) >= 2,
                lambda s: "A second re-localisation corrects %s m and the car completes. Without the re-join, the monitor's fallback stop had "
                          "held the car before the turn: the episode X3 counted." % _fmt(_jump(_events(s, "POSE_CORRECTION", 10.0)[1]))),
          Stage(lambda s: s.end is not None,
                lambda s: "%s at %.1f s; deepest crossing %s m. Sixth evaluation: road_driving/LOCALISATION.md." % (
                    s.end["outcome"], s.end["t"], _fmt(s.end["metrics"]["max_crossing"])))]),
    Demo("K", "Kensington: Kensington Gore and Queen's Gate",
         {"scenario": "KEN-A", "condition": "gnss_outage_ldn", "mode": "estimated", "estimator": "ekf_kerb2", "seed": 1, "rejoin": "unique"},
         "Kensington Gore westbound past the Royal Albert Hall, left into Queen's Gate, right into Elvaston Place: a source-informed "
         "reconstruction from OpenStreetMap. The stack drives on ekf_kerb2's estimate; GNSS drops out on Queen's Gate.",
         [Stage(lambda s: ((s.tick or {}).get("est") or {}).get("gnss") == "OUTAGE",
                lambda s: "GNSS outage: no fixes. The filter dead-reckons on the IMU and wheels and keeps registering kerb scans; "
                          "its error against truth is %s m (it reports %s m)." % (_fmt(_diag(s, "position")), _fmt((s.tick.get("est") or {}).get("sigma"))),
                live=True),
          Stage(lambda s: bool(_events(s, "KERB_RELOCALISATION", 5.0)),
                lambda s: "The kerb scan fits the map uniquely elsewhere: ekf_kerb2 re-localises. Error now %s m." % _fmt(_diag(s, "position"))),
          Stage(lambda s: s.end is not None,
                lambda s: "%s at %.1f s; position error RMSE %s m, deepest kerb crossing %s m." % (
                    s.end["outcome"], s.end["t"], _fmt(s.end["metrics"].get("est_rmse")), _fmt(s.end["metrics"]["max_crossing"])))]),
    Demo("N", "Knightsbridge: Brompton Road, Hans Road and Basil Street",
         {"scenario": "KNB-A", "condition": "multipath_ldn", "mode": "estimated", "estimator": "ekf_kerb2", "seed": 1, "rejoin": "unique"},
         "Brompton Road south-westbound past Harrods, left into Hans Road, left into Basil Street: a source-informed reconstruction "
         "from OpenStreetMap. On Brompton Road a multipath condition offsets the GNSS fixes by 6 m; it is placed there, not computed "
         "from the buildings.",
         [Stage(lambda s: ((s.tick or {}).get("est") or {}).get("gnss") == "FAULTED",
                lambda s: "Multipath: the fixes are 6 m off. The integrity monitor excludes GNSS; the estimate holds on the IMU, wheels and "
                          "kerb scans: %s m from truth." % _fmt(_diag(s, "position")), live=True),
          Stage(lambda s: ((s.tick or {}).get("est") or {}).get("gnss") == "TRUSTED" and s.tick["t"] > 30.0,
                lambda s: "The fixes agree with the estimate again and GNSS is readmitted. Error %s m." % _fmt(_diag(s, "position"))),
          Stage(lambda s: s.end is not None,
                lambda s: "%s at %.1f s; position error RMSE %s m, deepest kerb crossing %s m." % (
                    s.end["outcome"], s.end["t"], _fmt(s.end["metrics"].get("est_rmse")), _fmt(s.end["metrics"]["max_crossing"])))]),
    Demo("S", "Sandbox: inject faults yourself",
         {"scenario": "R7-A", "condition": "nominal_r7", "mode": "estimated", "estimator": "ekf_kerb2", "seed": 1, "rejoin": "unique"},
         "Inject a GNSS outage (G), a biased fix (B), a kerb-scanner blackout (K), or close the road ahead (C) and reopen it (O). "
         "Each takes effect at a recorded tick; save the session (R) to replay it."),
]


def by_key(key):
    return next(d for d in DEMOS if d.key == key)


class Narrator:
    """Advances a demonstration's captions as their triggers fire; holds the latest text."""

    def __init__(self, demo):
        self.demo = demo
        self.next = 0
        self.lines = [demo.intro] if demo else []
        self.live = None  # the newest stage: its text quotes current values until the next stage fires

    def update(self, state):
        if not self.demo:
            return False
        changed = False
        i = self.next
        while i < len(self.demo.stages):
            stage = self.demo.stages[i]
            try:
                fired = bool(stage.trigger(state))
            except (KeyError, TypeError, IndexError):
                fired = False
            if fired:  # a later stage may fire first; the earlier ones are then passed over
                self.lines.append(stage.text(state))
                self.live = stage if stage.live else None
                self.next = i + 1
                changed = True
            i += 1
        if self.live is not None and not changed:
            try:
                self.lines[-1] = self.live.text(state)
            except (KeyError, TypeError, IndexError):
                pass
        return changed

    def text(self, last=3):
        return "\n".join(self.lines[-last:])
