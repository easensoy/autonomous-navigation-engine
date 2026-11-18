"""Overlays drawn from backend messages: what localisation is doing, at the scale it reports.

- trails: true (white) and estimated (teal) rear-axle paths;
- the stack's planned reference ahead (amber), with its clearance shift and re-join, and its route (blue);
- raw GNSS fixes (violet), newest largest;
- kerb returns from the last scans: true end points (from the scanner's true pose), coloured by the filter's
  decision on that scan: used (green), rejected (red), no kerb filter (yellow); optional: the same returns
  placed from the estimated pose (cyan), which is where the filter believes they landed;
- the filter-reported 95% position ellipse (2.45 sigma of its own covariance) and heading spread;
- markers where corrections, re-localisations, monitor triggers and closure events happened;
- the closure barrier and works, from the backend's closure message.
"""
import math

from panda3d.core import NodePath, TextNode, TransparencyAttrib

from . import geometry as g
from .render_util import Builder, lines

TRUTH = (0.96, 0.97, 0.98, 1.0)
EST = (0.20, 0.85, 0.85, 1.0)
PATH = (1.00, 0.72, 0.15, 1.0)
ROUTE = (0.35, 0.55, 1.00, 0.6)
GNSS = (0.72, 0.45, 0.98, 1.0)
USED = (0.25, 0.95, 0.35, 1.0)
REJECTED = (1.00, 0.25, 0.25, 1.0)
NOFILTER = (1.00, 0.85, 0.20, 1.0)
PLACED = (0.30, 0.90, 1.00, 1.0)
ELLIPSE = (0.20, 0.95, 0.95, 1.0)
BARRIER_RED = (0.85, 0.08, 0.08, 1.0)
BARRIER_WHITE = (0.95, 0.95, 0.95, 1.0)
CONE = (1.00, 0.45, 0.05, 1.0)
CHI2_95_2D = 2.4477  # sqrt of the 95% chi-square quantile, 2 dof
MARKER_COLOURS = {
    "POSE_CORRECTION": (0.25, 0.95, 0.95, 1.0), "KERB_CORRECTION": (0.25, 0.95, 0.35, 1.0), "KERB_RELOCALISATION": (0.30, 0.90, 1.00, 1.0),
    "MONITOR_TRIGGER": (1.00, 0.30, 0.25, 1.0), "FALLBACK_STOP": (1.00, 0.15, 0.15, 1.0), "CLOSURE_NOTIFIED": (1.00, 0.60, 0.10, 1.0),
    "CLOSURE_LIFTED": (0.40, 0.90, 0.40, 1.0), "NO_CLEAR_CONTINUATION": (1.00, 0.85, 0.20, 1.0), "LOCALISATION_UNRELIABLE": (1.00, 0.50, 0.20, 1.0),
}


class Overlays:
    def __init__(self, parent):
        self.root = parent.attachNewNode("overlays")
        self.root.setLightOff()
        # Unlit and vertex-coloured: drawn without the scene's generated shaders, so a marker, label or
        # trail appearing for the first time does not stall a frame while a new shader is compiled.
        self.root.setShaderOff(1)
        self.layers = {}
        for name in ("truth_trail", "est_trail", "path", "route", "gnss", "scan", "placed", "ellipse", "markers", "closure", "footprints"):
            self.layers[name] = self.root.attachNewNode(name)
        self.show = {name: True for name in self.layers}
        self.labels = True  # the presentation overlay shows marker poles without their text
        self.compact = False  # the presentation overlay: the last 20 GNSS fixes, drawn smaller
        self.show["placed"] = False
        self.layers["placed"].hide()
        self._nodes = {}
        self._last = {}

    def toggle(self, name):
        self.show[name] = not self.show[name]
        (self.layers[name].show if self.show[name] else self.layers[name].hide)()
        return self.show[name]

    def _replace(self, layer, node):
        old = self._nodes.get(layer)
        if old is not None:
            old.removeNode()
        self._nodes[layer] = self.layers[layer].attachNewNode(node) if node is not None else None
        return self._nodes[layer]

    def clear(self):
        for layer in list(self._nodes):
            self._replace(layer, None)
        self._last = {}
        for child in self.layers["markers"].getChildren():
            child.removeNode()

    def update(self, state, frame):
        tk = state.tick
        if tk is None:
            return
        # Trails, twice a second.
        if frame % 15 == 0:
            self._replace("truth_trail", lines("truth", [list(state.truth_trail) + [(tk["truth"]["x"], tk["truth"]["y"])]], TRUTH, 2.0, 0.06))
            if state.est_trail:
                self._replace("est_trail", lines("est", [list(state.est_trail)], EST, 2.0, 0.07))
        if state.route is not None and self._last.get("route") is not state.route:
            self._last["route"] = state.route
            self._replace("route", lines("route", [state.route["pts"]], ROUTE, 1.5, 0.04))
        if state.path is not None and self._last.get("path") is not state.path:
            self._last["path"] = state.path
            pts = state.path["pts"]
            dashes = [pts[i:i + 3] for i in range(0, len(pts) - 2, 4)]
            self._replace("path", lines("path", dashes, PATH, 3.0, 0.08))
        if state.gnss and self._last.get("gnss") is not state.gnss[-1]:
            self._last["gnss"] = state.gnss[-1]
            b = Builder("gnss")
            fixes = list(state.gnss)[-(20 if self.compact else 60):]
            for i, fx in enumerate(fixes):
                newest = i == len(fixes) - 1
                r = (0.18 if newest else 0.07) if self.compact else (0.24 if newest else 0.11)
                a = 0.30 + 0.70 * (i + 1) / len(fixes)
                b.box(fx["x"], fx["y"], 0.08, 2 * r, 2 * r, 0.04, 0.785, (GNSS[0], GNSS[1], GNSS[2], a))
            n = self._replace("gnss", b.node())
            n.setTransparency(TransparencyAttrib.MAlpha)
        if state.scans and self._last.get("scan") is not state.scans[-1]:
            self._last["scan"] = state.scans[-1]
            self._draw_scans(state)
        self._draw_ellipse(tk)
        self._draw_markers(state)
        if state.closure is not None and self._last.get("closure") is not state.closure:
            self._last["closure"] = state.closure
            self._draw_closure(state.closure)

    def _draw_scans(self, state):
        b = Builder("scan")
        rays = []
        placed = Builder("placed")
        latest = state.scans[-1]
        est = (state.tick or {}).get("est")
        for k, scan in enumerate(state.scans):
            colour = {"MATCHED": USED, "REJECTED": REJECTED}.get(scan.get("match"), NOFILTER)
            fade = 0.25 + 0.75 * (k + 1) / len(state.scans)
            (sx, sy), pts = g.scan_points(scan)
            for x, y in pts:
                b.box(x, y, 0.14, 0.16, 0.16, 0.10, 0.0, (colour[0], colour[1], colour[2], fade))
            if scan is latest:
                rays = [[(sx, sy, 0.20), (x, y, 0.20)] for x, y in pts]
                if est and est.get("valid"):
                    _, epts = g.scan_points(scan, (est["x"], est["y"], est["yaw"]))
                    for x, y in epts:
                        placed.box(x, y, 0.15, 0.14, 0.14, 0.12, 0.0, PLACED)
        n = self._replace("scan", b.node())
        n.setTransparency(TransparencyAttrib.MAlpha)
        colour = {"MATCHED": USED, "REJECTED": REJECTED}.get(latest.get("match"), NOFILTER)
        ray = lines("rays", rays, (colour[0], colour[1], colour[2], 0.35), 1.0)
        rn = n.attachNewNode(ray)
        rn.setTransparency(TransparencyAttrib.MAlpha)
        self._replace("placed", placed.node())

    def _draw_ellipse(self, tk):
        est = tk.get("est")
        if not est or not est.get("valid"):
            self._replace("ellipse", None)
            return
        pts = g.covariance_ellipse(est["pxx"], est["pxy"], est["pyy"], CHI2_95_2D)
        ring = [(est["x"] + dx, est["y"] + dy, 0.10) for dx, dy in pts]
        polys = [ring]
        sh = math.sqrt(max(est["pyawyaw"], 0.0)) * CHI2_95_2D if est["pyawyaw"] else 0.0
        if sh > 0:
            for s in (-1, 1):
                a = est["yaw"] + s * min(sh, 1.0)
                polys.append([(est["x"], est["y"], 0.10), (est["x"] + 6.0 * math.cos(a), est["y"] + 6.0 * math.sin(a), 0.10)])
        self._replace("ellipse", lines("ellipse", polys, ELLIPSE, 2.0))

    def _draw_markers(self, state):
        have = self._last.setdefault("markers", 0)
        markers = list(state.markers)
        if len(markers) == have:
            return
        for mk in markers[have:]:
            if mk.get("pos") is None:
                continue
            x, y = mk["pos"]
            col = MARKER_COLOURS.get(mk["event"], (1, 1, 1, 1))
            tn = TextNode("label")
            tn.setText(_event_label(mk))
            tn.setTextColor(*col)
            tn.setAlign(TextNode.ACenter)
            tn.setCardColor(0.05, 0.06, 0.08, 0.75)
            tn.setCardAsMargin(0.3, 0.3, 0.15, 0.15)
            tn.setCardDecal(True)
            # Labels of events at the same place stack upwards instead of overlapping: each sits a small
            # gap above the top of the one below. Heights are in the label's own units, scaled with it.
            stack = 0.0
            for n in self.layers["markers"].getChildren():
                nx, ny = n.getPythonTag("xy")
                if math.hypot(nx - x, ny - y) < 2.0:
                    stack = max(stack, n.getPythonTag("stack") + n.getPythonTag("top") + 0.25 - (tn.getBottom() - 0.15))
            node = self.layers["markers"].attachNewNode("marker")
            node.setPythonTag("xy", (x, y))
            node.setPythonTag("stack", stack)
            node.setPythonTag("top", tn.getTop() + 0.15)
            b = Builder("pole")
            b.box(x, y, 0.0, 0.12, 0.12, 3.2, 0.0, col)
            node.attachNewNode(b.node()).setName("pole")
            text = node.attachNewNode(tn)
            text.setName("label")
            text.setScale(0.32)
            text.setPos(x, y, 3.6 + 0.32 * stack)
            text.setBillboardPointEye()
            text.setDepthTest(False)
            text.setBin("fixed", 50)
        self._last["markers"] = len(markers)

    def scale_labels(self, cam):
        """Keeps the marker labels about the same size on screen, and hides markers right at the camera."""
        for node in self.layers["markers"].getChildren():
            x, y = node.getPythonTag("xy")
            d = math.sqrt((cam[0] - x) ** 2 + (cam[1] - y) ** 2 + (cam[2] - 3.6) ** 2)
            label = node.find("label")
            if d < 7.0:
                node.hide()
                continue
            node.show()
            if not label.isEmpty():
                (label.show if self.labels else label.hide)()
                s = max(0.12, min(3.5, 0.018 * d))
                label.setScale(s)
                label.setZ(3.6 + s * node.getPythonTag("stack"))

    def _draw_closure(self, c):
        b = Builder("closure")
        if c["present"]:
            x, y, yaw = c["barrier"]
            w = c["lane_width"]
            nx, ny = -math.sin(yaw), math.cos(yaw)
            for k in range(6):
                f = -0.5 + (k + 0.5) / 6.0
                b.box(x + nx * f * w, y + ny * f * w, 0.0, 0.08, w / 6.0, 1.0, yaw, BARRIER_RED if k % 2 == 0 else BARRIER_WHITE)
            ex, ey, _ = c["works_end"]
            L = c["works_length"]
            for k in range(int(L // 2.5) + 1):
                f = k / max(1, int(L // 2.5))
                cx, cy = x + (ex - x) * f, y + (ey - y) * f
                for side in (-0.45, 0.45):
                    b.box(cx + nx * side * w, cy + ny * side * w, 0.0, 0.3, 0.3, 0.7, yaw, CONE)
        self._replace("closure", b.node())

    def footprints(self, params, truth, est):
        polys = [[(x, y, 0.09) for x, y in g.footprint(truth["x"], truth["y"], truth["yaw"], params["length"], params["width"], params["rear_overhang"])]]
        polys[0].append(polys[0][0])
        if est and est.get("valid"):
            e = [(x, y, 0.11) for x, y in g.footprint(est["x"], est["y"], est["yaw"], params["length"], params["width"], params["rear_overhang"])]
            self._replace("footprints", lines("fp", [polys[0], e + [e[0]]], EST, 2.5))
        else:
            self._replace("footprints", lines("fp", polys, TRUTH, 1.5))


def _event_label(mk):
    d = mk.get("detail", "")
    kind = mk["event"]
    if kind == "POSE_CORRECTION":
        jump = next((p[5:] for p in d.split(";") if p.startswith("jump=")), "?")
        return "stack re-joins %s m\n(%s)" % (jump, d.split(";")[0] if ";" in d else d)
    if kind == "KERB_CORRECTION":
        return "kerb correction %.2f m\n%s" % (math.hypot(mk.get("dx", 0.0), mk.get("dy", 0.0)), "unique match" if mk.get("unique") else "NOT unique")
    if kind == "KERB_RELOCALISATION":
        return "kerb re-localisation"
    return kind.replace("_", " ").lower()
