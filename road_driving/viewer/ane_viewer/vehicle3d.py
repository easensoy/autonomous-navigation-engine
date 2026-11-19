"""The vehicle in the scene: the procedural proxy, or an imported glTF/GLB model aligned to the same frame.

Either way the node's origin is the rear-axle centre on the ground, +X forward, +Y left, +Z up, so the
backend's pose (x, y, yaw) places it directly: no simulation code depends on which model is drawn.

An imported model is described by a JSON file next to it (see ../models/README.md): the model file, the
model's forward and up axes, and either `fit_length` (scale the model to the simulated vehicle's length)
or `scale`. The aligner reports how the model's extent compares with the simulated footprint.
"""
import json
import math
import pathlib

from panda3d.core import LPoint3f, NodePath, TransparencyAttrib

from . import vehicle_mesh as vm
from .align import alignment
from .render_util import mesh_node

AXES = {"+X": (1, 0, 0), "-X": (-1, 0, 0), "+Y": (0, 1, 0), "-Y": (0, -1, 0), "+Z": (0, 0, 1), "-Z": (0, 0, -1)}


def axis_rotation(forward, up):
    """HPR (degrees) turning a model whose forward and up axes are given into +X forward, +Z up."""
    f, u = AXES[forward], AXES[up]
    # Right-handed: left = up x forward.
    left = (u[1] * f[2] - u[2] * f[1], u[2] * f[0] - u[0] * f[2], u[0] * f[1] - u[1] * f[0])
    # Rows of the rotation taking model coordinates into body coordinates.
    from panda3d.core import LMatrix3f, LQuaternionf
    m = LMatrix3f(f[0], left[0], u[0], f[1], left[1], u[1], f[2], left[2], u[2])
    q = LQuaternionf()
    q.setFromMatrix(m)
    return q.getHpr()


class Vehicle:
    def __init__(self, loader, params, model_cfg=None, ghost=False):
        self.params = params
        self.root = NodePath("vehicle_ghost" if ghost else "vehicle")
        self.wheels = {}
        self.spin = 0.0
        self.report = None
        self.imported = False
        parts, wheels, positions = vm.proxy(params)
        if model_cfg and not ghost:
            self._import(loader, model_cfg)
        if ghost:
            self.root.attachNewNode(mesh_node(vm.body(params, stations=12, ring=10)))  # a coarse hull reads as an outline
        elif not self.imported:
            body = self.root.attachNewNode(mesh_node(parts["body"]))
            # Glossy paint: a specular highlight over the vertex colours (the colours stay the diffuse colour).
            from panda3d.core import Material
            paint = Material("paint")
            paint.setSpecular((0.55, 0.55, 0.55, 1.0))
            paint.setShininess(42.0)
            body.setMaterial(paint)
        self.wheel_radius = vm.design(params)["wheel_radius"]
        if not ghost:
            if (not self.imported or model_cfg.get("add_mirrors", False)) and parts["mirrors"].v:
                self.root.attachNewNode(mesh_node(parts["mirrors"]))
            if not self.imported or model_cfg.get("add_rig", True):
                if parts["roof_rig"].v:  # the Cybercab-inspired proxy has none
                    self.rig = self.root.attachNewNode(mesh_node(parts["roof_rig"]))
                self.root.attachNewNode(mesh_node(parts["gnss_antenna"]))
            self.root.attachNewNode(mesh_node(parts["kerb_scanner"]))
            if not self.imported or model_cfg.get("add_wheels", False):
                for key, mesh in wheels.items():
                    hub = self.root.attachNewNode("hub_" + key)
                    hub.setPos(*positions[key])
                    steer = hub.attachNewNode("steer_" + key)
                    spin = steer.attachNewNode("spin_" + key)
                    spin.attachNewNode(mesh_node(mesh))
                    self.wheels[key] = (steer, spin)
        else:
            # The estimate's car: a teal wireframe of the same body, so the true car stays visible under it.
            self.root.setRenderModeWireframe()
            self.root.setRenderModeThickness(1.0)
            self.root.setTransparency(TransparencyAttrib.MAlpha)
            self.root.setColorScale(0.20, 0.95, 0.95, 0.55)
            self.root.setTextureOff(1)
            self.root.setLightOff(1)
            self.root.setShaderOff(1)

    def _import(self, loader, cfg):
        path = pathlib.Path(cfg["model"])
        if not path.is_absolute():
            path = pathlib.Path(cfg["_dir"]) / path
        try:
            import gltf  # panda3d-gltf
            gltf.patch_loader(loader)
        except ImportError:
            pass
        model = loader.loadModel(str(path))
        holder = self.root.attachNewNode("imported")
        oriented = holder.attachNewNode("oriented")
        model.reparentTo(oriented)
        oriented.setHpr(axis_rotation(cfg.get("forward", "-Z"), cfg.get("up", "+Y")))
        bmin, bmax = LPoint3f(), LPoint3f()
        oriented.calcTightBounds(bmin, bmax, holder)
        scale, offset, report = alignment((bmin[0], bmin[1], bmin[2]), (bmax[0], bmax[1], bmax[2]), self.params, cfg)
        holder.setScale(scale)
        holder.setPos(*offset)
        report["model"] = str(path)
        report["attribution"] = cfg.get("attribution", "")
        report["licence"] = cfg.get("licence", "")
        self.report = report
        self.imported = True
        # Named wheel nodes, if the model has them, steer and spin like the proxy's.
        for key, node_name in (cfg.get("wheels") or {}).items():
            found = model.find("**/" + node_name)
            if not found.isEmpty():
                self.wheels[key] = (found, found)

    def set_pose(self, x, y, yaw, steer=0.0, v=0.0, dt=0.0):
        self.root.setPos(x, y, 0.0)
        self.root.setH(math.degrees(yaw))
        self.spin = (self.spin + math.degrees(v * dt / self.wheel_radius)) % 360.0
        for key, (steer_np, spin_np) in self.wheels.items():
            if key in ("fl", "fr"):
                steer_np.setH(math.degrees(steer))
            spin_np.setR(self.spin)


def load_model_config(path):
    p = pathlib.Path(path)
    cfg = json.loads(p.read_text(encoding="utf-8"))
    cfg["_dir"] = str(p.parent)
    return cfg
