"""Shared helpers for the ane_sim scripts: the scenario file format and small numeric utilities.

Standard library only, so the host-side tools (ros2/tools) can import it too.
"""

import hashlib
import math
import random
from dataclasses import dataclass, field
from pathlib import Path

TERMINAL_STATES = {"COMPLETED", "BLOCKED", "FAILED_CONTROLLER", "FAILED_LOCALIZATION", "STOPPED"}
TRIGGERS = {"at_start", "x_ge", "x_le", "y_ge", "y_le", "time_ge"}
ACTIONS = {"spawn_obstacle": 6, "close_region": 5, "odom_dropout": 1, "drive_stall": 1}


@dataclass
class Event:
    trigger: str
    value: float
    action: str
    args: list


@dataclass
class Scenario:
    path: Path
    name: str = ""
    title: str = ""
    start: str = ""
    goal: str = ""
    trials: int = 1
    seed: int = 0
    jitter: tuple = (0.0, 0.0, 0.0)
    timeout: float = 120.0
    observe: float = 3.0
    events: list = field(default_factory=list)
    accept: list = field(default_factory=list)  # (metric, op, value)

    def sha256(self):
        return hashlib.sha256(self.path.read_bytes()).hexdigest()

    def initial_pose(self, station_xy, trial):
        """Seeded initial pose of trial `trial`: uniform jitter around the start station, facing +x."""
        rng = random.Random(self.seed + trial)
        jx, jy, jyaw = self.jitter
        return (station_xy[0] + rng.uniform(-jx, jx), station_xy[1] + rng.uniform(-jy, jy), rng.uniform(-jyaw, jyaw))


def load_scenario(path):
    path = Path(path)
    s = Scenario(path=path)
    header = False
    for number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.split("#", 1)[0].split()
        if not line:
            continue
        key, args = line[0], line[1:]
        where = f"{path.name}:{number}"
        if not header:
            if key != "ane-scenario" or args != ["1"]:
                raise ValueError(f"{where}: expected 'ane-scenario 1'")
            header = True
        elif key in ("name", "start", "goal"):
            setattr(s, key, args[0])
        elif key == "title":
            s.title = " ".join(args)
        elif key == "trials":
            s.trials = int(args[0])
        elif key == "seed":
            s.seed = int(args[0])
        elif key == "initial_jitter":
            s.jitter = tuple(float(v) for v in args[:3])
        elif key == "timeout":
            s.timeout = float(args[0])
        elif key == "observe":
            s.observe = float(args[0])
        elif key == "event":
            trigger = args[0]
            if trigger not in TRIGGERS:
                raise ValueError(f"{where}: unknown trigger {trigger}")
            value, rest = (0.0, args[1:]) if trigger == "at_start" else (float(args[1]), args[2:])
            action, action_args = rest[0], rest[1:]
            if ACTIONS.get(action) != len(action_args):
                raise ValueError(f"{where}: action {action} needs {ACTIONS.get(action)} arguments")
            s.events.append(Event(trigger, value, action, action_args))
        elif key == "accept":
            s.accept.append((args[0], args[1], args[2]))
        else:
            raise ValueError(f"{where}: unknown keyword {key}")
    if not (s.name and s.start and s.goal):
        raise ValueError(f"{path}: a scenario needs name, start and goal")
    return s


def load_roadmap(path):
    """Nodes (map frame), edges, stations, obstacles and zones of an ane-roadmap 1 file."""
    frame = (1.0, 0.0, 0.0, 0.0)
    nodes, edges, stations, obstacles, zones = {}, [], {}, [], []
    for raw in Path(path).read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].split()
        if not line:
            continue
        key, args = line[0], line[1:]
        if key == "frame":
            frame = tuple(float(v) for v in args)
        elif key == "node":
            nodes[int(args[0])] = (float(args[1]), float(args[2]))
        elif key == "edge":
            edges.append((int(args[0]), int(args[1]), float(args[2])))
        elif key == "station":
            stations[args[0]] = int(args[1])
        elif key == "obstacle":
            obstacles.append({"kind": args[0], "rect": tuple(float(v) for v in args[1:5]), "height": float(args[5]), "label": args[6]})
        elif key == "zone":
            zones.append({"rect": tuple(float(v) for v in args[:4]), "multiplier": float(args[4]), "label": args[5]})
    scale, yaw, ox, oy = frame
    c, s = math.cos(yaw), math.sin(yaw)
    to_map = {i: (c * scale * gx - s * scale * gy + ox, s * scale * gx + c * scale * gy + oy) for i, (gx, gy) in nodes.items()}
    weights = {}
    for a, b, w in edges:
        weights[(a, b)] = w
        weights[(b, a)] = w
    return {"nodes": to_map, "edges": edges, "weights": weights, "stations": stations, "obstacles": obstacles, "zones": zones}


def compare(value, op, target):
    """Evaluates one acceptance criterion `<metric> <op> <target>` against a metric value."""
    if isinstance(value, bool) or target in ("true", "false"):
        return op == "==" and value == (target == "true")
    if isinstance(value, str):
        return op == "==" and value == target
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return False
    target = float(target)
    return {"==": value == target, "<=": value <= target, ">=": value >= target, "<": value < target,
            ">": value > target}[op]


TRIAL_INPUT_DIRS = ("docker", "ane_interfaces", "ane_description", "ane_navigation", "ane_sim")
TRIAL_INPUT_TOOLS = ("container_build.sh", "container_trial.sh", "container_capture.sh", "run_trials.py")


def integration_tree_sha256(root):
    """SHA-256 over everything that produces trial data and metrics, given `root` = the ros2 folder: the
    image definition (docker/), the four packages (sources, launch, configuration, world, roadmap,
    scenarios, runner and analysis scripts) and the scripts that run trials (tools/container_*.sh,
    tools/run_trials.py). Not hashed: Markdown, trial outputs (evidence/) and the tools that only
    summarise, draw or check outputs. Hashed as sorted relative paths plus contents, with line endings
    normalised to LF so that Windows and Linux checkouts hash alike."""
    root = Path(root)
    h = hashlib.sha256()
    files = sorted(p for p in root.rglob("*") if p.is_file() and "__pycache__" not in p.parts and p.suffix.lower() != ".md"
                   and (p.relative_to(root).parts[0] in TRIAL_INPUT_DIRS
                        or (p.parent == root / "tools" and p.name in TRIAL_INPUT_TOOLS)))
    for path in files:
        h.update(str(path.relative_to(root)).replace("\\", "/").encode())
        h.update(b"\0")
        h.update(path.read_bytes().replace(b"\r\n", b"\n"))
        h.update(b"\0")
    return h.hexdigest()
