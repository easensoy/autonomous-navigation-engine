#!/usr/bin/env python3
"""Computes one trial's metrics from its rosbag2 recording and checks the scenario's acceptance criteria.

  analyze_trial.py <trial dir> <bag dir> --route-check <ane_route_check> [--roadmap <file>]

Reads <trial dir>/runner_result.json (scenario_runner.py), the bag, the roadmap and the scenario file.
Writes <trial dir>/metrics.json (metrics, acceptance verdicts, consistency checks) and
<trial dir>/trajectory.json (downsampled ground truth, routes, blockages and events for the figures).
Metric definitions: ros2/README.md, "Metrics".

Robot motion always comes from the simulator's ground truth (/ground_truth/odom), never from the
navigator. The navigator's claims (outcome, final error, reached nodes) are compared with it.
"""

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

sys.path.insert(0, str(Path(__file__).resolve().parent))
from scenario_lib import TERMINAL_STATES, compare, load_roadmap, load_scenario  # noqa: E402

FOOTPRINT = (0.40, 0.37)         # length, width (m): chassis length x outer wheel faces
GOAL_TOLERANCE = 0.10            # m
STOP_SPEED = 0.01                # m/s and rad/s: below both the robot counts as stopped
CMD_LIMITS = (0.5, 1.0)          # navigator limits: |v| m/s, |w| rad/s
TOPICS = ["/clock", "/ground_truth/odom", "/wheel_odom", "/cmd_vel_nav", "/cmd_vel_robot", "/ane/status",
          "/ane/events", "/ane/fault_events", "/ane/blockage_reports", "/ane/estop", "/ane/planned_route"]


def stamp(t):
    return t.sec + t.nanosec * 1e-9


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in TOPICS if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    classes = {t: get_message(types[t]) for t in wanted}
    data = {t: [] for t in TOPICS}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        data[topic].append((t_ns * 1e-9, deserialize_message(raw, classes[topic])))
    return data


def point_segment(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / length2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def convex_intersect(a, b):
    for poly in (a, b):
        for i in range(len(poly)):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % len(poly)]
            nx, ny = y2 - y1, x1 - x2
            pa = [nx * x + ny * y for x, y in a]
            pb = [nx * x + ny * y for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def convex_distance(a, b):
    """Exact distance between two convex polygons (0 when they overlap or touch)."""
    if convex_intersect(a, b):
        return 0.0
    best = math.inf
    for p, q in ((a, b), (b, a)):
        for v in p:
            for i in range(len(q)):
                best = min(best, point_segment(v, q[i], q[(i + 1) % len(q)]))
    return best


def footprint(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    hl, hw = FOOTPRINT[0] / 2, FOOTPRINT[1] / 2
    return [(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))]


def rect_polygon(r):
    x0, y0, x1, y1 = r
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def rect_center_distance(x, y, r):
    x0, y0, x1, y1 = r
    return math.hypot(max(x0 - x, 0.0, x - x1), max(y0 - y, 0.0, y - y1))


def route_check(tool, roadmap_file, start, goal, blocked):
    out = subprocess.run([tool, str(roadmap_file), str(start), str(goal), *map(str, blocked)],
                         capture_output=True, text=True, check=True).stdout.split("\n")
    if out[0].strip() == "no-route":
        return None, None
    return [int(v) for v in out[0].split()[1:]], float(out[1].split()[1])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("trial_dir", type=Path)
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("--route-check", required=True)
    parser.add_argument("--roadmap", type=Path,
                        default=Path(__file__).resolve().parents[1] / "maps" / "warehouse_roadmap.txt")
    parser.add_argument("--scenario-dir", type=Path, default=Path(__file__).resolve().parents[1] / "scenarios")
    args = parser.parse_args()

    runner = json.loads((args.trial_dir / "runner_result.json").read_text())
    scenario = load_scenario(args.scenario_dir / runner["scenario_file"])
    roadmap = load_roadmap(args.roadmap)
    nodes = roadmap["nodes"]
    data = read_bag(args.bag_dir)

    # Receive times: the trials record with --use-sim-time, so a message's receive time is simulation
    # time. Older bags with wall-clock receive times are mapped through /clock.
    clock_recv = np.array([w for w, _ in data["/clock"]])
    clock_sim = np.array([stamp(m.clock) for _, m in data["/clock"]])
    sim_recorded = bool(np.median(np.abs(clock_recv - clock_sim)) < 0.05)
    order = np.argsort(clock_recv, kind="stable")
    to_sim = (lambda w: w) if sim_recorded else (lambda w: float(np.interp(w, clock_recv[order], clock_sim[order])))  # noqa: E731

    # Every analysis orders messages by their own simulation-time stamps (and events by sequence).
    gt = np.array(sorted((stamp(m.header.stamp), m.pose.pose.position.x, m.pose.pose.position.y,
                          2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w))
                         for _, m in data["/ground_truth/odom"]))
    gt_t, gt_x, gt_y, gt_yaw = gt[:, 0], gt[:, 1], gt[:, 2], gt[:, 3]
    step = np.hypot(np.diff(gt_x), np.diff(gt_y))
    dt = np.maximum(np.diff(gt_t), 1e-6)
    speed = np.concatenate([[0.0], step / dt])
    turn = np.concatenate([[0.0], np.abs(np.diff(np.unwrap(gt_yaw))) / dt])

    status = sorted(((stamp(m.stamp), m) for _, m in data["/ane/status"]), key=lambda x: x[0])
    events = sorted(((stamp(m.stamp), m) for _, m in data["/ane/events"]), key=lambda x: (x[0], x[1].sequence))
    faults = sorted(((stamp(m.stamp), m) for _, m in data["/ane/fault_events"]), key=lambda x: (x[0], x[1].sequence))
    # The navigator's command of each control period, with its stamp, from its status message.
    commands = [(t, s.cmd_linear_mps, s.cmd_angular_radps) for t, s in status]
    robot_commands = sorted((to_sim(w), m.linear.x, m.angular.z) for w, m in data["/cmd_vel_robot"])

    mission = runner["mission"]
    t_req = mission.get("requested_sim", float(gt_t[0]))
    goal = mission.get("goal_node", roadmap["stations"][scenario.goal])
    start_node = mission.get("start_node", -1)
    terminal_events = [t for t, e in events if e.type == "STATE" and any(f"-> {s}" in e.detail for s in TERMINAL_STATES)]
    t_term = terminal_events[0] if terminal_events else None
    final_state = status[-1][1].mission_state if status else "NONE"
    t_end = float(gt_t[-1])
    window = (gt_t >= t_req) & (gt_t <= (t_term if t_term is not None else t_end))

    def state_at(t):
        current = "IDLE"
        for ts, s in status:
            if ts > t:
                break
            current = s.mission_state
        return current

    # Obstacles: static map plus spawned obstacles from their confirmed appearance onwards.
    static_rects = [o["rect"] for o in roadmap["obstacles"]]
    spawned = []
    for e in runner["events"]:
        if e["action"] == "spawn_obstacle" and e.get("ok"):
            g = e["geometry"]
            spawned.append(((g["cx"] - g["sx"] / 2, g["cy"] - g["sy"] / 2, g["cx"] + g["sx"] / 2, g["cy"] + g["sy"] / 2),
                            e["confirmed_sim"], e["args"][0]))

    clearance = np.full(len(gt_t), np.inf)
    radius = math.hypot(*FOOTPRINT) / 2
    for i in range(len(gt_t)):
        rects = static_rects + [r for r, t0, _ in spawned if gt_t[i] >= t0]
        poly = None
        for r in rects:
            rough = rect_center_distance(gt_x[i], gt_y[i], r) - radius
            if rough > 1.0 or rough > clearance[i]:
                clearance[i] = min(clearance[i], max(rough, 0.0)) if rough > 1.0 else clearance[i]
                continue
            poly = poly or footprint(gt_x[i], gt_y[i], gt_yaw[i])
            clearance[i] = min(clearance[i], convex_distance(poly, rect_polygon(r)))
    colliding = clearance <= 0.0
    collisions = int(np.sum(colliding[1:] & ~colliding[:-1]) + (1 if colliding[0] else 0))

    # Lateral deviation from the roadmap (its edges are the clearance-checked corridors).
    edges = np.array([(*nodes[a], *nodes[b]) for a, b, _ in roadmap["edges"]])
    ax, ay, bx, by = edges.T
    executing = np.array([window[i] and state_at(gt_t[i]) == "EXECUTING" for i in range(len(gt_t))])
    px, py = gt_x[executing][:, None], gt_y[executing][:, None]
    dx, dy = bx - ax, by - ay
    tt = np.clip(((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy), 0.0, 1.0)
    deviation = np.min(np.hypot(px - (ax + tt * dx), py - (ay + tt * dy)), axis=1) if px.size else np.array([0.0])

    # Events by type.
    def of(kind, source=None):
        return [(t, e) for t, e in events if e.type == kind and (source is None or e.source == source)]

    reports = of("BLOCKAGE_REPORTED")
    sensor_reports = [r for r in reports if r[1].source == "SENSOR"]
    operator_reports = [r for r in reports if r[1].source in ("OPERATOR", "SCRIPTED")]
    reached = [(t, e.nodes[0]) for t, e in of("NODE_REACHED")]
    blocked_at = {}
    for t, e in reports:
        for n in e.nodes:
            blocked_at.setdefault(n, t)
    blocked_entered = sum(1 for t, n in reached if n in blocked_at and blocked_at[n] < t)

    # Every route the engine produced, re-planned offline with the same blocked nodes.
    route_checks = []
    blocked_so_far = []
    for t, e in events:
        if e.type == "BLOCKAGE_REPORTED":
            blocked_so_far += [n for n in e.nodes if n not in blocked_so_far]
        elif e.type in ("ROUTE_PLANNED", "ROUTE_REPLACED"):
            expected, cost = route_check(args.route_check, args.roadmap, e.nodes[0], goal, blocked_so_far)
            route_checks.append({"t": t, "type": e.type, "identical": expected == list(e.nodes),
                                 "engine_cost": cost, "nodes": len(e.nodes), "blocked": list(blocked_so_far)})
        elif e.type == "ROUTE_BLOCKED":
            expected, _ = route_check(args.route_check, args.roadmap, e.nodes[0], goal, blocked_so_far)
            route_checks.append({"t": t, "type": e.type, "identical": expected is None, "blocked": list(blocked_so_far)})
    routes_seen = {tuple(s.route) for _, s in status}
    published_paths_ok = True
    for _, path in data["/ane/planned_route"]:
        ids = []
        for pose in path.poses:
            p = (pose.pose.position.x, pose.pose.position.y)
            match = [n for n, q in nodes.items() if abs(q[0] - p[0]) < 1e-6 and abs(q[1] - p[1]) < 1e-6]
            ids.append(match[0] if match else -1)
        published_paths_ok &= (not ids) or tuple(ids) in routes_seen

    # Stopping after the terminal state.
    stop_time = drift = None
    if t_term is not None:
        still = (speed < STOP_SPEED) & (turn < STOP_SPEED)
        after = np.where(gt_t >= t_term)[0]
        for k, i in enumerate(after):
            if np.all(still[after[k:]]):
                stop_time = float(gt_t[i] - t_term)
                drift = float(np.max(np.hypot(gt_x[i:] - gt_x[i], gt_y[i:] - gt_y[i])))
                break

    def zero_between(cmds, t0, t1):
        sel = [(v, w) for t, v, w in cmds if t0 <= t <= t1]
        return all(abs(v) < 1e-9 and abs(w) < 1e-9 for v, w in sel), len(sel)

    # Faults.
    fault_start = next((t for t, e in faults if e.type == "FAULT_STARTED"), None)
    fault_end = next((t for t, e in faults if e.type == "FAULT_ENDED"), None)
    stale = of("FEEDBACK_STALE")
    recovered = of("FEEDBACK_RECOVERED")
    hold_detect = blind = None
    zero_in_hold = None
    if stale and fault_start is not None:
        t_stale = stale[0][0]
        hold_detect = t_stale - fault_start
        i0 = int(np.searchsorted(gt_t, fault_start))
        moving_after = np.where((gt_t > t_stale) & (speed < STOP_SPEED))[0]
        i1 = int(moving_after[0]) if moving_after.size else len(gt_t) - 1
        blind = float(np.sum(step[i0:i1]))
        hold_end = recovered[0][0] if recovered else t_end
        zero_in_hold, _ = zero_between(commands, t_stale + 0.02, hold_end - 0.02)
    failure_detect = None
    failed_controller = [t for t, e in events if e.type == "STATE" and "-> FAILED_CONTROLLER" in e.detail]
    if failed_controller and fault_start is not None:
        failure_detect = failed_controller[0] - fault_start

    completed = of("MISSION_COMPLETED")
    final_error = float(math.hypot(gt_x[-1] - nodes[goal][0], gt_y[-1] - nodes[goal][1]))
    reported_error = completed[0][1].value if completed else None
    error_at_claim = None
    if completed:
        i = int(np.searchsorted(gt_t, completed[0][0]))
        i = min(i, len(gt_t) - 1)
        error_at_claim = float(math.hypot(gt_x[i] - nodes[goal][0], gt_y[i] - nodes[goal][1]))

    last = status[-1][1] if status else None
    final_route = list(last.route) if last else []
    progress = (last.route_index / (len(final_route) - 1)) if last and len(final_route) > 1 else (1.0 if final_state == "COMPLETED" else 0.0)
    walk = [start_node] + [n for _, n in reached]
    walk_ok = all((a, b) in roadmap["weights"] for a, b in zip(walk, walk[1:])) and walk == final_route[:len(walk)]
    executed_cost = float(sum(roadmap["weights"][(a, b)] for a, b in zip(walk, walk[1:]) if (a, b) in roadmap["weights"]))

    estops = sum(1 for _, m in data["/ane/estop"] if m.data)
    scripted_closures = sum(1 for e in scenario.events if e.action == "close_region")
    interventions = estops + len(of("STOP_REQUESTED")) + len(of("ROUTE_RESTORED")) + max(0, len(data["/ane/blockage_reports"]) - scripted_closures)

    spawn_t = spawned[0][1] if spawned else None
    spawn_fired = next((e["fired_sim"] for e in runner["events"] if e["action"] == "spawn_obstacle"), None)
    detection = sensor_reports[0][0] - spawn_t if sensor_reports and spawn_t is not None else None
    replans = of("ROUTE_REPLACED")
    reaction = next((t - spawn_t for t, e in events if spawn_t is not None and t >= spawn_t and e.type in ("ROUTE_REPLACED", "ROUTE_BLOCKED")), None)
    engine_ms = {kind: [e.value for _, e in of(kind)] for kind in ("ROUTE_PLANNED", "ROUTE_REPLACED", "ROUTE_KEPT", "ROUTE_BLOCKED")}

    t_stop_window = t_term if t_term is not None else t_end
    # Wall-clock mission duration from the runner's monotonic clock (request -> terminal state).
    terminal = runner.get("terminal") or {}
    wall_mission = (terminal["mono"] - mission["requested_mono"]) if "mono" in terminal and "requested_mono" in mission else float("nan")
    zero_after, _ = zero_between(commands, (t_term or t_end) + 0.02, t_end) if t_term is not None else (None, 0)
    robot_zero_after, _ = zero_between(robot_commands, (t_term or t_end) + 0.15, t_end) if t_term is not None else (None, 0)
    within_limits = all(abs(v) <= CMD_LIMITS[0] + 1e-9 and abs(w) <= CMD_LIMITS[1] + 1e-9 for _, v, w in commands)

    m = {
        "outcome": final_state,
        "runner_outcome": runner["runner_outcome"],
        "goal_reached": bool(final_error <= GOAL_TOLERANCE),
        "final_position_error_m": final_error,
        "collisions": collisions,
        "min_clearance_m": float(np.min(clearance[window])) if np.any(window) else float(np.min(clearance)),
        "waypoint_progress": float(round(progress, 6)),
        "interventions": interventions,
        "max_cross_track_m": float(np.max(deviation)),
        "rms_cross_track_m": float(np.sqrt(np.mean(deviation ** 2))),
        "replans": len(replans),
        "sensor_blockage_reports": len(sensor_reports),
        "operator_blockage_reports": len(operator_reports),
        "blocked_nodes": sorted(blocked_at),
        "detection_latency_s": detection,
        "detection_latency_from_trigger_s": (sensor_reports[0][0] - spawn_fired) if sensor_reports and spawn_fired is not None else None,
        "replan_reaction_s": reaction,
        "engine_call_ms": engine_ms,
        "blocked_nodes_entered": blocked_entered,
        "route_matches_engine": bool(route_checks) and all(c["identical"] for c in route_checks),
        "published_routes_match_status": bool(published_paths_ok),
        "stop_time_after_terminal_s": stop_time,
        "drift_after_stop_m": drift,
        "feedback_holds": len(stale),
        "feedback_recoveries": len(recovered),
        "hold_detect_s": hold_detect,
        "blind_travel_m": blind,
        "commands_zero_during_hold": zero_in_hold,
        "failure_detect_s": failure_detect,
        "commands_zero_after_terminal": zero_after,
        "robot_commands_zero_after_terminal": robot_zero_after,
        "commands_within_limits": within_limits,
        "path_length_m": float(np.sum(step[window[1:]])),
        "executed_graph_cost": executed_cost,
        "planned_route_cost": mission.get("route_cost"),
        "final_route_cost_ahead": float(last.remaining_cost) if last and math.isfinite(last.remaining_cost) else None,
        "mission_time_s": (t_term - t_req) if t_term is not None else None,
        "mission_wall_s": wall_mission if math.isfinite(wall_mission) else None,
        "real_time_factor": (t_stop_window - t_req) / wall_mission if math.isfinite(wall_mission) and wall_mission > 0 else None,
        "bag_receive_clock": "simulation" if sim_recorded else "wall",
        "fault_window_s": [fault_start, fault_end],
    }
    checks = {
        "navigator_outcome_equals_runner": m["outcome"] == m["runner_outcome"],
        "reported_final_error_matches_ground_truth": (reported_error is None) or abs(reported_error - error_at_claim) <= 0.02,
        "reached_nodes_follow_roadmap_and_route": walk_ok,
        "every_engine_route_reproduced_offline": m["route_matches_engine"] or not route_checks,
        "published_routes_are_engine_routes": m["published_routes_match_status"],
        "commands_within_limits": within_limits,
    }
    verdicts = []
    for metric, op, target in scenario.accept:
        value = m.get(metric)
        verdicts.append({"metric": metric, "op": op, "target": target, "value": value, "pass": compare(value, op, target)})
    result = {
        "schema": "ane-sim/trial-metrics/1",
        "scenario": scenario.name,
        "trial": runner["trial"],
        "seed": runner["seed"],
        "metrics": m,
        "acceptance": verdicts,
        "accepted": all(v["pass"] for v in verdicts),
        "consistency": checks,
        "consistent": all(checks.values()),
        "route_checks": route_checks,
        "navigator_reported_final_error_m": reported_error,
        "ground_truth_error_at_claim_m": error_at_claim,
    }
    (args.trial_dir / "metrics.json").write_text(json.dumps(result, indent=1) + "\n", encoding="utf-8")

    keep = np.where(np.concatenate([[True], np.diff(np.floor(gt_t * 10)) > 0]))[0]
    # The navigator's own status (ane/status, 20 Hz), one message per 0.1 s of simulation time: what the
    # recordings' telemetry panels show, so every value displayed there can be checked in this file.
    status_rows, last_bucket = [], None
    for t, s in status:
        bucket = math.floor(t * 10)
        if bucket == last_bucket:
            continue
        last_bucket = bucket
        cost = s.remaining_cost if math.isfinite(s.remaining_cost) else None
        status_rows.append([round(t, 3), s.mission_state, s.engine_state, s.control_mode, s.current_node,
                            s.target_node, int(s.returning_to_node), s.route_version, s.replan_count,
                            None if cost is None else round(cost, 3), round(s.cross_track_m, 4),
                            round(s.feedback_age_s, 3), round(s.cmd_linear_mps, 3), round(s.cmd_angular_radps, 3)])
    trajectory = {
        "schema": "ane-sim/trajectory/1",
        "scenario": scenario.name, "trial": runner["trial"], "seed": runner["seed"],
        "ground_truth": [[round(float(gt_t[i]), 3), round(float(gt_x[i]), 4), round(float(gt_y[i]), 4),
                          round(float(gt_yaw[i]), 4)] for i in keep],
        "routes": [{"t": t, "type": e.type, "nodes": list(e.nodes)} for t, e in events
                   if e.type in ("ROUTE_PLANNED", "ROUTE_REPLACED", "ROUTE_BLOCKED", "ROUTE_RESTORED")],
        "blockages": [{"t": t, "source": e.source, "nodes": list(e.nodes), "detail": e.detail} for t, e in reports],
        "obstacles": [{"rect": r, "t": t0, "name": n} for r, t0, n in spawned],
        "states": [{"t": t, "detail": e.detail} for t, e in events if e.type == "STATE"],
        "faults": [{"t": t, "type": e.type, "detail": e.detail, "value": e.value} for t, e in faults],
        "mission_request": t_req, "terminal": t_term, "outcome": final_state,
        # Every navigator event in the order the navigator raised it (stamp, then sequence number), so a
        # checker can replay the engine calls: blocked nodes accumulate, routes must come out identical.
        "event_log": [{"t": round(t, 3), "seq": e.sequence, "type": e.type, "source": e.source, "nodes": list(e.nodes),
                       "value": e.value} for t, e in events],
        "status_columns": ["t", "mission_state", "engine_state", "control_mode", "current_node", "target_node",
                           "returning_to_node", "route_version", "replan_count", "remaining_cost", "cross_track_m",
                           "feedback_age_s", "cmd_linear_mps", "cmd_angular_radps"],
        "status": status_rows,
    }
    (args.trial_dir / "trajectory.json").write_text(json.dumps(trajectory) + "\n", encoding="utf-8")
    print(f"{scenario.name} trial {runner['trial']}: {final_state}; accepted {result['accepted']}; consistent {result['consistent']}")
    for v in verdicts:
        if not v["pass"]:
            print(f"  FAIL {v['metric']} {v['op']} {v['target']} (value {v['value']})")
    for k, ok in checks.items():
        if not ok:
            print(f"  INCONSISTENT {k}")


if __name__ == "__main__":
    main()
