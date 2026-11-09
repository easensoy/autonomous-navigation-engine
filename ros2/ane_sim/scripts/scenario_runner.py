#!/usr/bin/env python3
"""Runs one trial of a scenario and writes runner_result.json.

It waits for the simulation and the navigator, requests the mission, fires the scenario's events when
their triggers hold and stops once the navigator has been in a terminal state for the scenario's
observation window (or the timeout expired). The runner is the experimenter: it evaluates triggers on
the simulator's ground-truth pose. The navigator learns of scripted events only through the mechanism
each one uses:

  spawn_obstacle   a static model is created in Gazebo (the world's create service, through gz-transport);
                   the navigator can only detect it with its lidar
  close_region     an ane/blockage_reports message with source OPERATOR
  odom_dropout,    a request to the fault injector (ane/inject_fault)
  drive_stall

Every action is also published on ane/scenario_events (source SCRIPTED) so the recording shows when
it happened.

Times: *_sim fields are simulation time (the node clock), *_mono fields time.monotonic() of the container
(continuous; used for wall-clock durations), *_wall fields time.time() (informative only: the system
clock of a Docker Desktop VM is stepped by the host's time synchronisation).
"""

import json
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
from ane_interfaces.msg import BlockageReport, NavigatorEvent, NavigatorStatus
from ane_interfaces.srv import InjectFault, StartMission
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from scenario_lib import TERMINAL_STATES, load_scenario  # noqa: E402

PALLET_SDF = """<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <pose>0 0 {half} 0 0 0</pose>
        <geometry><box><size>{sx} {sy} {height}</size></box></geometry>
      </collision>
      <visual name="pallet">
        <pose>0 0 0.075 0 0 0</pose>
        <geometry><box><size>{sx} {sy} 0.15</size></box></geometry>
        <material><ambient>0.55 0.40 0.22 1</ambient><diffuse>0.55 0.40 0.22 1</diffuse></material>
      </visual>
      <visual name="load">
        <pose>0 0 {load_z} 0 0 0</pose>
        <geometry><box><size>{sx} {sy} {load_h}</size></box></geometry>
        <material><ambient>0.80 0.20 0.15 1</ambient><diffuse>0.80 0.20 0.15 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""


class ScenarioRunner(Node):
    def __init__(self):
        super().__init__("scenario_runner")
        self.scenario = load_scenario(self.declare_parameter("scenario_file", "").value)
        self.trial = int(self.declare_parameter("trial", 0).value)
        self.seed = int(self.declare_parameter("seed", 0).value)
        self.output_dir = Path(self.declare_parameter("output_dir", "/tmp/ane_trial").value)
        self.world = self.declare_parameter("world", "ane_warehouse").value
        self.settle = float(self.declare_parameter("settle_s", 2.0).value)
        self.initial_pose = [float(self.declare_parameter(k, 0.0).value) for k in ("x", "y", "yaw")]
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.gz = GzNode()  # Gazebo transport, for creating obstacles

        self.truth = None
        self.status = None
        self.phase = "WAIT"
        self.mission = {}
        self.records = []
        self.fired = [False] * len(self.scenario.events)
        self.requested_sim = None
        self.terminal = None
        self.wall_start = time.time()
        self.mono_start = time.monotonic()
        self.sequence = 0
        self.lock = threading.Lock()

        self.create_subscription(Odometry, "ground_truth/odom", self.on_truth, 50)
        self.create_subscription(NavigatorStatus, "ane/status", self.on_status, 20)
        reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.reports = self.create_publisher(BlockageReport, "ane/blockage_reports", reliable)
        self.events = self.create_publisher(NavigatorEvent, "ane/scenario_events", reliable)
        self.start_client = self.create_client(StartMission, "ane/start_mission")
        self.fault_client = self.create_client(InjectFault, "ane/inject_fault")
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(f"{self.scenario.name} trial {self.trial} seed {self.seed}")

    # -- inputs --------------------------------------------------------------------------------------
    def on_truth(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.truth = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def on_status(self, msg):
        self.status = msg

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # -- bookkeeping ---------------------------------------------------------------------------------
    def announce(self, kind, detail, value=0.0, nodes=()):
        msg = NavigatorEvent()
        msg.stamp = self.get_clock().now().to_msg()
        with self.lock:
            self.sequence += 1
            msg.sequence = self.sequence
        msg.wall_time_ns = time.time_ns()
        msg.type = kind
        msg.source = "SCRIPTED"
        msg.nodes = list(nodes)
        msg.value = float(value)
        msg.detail = detail
        self.events.publish(msg)

    def record(self, entry):
        with self.lock:
            self.records.append(entry)

    # -- actions -------------------------------------------------------------------------------------
    def spawn(self, event, entry):
        name, cx, cy, sx, sy, height = event.args[0], *(float(v) for v in event.args[1:])
        sdf = self.output_dir / f"{name}.sdf"
        sdf.write_text(PALLET_SDF.format(name=name, sx=sx, sy=sy, height=height, half=height / 2,
                                         load_z=0.15 + (height - 0.15) / 2, load_h=height - 0.15), encoding="utf-8")
        # Through the gz-transport bindings rather than the `gz service` command line, whose start-up alone
        # can take seconds on a loaded host and used to exhaust the timeout.
        request = EntityFactory()
        request.sdf_filename = str(sdf)
        request.name = name
        request.allow_renaming = False
        request.pose.position.x, request.pose.position.y, request.pose.position.z = cx, cy, 0.0
        sent, reply = self.gz.request(f"/world/{self.world}/create", request, EntityFactory, Boolean, 10000)
        entry.update({"confirmed_sim": self.sim_now(), "confirmed_mono": time.monotonic(), "confirmed_wall": time.time(),
                      "result": f"service reply: {reply.data}" if sent else "service call failed or timed out (10 s)",
                      "ok": bool(sent and reply.data),
                      "geometry": {"cx": cx, "cy": cy, "sx": sx, "sy": sy, "height": height}})
        self.announce("OBSTACLE_SPAWNED", json.dumps(entry["geometry"]), value=height)
        self.get_logger().info(f"spawned {name} at ({cx}, {cy}): {entry['result']}")

    def fire(self, event):
        entry = {"trigger": event.trigger, "trigger_value": event.value, "action": event.action, "args": event.args,
                 "fired_sim": self.sim_now(), "fired_mono": time.monotonic(), "fired_wall": time.time(),
                 "truth_at_fire": list(self.truth) if self.truth else None}
        self.record(entry)
        if event.action == "spawn_obstacle":
            threading.Thread(target=self.spawn, args=(event, entry), daemon=True).start()
        elif event.action == "close_region":
            label, x0, y0, x1, y1 = event.args[0], *(float(v) for v in event.args[1:])
            msg = BlockageReport()
            msg.stamp = self.get_clock().now().to_msg()
            msg.source = "OPERATOR"
            msg.label = label
            msg.use_region = True
            msg.min_x, msg.min_y, msg.max_x, msg.max_y = x0, y0, x1, y1
            self.reports.publish(msg)
            entry.update({"confirmed_sim": entry["fired_sim"], "confirmed_mono": entry["fired_mono"], "ok": True})
            self.announce("OPERATOR_CLOSURE", f"{label} {x0} {y0} {x1} {y1}")
        else:
            request = InjectFault.Request()
            request.fault = event.action
            request.duration_s = float(event.args[0])
            future = self.fault_client.call_async(request)

            def done(f, entry=entry):
                response = f.result()
                entry.update({"confirmed_sim": self.sim_now(), "confirmed_mono": time.monotonic(),
                              "ok": bool(response and response.accepted),
                              "result": response.message if response else "no response"})
            future.add_done_callback(done)
            self.announce("FAULT_REQUESTED", event.action, value=request.duration_s)

    # -- state machine -------------------------------------------------------------------------------
    def tick(self):
        now = self.sim_now()
        if self.phase == "WAIT":
            ready = (self.truth is not None and self.status is not None and self.status.mission_state == "IDLE"
                     and self.start_client.service_is_ready() and self.fault_client.service_is_ready() and now > 0.0)
            if ready:
                self.phase, self.ready_at = "SETTLE", now
            return
        if self.phase == "SETTLE":
            if now - self.ready_at >= self.settle:
                request = StartMission.Request()
                request.goal_station = self.scenario.goal
                self.requested_sim, self.requested_mono, self.requested_wall = now, time.monotonic(), time.time()
                self.start_future = self.start_client.call_async(request)
                self.phase = "STARTING"
            return
        if self.phase == "STARTING":
            if not self.start_future.done():
                return
            response = self.start_future.result()
            self.mission = {"requested_sim": self.requested_sim, "requested_mono": self.requested_mono,
                            "requested_wall": self.requested_wall,
                            "accepted": response.accepted, "message": response.message,
                            "start_node": response.start_node, "goal_node": response.goal_node,
                            "route": list(response.route), "route_cost": response.route_cost,
                            "truth_at_request": list(self.truth)}
            self.get_logger().info(f"mission: {response.message}")
            if not response.accepted:
                self.finish("START_REJECTED")
                return
            self.phase = "RUNNING"
        if self.phase in ("RUNNING", "OBSERVE"):
            x, y, _ = self.truth
            since = now - self.requested_sim
            for i, event in enumerate(self.scenario.events):
                if self.fired[i]:
                    continue
                due = {"at_start": True, "x_ge": x >= event.value, "x_le": x <= event.value,
                       "y_ge": y >= event.value, "y_le": y <= event.value, "time_ge": since >= event.value}[event.trigger]
                if due:
                    self.fired[i] = True
                    self.fire(event)
            state = self.status.mission_state
            if self.phase == "RUNNING" and state in TERMINAL_STATES:
                self.terminal = {"state": state, "sim": now, "mono": time.monotonic(), "wall": time.time()}
                self.phase = "OBSERVE"
                self.get_logger().info(f"terminal state {state} at {now:.3f} s; observing {self.scenario.observe} s")
            if self.phase == "OBSERVE":
                if now - self.terminal["sim"] >= self.scenario.observe:
                    self.finish(self.terminal["state"])
            elif since > self.scenario.timeout:
                self.finish("TIMEOUT")

    def finish(self, outcome):
        self.phase = "DONE"
        result = {
            "schema": "ane-sim/runner-result/1",
            "scenario": self.scenario.name,
            "scenario_title": self.scenario.title,
            "scenario_file": self.scenario.path.name,
            "scenario_sha256": self.scenario.sha256(),
            "trial": self.trial,
            "seed": self.seed,
            "initial_pose": dict(zip(("x", "y", "yaw"), self.initial_pose)),
            "mission": self.mission,
            "events": self.records,
            "runner_outcome": outcome,
            "terminal": self.terminal,
            "finished_sim": self.sim_now(),
            "finished_mono": time.monotonic(),
            "finished_wall": time.time(),
            "mono_started": self.mono_start,
            "wall_started": self.wall_start,
        }
        (self.output_dir / "runner_result.json").write_text(json.dumps(result, indent=1) + "\n", encoding="utf-8")
        self.get_logger().info(f"finished: {outcome}")
        self.timer.cancel()
        raise SystemExit(0)


def main():
    rclpy.init()
    node = ScenarioRunner()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
