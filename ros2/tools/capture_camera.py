#!/usr/bin/env python3
"""Steers the Gazebo GUI camera during a recorded run: lane views that follow the robot, and an
overview after chosen navigator events.

  python3 capture_camera.py --log <camera_log.csv> [--overview ROUTE_REPLACED 6] [--overview ROUTE_BLOCKED inf]

Lane views. The camera follows the robot from behind and above, looking along the lane the robot is
travelling: east for the east-west aisles and spurs, south or north in the cross aisles. The view
changes only after the robot has moved in a new direction for a moment, so turning in place never
swings the camera; while the robot backs away from an obstacle, the camera keeps looking east and the
obstacle stays in view. The camera stays inside the hall.

Overview. When the navigator publishes one of the given events on ane/events (for example the engine's
route replacement), the camera moves to a steep view of the middle and south aisles for the given number
of seconds (inf: until the end), so both the blocked aisle and the alternative are visible.

The robot pose is the simulator's ground truth (OdometryPublisher topic); camera moves are smoothed and
sent to the GUI's /gui/move_to/pose ten times per second. Every pose sent is written to the log with the
simulation time of the robot pose it came from and the view in use. This moves only the GUI's viewpoint;
it does not touch the simulation or the navigator. Needs the gz-transport and gz-msgs Python bindings,
rclpy with ane_interfaces (the ane-sim image and workspace) and the CameraTracking GUI plugin.
"""

import argparse
import math
import threading
import time

import rclpy
from ane_interfaces.msg import NavigatorEvent
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.gui_camera_pb2 import GUICamera
from gz.msgs10.odometry_pb2 import Odometry
from gz.transport13 import Node

HALL = (-7.75, -4.75, 7.75, 4.75)        # camera positions are kept inside the walls (map frame, m)
# Lane views: eye and look-at point relative to the robot (world frame, m).
LANE_VIEWS = {
    "east": ((-3.6, -0.25, 2.8), (0.8, 0.0, 0.25)),
    "south": ((0.25, 3.3, 2.8), (0.0, -0.8, 0.25)),
    "north": ((-0.25, -3.0, 2.8), (0.0, 0.8, 0.25)),
}
OVERVIEW = ((-0.8, -4.5, 11.5), (-0.8, -1.4, 0.0))  # fixed: the middle and south aisles from above


def look_at_quaternion(eye, target):
    """(w, x, y, z) of a camera at eye looking at target, x forward and z up (Gazebo's convention)."""
    dx, dy, dz = (t - e for t, e in zip(target, eye))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))  # positive pitch looks down
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return cy * cp, -sy * sp, cy * sp, sy * cp


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--model", default="ane_amr")
    parser.add_argument("--overview", nargs=2, action="append", default=[], metavar=("EVENT", "SECONDS"))
    parser.add_argument("--follow", type=float, default=0.35, help="robot-position smoothing (s, wall clock)")
    parser.add_argument("--blend", type=float, default=1.0, help="view-change smoothing (s, wall clock)")
    parser.add_argument("--rate", type=float, default=10.0, help="camera updates per second (wall clock)")
    parser.add_argument("--log", required=True)
    args = parser.parse_args()
    overview_events = {event: float(seconds) for event, seconds in args.overview}

    state = {"t": None, "xy": None, "overview_until": -1.0}
    lock = threading.Lock()

    def on_odometry(msg):
        with lock:
            state["t"] = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            state["xy"] = (msg.pose.position.x, msg.pose.position.y)

    def on_event(msg):
        if msg.type in overview_events:
            with lock:
                state["overview_until"] = time.monotonic() + overview_events[msg.type]

    gz = Node()
    gz.subscribe(Odometry, f"/model/{args.model}/ground_truth/odometry", on_odometry)
    rclpy.init()
    ros = rclpy.create_node("capture_camera")
    ros.create_subscription(NavigatorEvent, "ane/events", on_event, 50)
    threading.Thread(target=rclpy.spin, args=(ros,), daemon=True).start()

    period = 1.0 / args.rate
    follow = 1.0 - math.exp(-period / args.follow)
    blend = 1.0 - math.exp(-period / args.blend)
    view, candidate, candidate_since = "east", None, 0.0
    history = []                      # (monotonic, x, y) over the last second, for the travel direction
    position = offsets = None         # smoothed robot position; blended lane-view offsets
    weight = 0.0                      # blend towards the overview (0: lane view, 1: overview)
    with open(args.log, "w") as log:
        log.write("sim_s,view,robot_x,robot_y,cam_x,cam_y,cam_z,look_x,look_y,look_z,qw,qx,qy,qz\n")
        next_tick = time.monotonic()
        while True:
            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))
            now = time.monotonic()
            with lock:
                if state["xy"] is None:
                    continue
                t, (x, y), overview_until = state["t"], state["xy"], state["overview_until"]
            history = [h for h in history if now - h[0] <= 1.0] + [(now, x, y)]
            vx, vy = (history[-1][1] - history[0][1], history[-1][2] - history[0][2])
            moving = math.hypot(vx, vy) > 0.12 * max(history[-1][0] - history[0][0], 1e-3)
            if moving:
                direction = "east" if abs(vx) >= abs(vy) else ("south" if vy < 0 else "north")
                if direction != view:
                    if direction != candidate:
                        candidate, candidate_since = direction, now
                    elif now - candidate_since >= 0.6:
                        view, candidate = direction, None
                else:
                    candidate = None
            # Follow the robot quickly; change views and switch to or from the overview slowly.
            if position is None:
                position, offsets = [x, y], [*LANE_VIEWS[view][0], *LANE_VIEWS[view][1]]
            position = [position[0] + follow * (x - position[0]), position[1] + follow * (y - position[1])]
            target = [*LANE_VIEWS[view][0], *LANE_VIEWS[view][1]]
            offsets = [o + blend * (g - o) for o, g in zip(offsets, target)]
            overview = now < overview_until
            weight += blend * ((1.0 if overview else 0.0) - weight)
            name = "overview" if overview else view
            lane_eye = (min(max(position[0] + offsets[0], HALL[0]), HALL[2]),
                        min(max(position[1] + offsets[1], HALL[1]), HALL[3]), offsets[2])
            lane_look = (position[0] + offsets[3], position[1] + offsets[4], offsets[5])
            eye = [a + weight * (b - a) for a, b in zip(lane_eye, OVERVIEW[0])]
            look = [a + weight * (b - a) for a, b in zip(lane_look, OVERVIEW[1])]
            qw, qx, qy, qz = look_at_quaternion(eye, look)
            request = GUICamera()
            request.pose.position.x, request.pose.position.y, request.pose.position.z = eye
            request.pose.orientation.w, request.pose.orientation.x = qw, qx
            request.pose.orientation.y, request.pose.orientation.z = qy, qz
            gz.request("/gui/move_to/pose", request, GUICamera, Boolean, 500)
            log.write(f"{t:.3f},{name},{x:.4f},{y:.4f},{eye[0]:.4f},{eye[1]:.4f},{eye[2]:.4f},"
                      f"{look[0]:.4f},{look[1]:.4f},{look[2]:.4f},{qw:.6f},{qx:.6f},{qy:.6f},{qz:.6f}\n")
            log.flush()


if __name__ == "__main__":
    main()
