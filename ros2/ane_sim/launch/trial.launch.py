"""One recorded trial: simulation + navigation + scenario runner + rosbag2 recorder. The launch shuts
down when the scenario runner exits (after the observation window, or on timeout).

  ros2 launch ane_sim trial.launch.py scenario:=<file> trial:=0 seed:=1000 x:=... y:=... yaw:=...
      output_dir:=<dir> bag_dir:=<dir> [gui:=true rviz:=true]

The recorder stamps messages on receipt with simulation time (--use-sim-time). Wall-clock receive times
would come from the container's system clock, which a Docker Desktop VM's time synchronisation steps
backwards by about a second every 30 s; ordered by such stamps, the bag would interleave messages
from before and after each step. Wall-clock durations are measured with monotonic clocks instead (the
scenario runner's *_mono fields, the engine-call timings).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TOPICS = [
    "/clock", "/tf", "/tf_static", "/robot_description", "/joint_states",
    "/ground_truth/odom", "/odom", "/wheel_odom", "/scan",
    "/cmd_vel_nav", "/cmd_vel", "/cmd_vel_robot",
    "/ane/status", "/ane/events", "/ane/fault_events", "/ane/scenario_events", "/ane/blockage_reports",
    "/ane/planned_route", "/ane/invalidated_route", "/ane/observed_trajectory",
    "/ane/graph_markers", "/ane/blocked_markers", "/ane/gate_state", "/ane/estop",
]


def setup(context):
    sim = Path(get_package_share_directory("ane_sim"))
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    include = lambda name, extra: IncludeLaunchDescription(  # noqa: E731
        PythonLaunchDescriptionSource(str(sim / "launch" / name)), launch_arguments=extra.items())
    runner = Node(
        package="ane_sim", executable="scenario_runner.py", output="screen",
        parameters=[{"use_sim_time": True, "scenario_file": arg("scenario"), "trial": int(arg("trial")),
                     "seed": int(arg("seed")), "output_dir": arg("output_dir"), "settle_s": float(arg("settle")),
                     "x": float(arg("x")), "y": float(arg("y")), "yaw": float(arg("yaw"))}])
    recorder = ExecuteProcess(cmd=["ros2", "bag", "record", "-s", "mcap", "--storage-preset-profile", "zstd_fast",
                                   "--use-sim-time", "-o", arg("bag_dir"), *TOPICS], output="log")
    return [
        include("sim.launch.py", {"gui": arg("gui"), "rviz": arg("rviz"), "seed": arg("seed"),
                                  "x": arg("x"), "y": arg("y"), "yaw": arg("yaw")}),
        include("navigation.launch.py", {}),
        recorder,
        runner,
        RegisterEventHandler(OnProcessExit(target_action=runner,
                                           on_exit=[EmitEvent(event=Shutdown(reason="scenario runner finished"))])),
    ]


def generate_launch_description():
    args = [("scenario", ""), ("trial", "0"), ("seed", "1"), ("x", "-6.5"), ("y", "0.0"), ("yaw", "0.0"),
            ("output_dir", "/tmp/ane_trial"), ("bag_dir", "/tmp/ane_trial_bag"), ("gui", "false"), ("rviz", "false"),
            ("settle", "2.0")]
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value) for name, value in args]
                             + [OpaqueFunction(function=setup)])
