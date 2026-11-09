"""The ane_navigation nodes: graph_navigator (engine adapter), cmd_vel_gate (stop mechanism),
fault_injector (test instrument; transparent unless a fault is requested) and trajectory_path.

Data flow:
  /ground_truth/odom -> fault_injector -> /odom -> graph_navigator -> /cmd_vel_nav -> cmd_vel_gate
  -> /cmd_vel -> fault_injector -> /cmd_vel_robot -> ros_gz_bridge -> Gazebo DiffDrive
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = str(Path(get_package_share_directory("ane_navigation")) / "config" / "navigator.yaml")
    roadmap = str(Path(get_package_share_directory("ane_sim")) / "maps" / "warehouse_roadmap.txt")
    return LaunchDescription([
        Node(package="ane_navigation", executable="graph_navigator", output="log",
             parameters=[params, {"roadmap_file": roadmap, "use_sim_time": True}]),
        Node(package="ane_navigation", executable="cmd_vel_gate", output="log",
             parameters=[params, {"use_sim_time": False}]),
        Node(package="ane_navigation", executable="fault_injector", output="log",
             parameters=[{"use_sim_time": True}]),
        Node(package="ane_sim", executable="trajectory_path.py", output="log",
             parameters=[{"use_sim_time": True}]),
    ])
