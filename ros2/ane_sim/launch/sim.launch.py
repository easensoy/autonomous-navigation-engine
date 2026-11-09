"""Gazebo Harmonic warehouse with ane_amr, the ROS-Gazebo bridge, robot_state_publisher and the
static map -> odom transform; optionally the Gazebo GUI and RViz2.

  ros2 launch ane_sim sim.launch.py [gui:=true] [rviz:=true] [x:=-6.5 y:=0 yaw:=0] [seed:=1]

map -> odom is the identity: the ground-truth odometry frame of the OdometryPublisher coincides with the
Gazebo world frame, which is the map frame.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    sim = Path(get_package_share_directory("ane_sim"))
    description = Path(get_package_share_directory("ane_description"))
    world = sim / "worlds" / "warehouse.sdf"
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    seed = LaunchConfiguration("seed").perform(context)
    verbosity = LaunchConfiguration("gz_verbosity").perform(context)
    if gui:
        gz_args = f"-r -v {verbosity} --seed {seed} --gui-config {sim / 'config' / 'gui.config'} {world}"
    else:
        gz_args = f"-r -s --headless-rendering -v {verbosity} --seed {seed} {world}"

    robot_description = ParameterValue(Command(["xacro ", str(description / "urdf" / "ane_amr.urdf.xacro"), " sim:=true"]),
                                       value_type=str)
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items()),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="log",
             parameters=[{"robot_description": robot_description, "use_sim_time": True}]),
        Node(package="ros_gz_sim", executable="create", output="log",
             arguments=["-world", "ane_warehouse", "-topic", "robot_description", "-name", "ane_amr",
                        "-x", LaunchConfiguration("x"), "-y", LaunchConfiguration("y"), "-z", "0.0",
                        "-Y", LaunchConfiguration("yaw")]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="log",
             parameters=[{"config_file": str(sim / "config" / "bridge.yaml")}]),
        Node(package="tf2_ros", executable="static_transform_publisher", output="log",
             arguments=["--frame-id", "map", "--child-frame-id", "odom"]),
    ]
    if LaunchConfiguration("rviz").perform(context).lower() == "true":
        actions.append(Node(package="rviz2", executable="rviz2", output="log",
                            arguments=["-d", str(sim / "config" / "ane.rviz")],
                            parameters=[{"use_sim_time": True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false", description="run the Gazebo GUI (needs a display)"),
        DeclareLaunchArgument("rviz", default_value="false", description="run RViz2 (needs a display)"),
        DeclareLaunchArgument("seed", default_value="1", description="Gazebo random seed (sensor noise)"),
        DeclareLaunchArgument("gz_verbosity", default_value="2"),
        DeclareLaunchArgument("x", default_value="-6.5"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        OpaqueFunction(function=setup),
    ])
