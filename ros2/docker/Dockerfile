# Simulation environment for the ROS 2 integration of the Autonomous Navigation Engine.
#
# ROS 2 Jazzy Jalisco on Ubuntu 24.04 with Gazebo Harmonic, the combination the Gazebo documentation
# recommends for Jazzy (https://gazebosim.org/docs/latest/ros_installation/). As of Jazzy, Gazebo is
# installed from packages.ros.org as ROS vendor packages through ros-jazzy-ros-gz.
#
# Rendering is software (Mesa llvmpipe), headless through EGL for the lidar sensor, and through Xvfb
# for the Gazebo GUI and RViz2 when a run is recorded. No GPU is required.
#
# Build (from the repository root):
#   docker build -t ane-sim:jazzy ros2/docker
# The exact package versions of a build are written to /opt/ane/versions.txt inside the image.

# ros:jazzy-ros-base, linux/amd64 manifest published 2026-10-27 (Docker Hub official image).
FROM ros:jazzy-ros-base@sha256:c26c86a28e1addf9d10a81232869791974a3f7907943d5634742b58b2f26e1a3

LABEL org.opencontainers.image.title="ane-sim" \
      org.opencontainers.image.description="ROS 2 Jazzy + Gazebo Harmonic environment for the Autonomous Navigation Engine simulation trials"

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      build-essential cmake python3-colcon-common-extensions \
      ros-jazzy-ros-gz \
      ros-jazzy-rviz2 \
      ros-jazzy-xacro \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-rosbag2-storage-mcap \
      ros-jazzy-tf2-ros \
      liburdfdom-tools \
      xvfb x11-utils xdotool \
      ffmpeg \
      mesa-utils libgl1-mesa-dri libegl1 libegl-mesa0 \
      python3-yaml python3-numpy python3-matplotlib python3-pil \
 && rm -rf /var/lib/apt/lists/*

# Record what this build installed, so trial manifests can cite exact versions.
RUN mkdir -p /opt/ane \
 && { echo "image-built-utc $(date -u +%Y-%m-%dT%H:%M:%SZ)"; \
      grep -E '^(PRETTY_NAME|VERSION_ID)=' /etc/os-release; \
      dpkg-query -W -f='${Package} ${Version}\n' \
        'ros-jazzy-ros-base' 'ros-jazzy-rclcpp' 'ros-jazzy-ros-gz' 'ros-jazzy-ros-gz-sim' 'ros-jazzy-ros-gz-bridge' \
        'ros-jazzy-gz-sim-vendor' 'ros-jazzy-gz-physics-vendor' 'ros-jazzy-gz-sensors-vendor' \
        'ros-jazzy-gz-rendering-vendor' 'ros-jazzy-gz-transport-vendor' 'ros-jazzy-gz-tools-vendor' \
        'ros-jazzy-sdformat-vendor' 'ros-jazzy-rviz2' 'ros-jazzy-xacro' 'ros-jazzy-robot-state-publisher' \
        'ros-jazzy-rosbag2' 'ros-jazzy-rosbag2-storage-mcap' 'ros-jazzy-tf2-ros' \
        'cmake' 'g++' 'libgl1-mesa-dri' 'xvfb' 'ffmpeg' 'python3' 2>/dev/null; \
    } > /opt/ane/versions.txt \
 && cat /opt/ane/versions.txt

WORKDIR /ws
