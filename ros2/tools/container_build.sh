#!/usr/bin/env bash
# Builds and tests the ROS 2 workspace inside the ane-sim container.
#   /ane  the repository (bind mount)
#   /ws   the colcon workspace: build/, install/, log/ (a named Docker volume)
# Usage (from the repository root on the host):
#   docker run --rm -v "$PWD:/ane" -v ane-ws:/ws ane-sim:jazzy bash /ane/ros2/tools/container_build.sh [--test]
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd /ws
colcon build --base-paths /ane/ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  --event-handlers console_direct- console_stderr+ desktop_notification-
echo "colcon build: ok"
if [ "${1:-}" = "--test" ]; then
  source /ws/install/setup.bash
  colcon test --base-paths /ane/ros2 --event-handlers console_direct- desktop_notification- || true
  colcon test-result --verbose
fi
