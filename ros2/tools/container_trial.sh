#!/usr/bin/env bash
# Runs one trial inside a fresh ane-sim container and analyses it. Called by run_trials.py; usage:
#   container_trial.sh <scenario file> <trial> <seed> <x> <y> <yaw> <output dir> <bag dir> [gui]
# Paths are container paths (/ane is the repository, /ws the built workspace).
set -o pipefail
SCENARIO=$1; TRIAL=$2; SEED=$3; X=$4; Y=$5; YAW=$6; OUT=$7; BAG=$8; GUI=${9:-false}
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
# Keep Gazebo transport and DDS discovery inside this container.
export GZ_IP=127.0.0.1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
rm -rf "$BAG"
rm -f "$OUT"/runner_result.json "$OUT"/metrics.json "$OUT"/trajectory.json "$OUT"/*.sdf  # keep manifest.json
mkdir -p "$OUT" "$(dirname "$BAG")"
export ROS_LOG_DIR="$(dirname "$BAG")/ros_log_trial_$TRIAL"
start=$(date +%s)
ros2 launch ane_sim trial.launch.py scenario:="$SCENARIO" trial:="$TRIAL" seed:="$SEED" x:="$X" y:="$Y" yaw:="$YAW" \
  output_dir:="$OUT" bag_dir:="$BAG" gui:="$GUI" rviz:="$GUI" > "$(dirname "$BAG")/launch_trial_$TRIAL.log" 2>&1
code=$?
# ros2 launch stops the `ruby gz sim` wrapper, but the simulation server it started can survive it;
# end everything so nothing outlives the trial.
pkill -f "gz sim" 2>/dev/null; sleep 1; pkill -9 -f "gz sim" 2>/dev/null
echo "launch exit $code after $(( $(date +%s) - start )) s"
if [ ! -f "$OUT/runner_result.json" ]; then
  echo "no runner_result.json: infrastructure failure"
  exit 3
fi
python3 /ane/ros2/ane_sim/scripts/analyze_trial.py "$OUT" "$BAG" \
  --route-check /ws/install/ane_navigation/lib/ane_navigation/ane_route_check
