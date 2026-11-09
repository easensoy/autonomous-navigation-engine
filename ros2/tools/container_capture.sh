#!/usr/bin/env bash
# Records one trial with the Gazebo GUI and RViz2 side by side on a virtual display and analyses it.
#   container_capture.sh <scenario file> <trial> <seed> <x> <y> <yaw> <output dir> <capture dir>
# The capture dir receives the bag, the screen frames (JPEG, up to 10 per second) with frames.csv (each
# frame's monotonic time), capture_clock.csv (monotonic time -> simulation time, 10 Hz), camera_log.csv
# (every camera pose sent to the Gazebo GUI, with the simulation time of the robot pose it followed),
# windows.txt (window geometry, for cropping) and the logs.
set -o pipefail
SCENARIO=$1; TRIAL=$2; SEED=$3; X=$4; Y=$5; YAW=$6; OUT=$7; CAP=$8
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
export GZ_IP=127.0.0.1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export DISPLAY=:99 QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1
rm -rf "$CAP"
rm -f "$OUT"/runner_result.json "$OUT"/metrics.json "$OUT"/trajectory.json "$OUT"/*.sdf
mkdir -p "$OUT" "$CAP/frames"
export ROS_LOG_DIR="$CAP/ros_log"
# Gazebo (1280 x 768: a 48 px title bar over a 1280 x 720 scene) and RViz2 (800 x 600) side by side.
Xvfb :99 -screen 0 2080x768x24 -nolisten tcp > "$CAP/xvfb.log" 2>&1 &
sleep 2
# A long settle time keeps the robot still until the windows are placed and the recording runs.
ros2 launch ane_sim trial.launch.py scenario:="$SCENARIO" trial:="$TRIAL" seed:="$SEED" x:="$X" y:="$Y" yaw:="$YAW" \
  output_dir:="$OUT" bag_dir:="$CAP/bag" gui:=true rviz:=true settle:=25.0 > "$CAP/launch.log" 2>&1 &
LAUNCH=$!
for i in $(seq 1 120); do
  GZW=$(xdotool search --name "Gazebo Sim" 2>/dev/null | head -1)
  RVW=$(xdotool search --name "RViz" 2>/dev/null | head -1)
  [ -n "$GZW" ] && [ -n "$RVW" ] && break
  sleep 1
done
echo "windows: gazebo=$GZW rviz=$RVW after ${i} s"
xdotool windowsize "$GZW" 1280 768 windowmove "$GZW" 0 0
xdotool windowsize "$RVW" 800 600 windowmove "$RVW" 1280 0
xdotool mousemove 2079 767
sleep 3
{ echo "gazebo $(xdotool getwindowgeometry --shell "$GZW" | tr '\n' ' ')";
  echo "rviz $(xdotool getwindowgeometry --shell "$RVW" | tr '\n' ' ')"; } > "$CAP/windows.txt"
# Camera (ros2/tools/capture_camera.py): follows the robot along its lane; after the engine replaces the
# route (scenario B) or finds none (scenario C), an overview of the middle and south aisles shows the
# blocked aisle and the alternative.
case "$(basename "$SCENARIO")" in
  B_*) OVERVIEW=(--overview ROUTE_REPLACED 6) ;;
  C_*) OVERVIEW=(--overview ROUTE_BLOCKED inf) ;;
  *)   OVERVIEW=() ;;
esac
python3 /ane/ros2/tools/capture_camera.py --log "$CAP/camera_log.csv" "${OVERVIEW[@]}" > "$CAP/camera.log" 2>&1 &
CAMERA=$!
python3 - "$CAP/capture_clock.csv" > /dev/null 2>&1 <<'EOF' &
import sys, time
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
rclpy.init()
node = Node("capture_clock")
latest = [None]
node.create_subscription(Clock, "/clock", lambda m: latest.__setitem__(0, m.clock.sec + m.clock.nanosec * 1e-9), 10)
with open(sys.argv[1], "w") as out:
    out.write("monotonic_s,sim_s\n")
    next_tick = time.monotonic()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        if time.monotonic() >= next_tick and latest[0] is not None:
            out.write(f"{time.monotonic():.4f},{latest[0]:.4f}\n")
            out.flush()
            next_tick += 0.1
EOF
CLOCK=$!
# Screen frames, each stamped with the monotonic clock (ros2/tools/capture_frames.py); capture_clock.csv
# maps that clock to simulation time.
python3 /ane/ros2/tools/capture_frames.py --display :99 --out "$CAP" --fps 10 > "$CAP/frames.log" 2>&1 &
GRAB=$!
wait $LAUNCH
code=$?
kill -TERM $GRAB; wait $GRAB 2>/dev/null
kill $CLOCK $CAMERA 2>/dev/null
pkill -f "gz sim" 2>/dev/null; sleep 1; pkill -9 -f "gz sim" 2>/dev/null
pkill -f rviz2 2>/dev/null
echo "launch exit $code; $(ls "$CAP/frames" | wc -l) frames"
# Published with the trial: every camera move, with the simulation time and the view it served.
cp "$CAP/camera_log.csv" "$OUT/camera_log.csv" 2>/dev/null
[ -f "$OUT/runner_result.json" ] || { echo "no runner_result.json: infrastructure failure"; exit 3; }
python3 /ane/ros2/ane_sim/scripts/analyze_trial.py "$OUT" "$CAP/bag" \
  --route-check /ws/install/ane_navigation/lib/ane_navigation/ane_route_check
