#!/bin/bash
# Multi-Domain Vision Navigation Launcher (GUI Mode)
# Domain 6: Vision processing (camera_stream, lane_detection)
# Domain 5: Control interface (rover_kinematic_control)

source /opt/ros/humble/setup.bash
source ~/almondmatcha/common_ifaces/install/setup.bash
source ~/almondmatcha/ws_jetson/install/setup.bash

# ── One run directory, shared by every logging node ───────────────────────────
# Each logging node is a separate process, so if they each allocated their own
# run number and timestamp a single launch would scatter output across several
# run_NNN_* directories (and a stray runs/logs/). Allocate it once here and
# export it; every node prefers $ROVER_RUN_DIR over computing its own.
#
# Deliberately NOT created here — the nodes create it on their first actual
# write, so a launch that logs nothing leaves no empty directory behind.
RUNS_DIR="$HOME/almondmatcha/ws_jetson/runs"
_last=$(ls -d "$RUNS_DIR"/run_[0-9][0-9][0-9]_* 2>/dev/null \
        | sed -n 's#.*/run_\([0-9][0-9][0-9]\)_.*#\1#p' | sort -n | tail -1)
_next=$(( 10#${_last:-000} + 1 ))
export ROVER_RUN_DIR="$RUNS_DIR/$(printf 'run_%03d_%s' "$_next" "$(date +%Y%m%d_%H%M%S)")"
echo "[run] output directory for this launch: $ROVER_RUN_DIR"

echo "========================================="
echo "Multi-Domain Vision Navigation System (GUI)"
echo "========================================="
echo "Domain 6: Vision processing (isolated)"
echo "Domain 5: Control output (rover network)"
echo "========================================="

# Launch Domain 6 (vision processing with GUI) in background
echo "[Starting] Domain 6 vision processing with GUI..."
ros2 launch vision_navigation vision_nav_gui.launch.py &
VISION_PID=$!

# Wait for camera initialization
sleep 3

# Launch Domain 5 (control interface)
echo "[Starting] Domain 5 control interface..."
ros2 launch vision_navigation control_domain5.launch.py &
CONTROL_PID=$!

echo "========================================="
echo "Multi-Domain System Running"
echo "Domain 6 PID: $VISION_PID (with GUI)"
echo "Domain 5 PID: $CONTROL_PID"
echo "Press Ctrl+C to stop all nodes"
echo "========================================="

# Wait for both processes and cleanup on exit
trap "kill $VISION_PID $CONTROL_PID 2>/dev/null; exit" SIGINT SIGTERM
wait
