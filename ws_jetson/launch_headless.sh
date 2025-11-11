#!/bin/bash
# Multi-Domain Vision Navigation Launcher (Headless Mode)
# Domain 6: Vision processing (camera_stream, lane_detection)
# Domain 5: Control interface (steering_control_domain5)

source ~/almondmatcha/ws_jetson/install/setup.bash

echo "========================================="
echo "Multi-Domain Vision Navigation System"
echo "========================================="
echo "Domain 6: Vision processing (isolated)"
echo "Domain 5: Control output (rover network)"
echo "========================================="

# Launch Domain 6 (vision processing) in background
echo "[Starting] Domain 6 vision processing..."
ros2 launch vision_navigation vision_domain6.launch.py &
VISION_PID=$!

# Wait for camera initialization
sleep 3

# Launch Domain 5 (control interface)
echo "[Starting] Domain 5 control interface..."
ros2 launch vision_navigation control_domain5.launch.py &
CONTROL_PID=$!

echo "========================================="
echo "Multi-Domain System Running"
echo "Domain 6 PID: $VISION_PID"
echo "Domain 5 PID: $CONTROL_PID"
echo "Press Ctrl+C to stop all nodes"
echo "========================================="

# Wait for both processes and cleanup on exit
trap "kill $VISION_PID $CONTROL_PID 2>/dev/null; exit" SIGINT SIGTERM
wait
