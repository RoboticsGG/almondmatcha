#!/bin/bash
# setup_tracing.sh — Pre-flight environment check for collect_latency.py
#
# The measurement tool (collect_latency.py) uses rclpy — no LTTng installation needed.
# NOTE: ros-humble-rclcpp on arm64 apt is compiled with TRACETOOLS_DISABLED=1,
#       so LTTng ros2:* tracepoints are not available. We use rclpy subscriber-based
#       latency/jitter collection instead.
#
# Run on each SBC to verify the environment is ready:
#   ssh curry@192.168.1.1   'bash ~/almondmatcha/ws_base/tools/tracing/setup_tracing.sh'
#   ssh yupi@192.168.1.5    'bash ~/almondmatcha/ws_base/tools/tracing/setup_tracing.sh'

set -e

ROS_DISTRO="${ROS_DISTRO:-humble}"
PASS=0
FAIL=0

check() {
    local desc="$1"
    local cmd="$2"
    if eval "$cmd" > /dev/null 2>&1; then
        echo "  [OK]   $desc"
        PASS=$((PASS+1))
    else
        echo "  [FAIL] $desc"
        FAIL=$((FAIL+1))
    fi
}

echo "=== Pre-flight check for collect_latency.py ==="
echo "    Host: $(hostname)  Kernel: $(uname -r)"
echo ""

# ── ROS2 ─────────────────────────────────────────────────────────────────
check "ROS2 Humble installed" \
    "[ -d /opt/ros/humble ]"

check "ROS2 sourced (rclpy importable)" \
    "source /opt/ros/humble/setup.bash && python3 -c 'import rclpy'"

check "rosidl_runtime_py importable" \
    "source /opt/ros/humble/setup.bash && python3 -c 'from rosidl_runtime_py.utilities import get_message'"

# ── Workspaces ────────────────────────────────────────────────────────────
# ws_rpi has msgs_ifaces symlinked into src/ and built into ws_rpi/install/.
# ws_jetson does NOT include interface packages in its src/ — it loads them
# from common_ifaces/install/ which is pre-built in the repo.
# Either way, the 'custom messages loadable' check below is the real gate.
check "ws_rpi built  (or ws_jetson)" \
    "[ -d ~/almondmatcha/ws_rpi/install/rover_monitoring ] || [ -d ~/almondmatcha/ws_jetson/install/vision_navigation ]"

# ── Custom messages loadable ──────────────────────────────────────────────
# collect_latency.py needs to dynamically load ChassisIMU, TelemetryRelay, etc.
check "custom messages loadable by rclpy" \
    "source /opt/ros/humble/setup.bash && \
     (source ~/almondmatcha/ws_rpi/install/setup.bash 2>/dev/null || \
      source ~/almondmatcha/ws_jetson/install/setup.bash 2>/dev/null) && \
     python3 -c 'from rosidl_runtime_py.utilities import get_message; get_message(\"msgs_ifaces/msg/ChassisIMU\")'"

# ── collect_latency.py ────────────────────────────────────────────────────
check "collect_latency.py present" \
    "[ -f ~/almondmatcha/ws_base/tools/tracing/collect_latency.py ]"

# ── Output dir ────────────────────────────────────────────────────────────
mkdir -p ~/ros2_traces
check "~/ros2_traces/ writable" \
    "touch ~/ros2_traces/.test && rm ~/ros2_traces/.test"

echo ""
if [ "$FAIL" -eq 0 ]; then
    echo "=== All checks passed. Ready to collect latency data. ==="
else
    echo "=== $FAIL check(s) FAILED — fix before running the experiment ==="
    exit 1
fi
