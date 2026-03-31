#!/bin/bash
# start_trace.sh — Start an LTTng trace session on a target machine
#
# Usage (from base PC, run against RPi or Jetson via SSH):
#   TARGET_HOST=pi@192.168.1.x TARGET_LABEL=rpi  bash start_trace.sh
#   TARGET_HOST=yupi@192.168.1.x TARGET_LABEL=jetson bash start_trace.sh
#
# Or run directly ON the target:
#   TARGET_LABEL=rpi bash start_trace.sh
#
# Collected tracepoints:
#   ros2:*               — rclcpp publish, callback_start/end, subscription callbacks
#   ros2_rmw:*           — RMW-layer publish/receive timestamps (DDS latency contribution)
#   lttng_ust_cyg_profile — (optional) function-level profiling, DISABLED by default
#
# Output: /tmp/ros2_trace_<LABEL>_<TIMESTAMP>/  (CTF format)
#         After stopping, use stop_and_collect_trace.sh to pull it back.

set -e

TARGET_HOST="${TARGET_HOST:-}"          # empty = run locally
TARGET_LABEL="${TARGET_LABEL:-unknown}"
SESSION_NAME="ros2_poc_${TARGET_LABEL}"
TRACE_DIR="\$HOME/ros2_traces/ros2_trace_${TARGET_LABEL}_$(date +%Y%m%d_%H%M%S)"

# Commands to execute (either locally or via SSH)
run_cmd() {
    if [ -n "$TARGET_HOST" ]; then
        ssh "$TARGET_HOST" "$@"
    else
        bash -c "$@"
    fi
}

echo "=== Starting LTTng trace session '${SESSION_NAME}' on ${TARGET_HOST:-localhost} ==="
echo "=== Trace output: ${TRACE_DIR} ==="

run_cmd "
set -e
mkdir -p \"\$HOME/ros2_traces\"
source /opt/ros/humble/setup.bash 2>/dev/null || true

# Destroy stale session if it exists
lttng destroy '${SESSION_NAME}' 2>/dev/null || true

# Create session with output directory
lttng create '${SESSION_NAME}' --output='${TRACE_DIR}'

# ── ROS2 userspace tracepoints ────────────────────────────────────────────────
lttng enable-event --userspace 'ros2:*'

# ── RMW (DDS) userspace tracepoints (available in rmw_fastrtps) ──────────────
lttng enable-event --userspace 'ros2_rmw:*' 2>/dev/null || \
    echo '[WARN] ros2_rmw tracepoints not available on this rmw implementation'

# ── Start the session ─────────────────────────────────────────────────────────
lttng start '${SESSION_NAME}'

echo '[OK] Trace session started. Run stop_and_collect_trace.sh when ready.'
echo '[OK] Trace dir: ${TRACE_DIR}'
"

# Save trace dir path locally for stop script to use
mkdir -p "$HOME/ros2_traces"
echo "${TRACE_DIR}" > "$HOME/ros2_traces/last_trace_dir_${TARGET_LABEL}.txt"
echo "Saved trace dir reference: $HOME/ros2_traces/last_trace_dir_${TARGET_LABEL}.txt"
