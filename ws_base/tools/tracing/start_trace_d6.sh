#!/bin/bash
# start_trace_d6.sh — Start collect_latency.py for D6 (vision) topics on the Jetson.
#
# Multi-domain POC only. D6 is Jetson-localhost shared-memory domain; these topics
# are invisible to D5 subscribers. This script runs a second collector instance
# alongside the regular D5 collector (started by start_trace.sh).
#
# Usage (from base PC via launch_poc_experiment.sh):
#   TARGET_HOST=yupi@192.168.1.5 TARGET_LABEL=jetson_d6 SSH_OPTS="..." bash start_trace_d6.sh
#
# Output: ~/ros2_traces/latency_jetson_d6.csv  (on Jetson)
# PID:    ~/ros2_traces/collector_jetson_d6.pid
# Log:    ~/ros2_traces/collector_jetson_d6.log
#
# Stop and pull with stop_and_collect_trace.sh:
#   LOCAL_DEST_CSV=".../latency_jetson_d6.csv" TARGET_HOST=yupi@192.168.1.5 \
#       TARGET_LABEL=jetson_d6 bash stop_and_collect_trace.sh

set -e

TARGET_HOST="${TARGET_HOST:-}"
TARGET_LABEL="${TARGET_LABEL:-jetson_d6}"    # must stay jetson_d6 — drives PID/CSV names
SSH_OPTS="${SSH_OPTS:-}"

OUT_CSV="\$HOME/ros2_traces/latency_${TARGET_LABEL}.csv"
PID_FILE="\$HOME/ros2_traces/collector_${TARGET_LABEL}.pid"
LOG_FILE="\$HOME/ros2_traces/collector_${TARGET_LABEL}.log"

# D6 vision topics only. These are published on ROS_DOMAIN_ID=6 via Jetson shared memory.
# /tpc_rover_d415_depth is included but will be skipped silently if not published.
D6_TOPICS="/tpc_rover_d415_rgb /tpc_rover_d415_depth /tpc_rover_nav_lane"

run_remote() {
    if [ -n "$TARGET_HOST" ]; then
        ssh $SSH_OPTS "$TARGET_HOST" "$@"
    else
        bash -c "$@"
    fi
}

echo "=== Starting D6 latency collector on ${TARGET_HOST:-localhost} (label=${TARGET_LABEL}) ==="

run_remote "
set -e
source /opt/ros/humble/setup.bash
source ~/almondmatcha/ws_jetson/install/setup.bash 2>/dev/null || true
source ~/almondmatcha/common_ifaces/install/setup.bash 2>/dev/null || true

# D6: shared memory, Jetson localhost only.
# Do NOT set FASTRTPS_DEFAULT_PROFILES_FILE — the XML whitelists 192.168.1.5
# which disables shared-memory transport, causing D6 topics to never be seen.
export ROS_DOMAIN_ID=6
unset FASTRTPS_DEFAULT_PROFILES_FILE

mkdir -p ~/ros2_traces

# Kill any stale D6 collector
if [ -f ${PID_FILE} ]; then
    OLD_PID=\$(cat ${PID_FILE})
    kill \"\$OLD_PID\" 2>/dev/null || true
fi

nohup python3 ~/almondmatcha/ws_base/tools/tracing/collect_latency.py \
    --topics ${D6_TOPICS} \
    --out ${OUT_CSV} \
    < /dev/null > ${LOG_FILE} 2>&1 &
echo \$! > ${PID_FILE}

echo '[OK] D6 collector PID: '\$(cat ${PID_FILE})
echo '[OK] Output CSV:       ${OUT_CSV}'
echo '[OK] Log:              ${LOG_FILE}'
"
echo "=== D6 collector started. Topics will appear once vision nodes are up. ==="
echo "    Run: stop_and_collect_trace.sh (TARGET_LABEL=jetson_d6) when ready to stop."
