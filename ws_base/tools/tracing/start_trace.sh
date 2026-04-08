#!/bin/bash
# start_trace.sh — Start collect_latency.py on a target SBC in background.
#
# Usage (from base PC):
#   TARGET_HOST=curry@192.168.1.1 TARGET_LABEL=rpi   bash start_trace.sh
#   TARGET_HOST=yupi@192.168.1.5  TARGET_LABEL=jetson bash start_trace.sh
#
# Or run directly on the SBC:
#   TARGET_LABEL=rpi bash start_trace.sh
#
# The collector writes to ~/ros2_traces/latency_<LABEL>.csv on the SBC.
# Use stop_and_collect_trace.sh to stop it and pull the CSV to the base PC.

set -e

TARGET_HOST="${TARGET_HOST:-}"          # empty = run locally
TARGET_LABEL="${TARGET_LABEL:-unknown}"
SSH_OPTS="${SSH_OPTS:-}"                 # ControlMaster opts passed from launch_poc_experiment.sh
OUT_CSV="\$HOME/ros2_traces/latency_${TARGET_LABEL}.csv"
PID_FILE="\$HOME/ros2_traces/collector_${TARGET_LABEL}.pid"
LOG_FILE="\$HOME/ros2_traces/collector_${TARGET_LABEL}.log"

# All D5 topics for this project
# Camera/vision topics (/tpc_rover_d415_rgb, /tpc_rover_d415_depth, /tpc_rover_nav_lane)
# are included so the Jetson collector captures camera frame jitter and latency.
# If these topics are not present on the DDS bus (e.g. vision nodes not launched),
# collect_latency.py simply skips them — no error.
TOPICS="/tpc_chassis_imu /tpc_chassis_sensors /tpc_chassis_cmd /tpc_gnss_spresense /tpc_gnss_ublox /tpc_rover_ctrl_cmd /tpc_telemetry_relay /tpc_rover_d415_rgb /tpc_rover_d415_depth /tpc_rover_nav_lane"

run_remote() {
    if [ -n "$TARGET_HOST" ]; then
        ssh $SSH_OPTS "$TARGET_HOST" "$@"
    else
        bash -c "$@"
    fi
}

echo "=== Starting latency collector on ${TARGET_HOST:-localhost} (label=${TARGET_LABEL}) ==="

run_remote "
set -e
source /opt/ros/humble/setup.bash
source ~/almondmatcha/ws_rpi/install/setup.bash 2>/dev/null || \
    source ~/almondmatcha/ws_jetson/install/setup.bash 2>/dev/null || true
# common_ifaces provides msgs_ifaces type support (ChassisIMU, SpresenseGNSS, etc.)
# Without this, collect_latency.py cannot deserialise custom types and skips them.
source ~/almondmatcha/common_ifaces/install/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=5

# Pin Fast-DDS to the correct NIC for this host.
# Use TARGET_LABEL (set by the caller) — NOT file existence, because both
# ws_rpi/ and ws_jetson/ exist on every machine (same git repo), so a
# file-existence check always picks ws_rpi/fastdds_rover.xml first, which
# whitelists 192.168.1.1 (RPi IP) and breaks DDS transport on the Jetson.
if [ "${TARGET_LABEL}" = "rpi" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=~/almondmatcha/ws_rpi/fastdds_rover.xml
elif [ "${TARGET_LABEL}" = "jetson" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=~/almondmatcha/ws_jetson/fastdds_jetson.xml
fi

mkdir -p ~/ros2_traces

# Kill any stale collector for this label
if [ -f ${PID_FILE} ]; then
    OLD_PID=\$(cat ${PID_FILE})
    kill \"\$OLD_PID\" 2>/dev/null || true
fi

nohup python3 ~/almondmatcha/ws_base/tools/tracing/collect_latency.py \
    --topics ${TOPICS} \
    --out ${OUT_CSV} \
    < /dev/null > ${LOG_FILE} 2>&1 &
echo \$! > ${PID_FILE}

echo '[OK] Collector PID: '\$(cat ${PID_FILE})
echo '[OK] Output CSV:    ${OUT_CSV}'
echo '[OK] Log:           ${LOG_FILE}'
"
echo "=== Collector started. Topics will appear as ROS2 nodes come up. ==="
echo "    Run: stop_and_collect_trace.sh when ready to stop."
