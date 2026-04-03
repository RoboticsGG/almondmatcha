#!/bin/bash
# stop_and_collect_trace.sh — Stop collect_latency.py and pull CSV to base PC.
#
# Usage (from base PC):
#   TARGET_HOST=curry@192.168.1.1 TARGET_LABEL=rpi   bash stop_and_collect_trace.sh
#   TARGET_HOST=yupi@192.168.1.5  TARGET_LABEL=jetson bash stop_and_collect_trace.sh

set -e

TARGET_HOST="${TARGET_HOST:-}"
TARGET_LABEL="${TARGET_LABEL:-unknown}"
PID_FILE="\$HOME/ros2_traces/collector_${TARGET_LABEL}.pid"
REMOTE_CSV="~/ros2_traces/latency_${TARGET_LABEL}.csv"

LOCAL_DATA_DIR="$HOME/almondmatcha/ws_base/tools/tracing/data"
mkdir -p "$LOCAL_DATA_DIR"

run_remote() {
    if [ -n "$TARGET_HOST" ]; then
        ssh "$TARGET_HOST" "$@"
    else
        bash -c "$@"
    fi
}

echo "=== Stopping collector on ${TARGET_HOST:-localhost} (label=${TARGET_LABEL}) ==="

run_remote "
if [ -f '${PID_FILE}' ]; then
    PID=\$(cat '${PID_FILE}')
    if kill -0 \"\$PID\" 2>/dev/null; then
        kill \"\$PID\"
        echo '[OK] Collector stopped (PID '\$PID')'
    else
        echo '[WARN] Collector PID '\$PID' was not running'
    fi
    rm -f '${PID_FILE}'
else
    echo '[WARN] No PID file found at ${PID_FILE} — collector may already be stopped'
fi
"

if [ -n "$TARGET_HOST" ]; then
    echo "=== Pulling CSV from ${TARGET_HOST}:${REMOTE_CSV} ==="
    scp "${TARGET_HOST}:${REMOTE_CSV}" \
        "${LOCAL_DATA_DIR}/poc_latency_${TARGET_LABEL}.csv"
    echo "[OK] CSV saved to: ${LOCAL_DATA_DIR}/poc_latency_${TARGET_LABEL}.csv"
else
    cp "$(eval echo $REMOTE_CSV)" \
       "${LOCAL_DATA_DIR}/poc_latency_${TARGET_LABEL}.csv"
fi

echo ""
echo "Next: analyze with:"
echo "  python3 ws_base/tools/tracing/analyze_latency.py \\"
echo "      --poc ${LOCAL_DATA_DIR}/poc_latency_${TARGET_LABEL}.csv"
