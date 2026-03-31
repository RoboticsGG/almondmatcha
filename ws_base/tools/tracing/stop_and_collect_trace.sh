#!/bin/bash
# stop_and_collect_trace.sh — Stop LTTng session and pull CTF data to base PC
#
# Usage:
#   TARGET_HOST=pi@192.168.1.x TARGET_LABEL=rpi   bash stop_and_collect_trace.sh
#   TARGET_HOST=yupi@192.168.1.x TARGET_LABEL=jetson bash stop_and_collect_trace.sh
#
# After pulling, run analyze_latency.py on the local CTF directory.

set -e

TARGET_HOST="${TARGET_HOST:-}"
TARGET_LABEL="${TARGET_LABEL:-unknown}"
SESSION_NAME="ros2_poc_${TARGET_LABEL}"

# Read trace dir saved by start_trace.sh
TRACE_DIR_FILE="$HOME/ros2_traces/last_trace_dir_${TARGET_LABEL}.txt"
if [ -f "$TRACE_DIR_FILE" ]; then
    REMOTE_TRACE_DIR=$(cat "$TRACE_DIR_FILE")
else
    echo "[ERROR] Cannot find trace dir reference at $TRACE_DIR_FILE"
    echo "        Set REMOTE_TRACE_DIR manually and re-run."
    exit 1
fi

LOCAL_DEST="$HOME/almondmatcha/ws_base/tools/tracing/traces/${TARGET_LABEL}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOCAL_DEST"

run_cmd() {
    if [ -n "$TARGET_HOST" ]; then
        ssh "$TARGET_HOST" "$@"
    else
        bash -c "$@"
    fi
}

echo "=== Stopping LTTng session '${SESSION_NAME}' ==="
run_cmd "
lttng stop '${SESSION_NAME}' && \
lttng destroy '${SESSION_NAME}' && \
echo '[OK] Session stopped.'
"

if [ -n "$TARGET_HOST" ]; then
    echo "=== Pulling CTF data from ${TARGET_HOST}:${REMOTE_TRACE_DIR} ==="
    scp -r "${TARGET_HOST}:${REMOTE_TRACE_DIR}" "$LOCAL_DEST/"
    echo "[OK] Traces saved to: $LOCAL_DEST"
else
    cp -r "$REMOTE_TRACE_DIR" "$LOCAL_DEST/"
    echo "[OK] Local traces copied to: $LOCAL_DEST"
fi

echo ""
echo "Next: analyze latency/jitter with:"
echo "  python3 ws_base/tools/tracing/analyze_latency.py --trace-dir '$LOCAL_DEST'"
