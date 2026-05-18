#!/bin/bash
# launch_poc_lab.sh — Multi-Domain POC Experiment Launcher  [LABORATORY MODE]
#
# Executes the complete multi-domain POC measurement sequence in order:
#   1. Clean stale DDS participants on all machines
#   2. Start STM32 memory collector (background)
#   3. Launch ROS2 nodes on RPi, Jetson, and base PC
#      • Camera: video-file fallback enabled if D415 cannot open
#      • Params: loaded from ws_base/params/lab/ (edit without navigating src/)
#   4. Wait for STM32 topics to be discovered and flowing
#   5. Start latency collectors on RPi and Jetson (D5 + D6)
#   6. Start net/CPU/SoftIRQ collectors + D4/D6 topic BW collectors
#   7. Wait for the run duration with a live verbose dashboard
#   8. Stop all collectors and pull CSVs (including telemetry_relay.csv)
#
# STM32 boards are assumed to be always powered on. No reset step is needed.
#
# Usage:
#   bash ws_base/launch_poc_lab.sh
#   bash ws_base/launch_poc_lab.sh --duration 600   # 10-minute run (default: 300s)
#   bash ws_base/launch_poc_lab.sh --skip-launch    # collectors only
#   bash ws_base/launch_poc_lab.sh --skip-stm32     # skip STM32 serial collection
#   bash ws_base/launch_poc_lab.sh --verbose        # extra logging + set -x
#
# Output files (all on base PC under a single per-run directory):
#   ws_base/runs/multi_domain/run_NNN/latency_rpi.csv
#   ws_base/runs/multi_domain/run_NNN/latency_jetson.csv
#   ws_base/runs/multi_domain/run_NNN/latency_jetson_d6.csv
#   ws_base/runs/multi_domain/run_NNN/net_stats_rpi.csv
#   ws_base/runs/multi_domain/run_NNN/net_stats_jetson.csv
#   ws_base/runs/multi_domain/run_NNN/topic_bw.csv
#   ws_base/runs/multi_domain/run_NNN/topic_bw_d4.csv
#   ws_base/runs/multi_domain/run_NNN/topic_bw_d6.csv
#   ws_base/runs/multi_domain/run_NNN/stm32_chassis.csv
#   ws_base/runs/multi_domain/run_NNN/stm32_sensors.csv
#   ws_base/runs/multi_domain/run_NNN/telemetry_relay.csv   ← base PC telemetry log
#   ws_base/runs/multi_domain/run_NNN/cpu_rpi.csv
#   ws_base/runs/multi_domain/run_NNN/cpu_jetson_tegrastats.txt
#   ws_base/runs/multi_domain/run_NNN/softirq_rpi.csv
#   ws_base/runs/multi_domain/run_NNN/softirq_jetson.csv
#   ws_base/runs/multi_domain/run_NNN/hz_report_rpi.txt
#   ws_base/runs/multi_domain/run_NNN/hz_report_jetson.txt
#   ws_base/runs/multi_domain/run_NNN/hz_report_jetson_d6.txt
#   ws_base/runs/multi_domain/run_NNN/merged_all.csv
#   ws_base/runs/multi_domain/run_NNN/logs/

set -euo pipefail

# ============================================================================
# Configuration — edit these if your network addresses change
# ============================================================================

RPI_HOST="curry@192.168.1.1"
JETSON_HOST="yupi@192.168.1.5"

CHASSIS_PORT="auto"
SENSORS_PORT="auto"

TOOLS_DIR="$HOME/almondmatcha/ws_base/tools"
WORKSPACE="$HOME/almondmatcha"

POC_RUN_BASE="$WORKSPACE/ws_base/runs/multi_domain"
PARAMS_DIR="$WORKSPACE/ws_base/params/lab"

_next_run_dir() {
    local last
    last=$(ls -d "${POC_RUN_BASE}"/run_* 2>/dev/null \
           | grep -oP 'run_\K[0-9]+' | sort -n | tail -1)
    last=$(( 10#${last:-0} ))
    printf '%s/run_%03d' "$POC_RUN_BASE" "$(( last + 1 ))"
}

RUN_DIR="$(_next_run_dir)"
LOG_DIR="$RUN_DIR/logs"

RUN_DURATION=300
SKIP_LAUNCH=false
SKIP_STM32=false
VERBOSE=false

SSH_CONTROL_DIR="$(mktemp -d /tmp/poc_ssh_ctl.XXXXXX)"
SSH_OPTS="-o ControlMaster=auto -o ControlPath=${SSH_CONTROL_DIR}/%r@%h:%p -o ControlPersist=600"

# ============================================================================
# Colours
# ============================================================================

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

log()  { echo -e "${CYAN}[$(date +%H:%M:%S)]${NC} $*"; }
ok()   { echo -e "${GREEN}[$(date +%H:%M:%S)] OK${NC} $*"; }
warn() { echo -e "${YELLOW}[$(date +%H:%M:%S)] WARN${NC} $*"; }
die()  { echo -e "${RED}[$(date +%H:%M:%S)] ERROR${NC} $*" >&2; exit 1; }
vlog() { [[ "$VERBOSE" == true ]] && echo -e "${CYAN}[$(date +%H:%M:%S)] [DBG]${NC} $*" || true; }
pause(){ echo -e "${YELLOW}$*${NC}"; read -rp "    Press ENTER when ready... " _; }

# ============================================================================
# Argument parsing
# ============================================================================

while [[ $# -gt 0 ]]; do
    case "$1" in
        --duration)    RUN_DURATION="$2"; shift 2 ;;
        --skip-launch) SKIP_LAUNCH=true; shift ;;
        --skip-stm32)  SKIP_STM32=true; shift ;;
        --chassis)     CHASSIS_PORT="$2"; shift 2 ;;
        --sensors)     SENSORS_PORT="$2"; shift 2 ;;
        --verbose)     VERBOSE=true; shift ;;
        *) die "Unknown argument: $1" ;;
    esac
done

[[ "$VERBOSE" == true ]] && set -x

# ============================================================================
# Helper — Stop ROS2 node tmux sessions on all machines
# ============================================================================

stop_ros2_nodes() {
    log "  Stopping ROS2 node tmux sessions on all machines"

    # ── Base PC ────────────────────────────────────────────────────────────────
    tmux kill-session -t base_poc   2>/dev/null || true
    pkill -f "ros2 run"             2>/dev/null || true

    # ── RPi ────────────────────────────────────────────────────────────────────
    ssh $SSH_OPTS "$RPI_HOST" "
        tmux kill-session -t rover_poc 2>/dev/null || true
        pkill -f 'ros2 run'            2>/dev/null || true
    " 2>/dev/null || warn "  RPi node stop had errors (non-fatal)"

    # ── Jetson ─────────────────────────────────────────────────────────────────
    ssh $SSH_OPTS "$JETSON_HOST" "
        tmux kill-session -t jetson_poc 2>/dev/null || true
        pkill -f 'ros2 run'             2>/dev/null || true
    " 2>/dev/null || warn "  Jetson node stop had errors (non-fatal)"

    ok "  All ROS2 node sessions stopped"
}

# ============================================================================
# Cleanup handler — kill background jobs on Ctrl-C or exit
# ============================================================================

STM32_COLLECTOR_PID=""
TOPIC_BW_PID=""
TOPIC_BW_D4_PID=""
# D6 bw runs remotely on Jetson — no local PID; killed via SSH in cleanup
_CLEANED_UP=false

cleanup() {
    if $_CLEANED_UP; then return; fi
    _CLEANED_UP=true
    echo ""
    warn "Interrupted — stopping all background collectors..."
    [[ -n "$STM32_COLLECTOR_PID" ]] && kill "$STM32_COLLECTOR_PID" 2>/dev/null || true
    [[ -n "$TOPIC_BW_PID"        ]] && kill "$TOPIC_BW_PID"        2>/dev/null || true
    # Stop CPU / SoftIRQ loggers on SBCs
    ssh $SSH_OPTS "$RPI_HOST"    "pkill -f 'cpu_logger_rpi\|softirq_logger_rpi'   2>/dev/null || true" 2>/dev/null || true
    ssh $SSH_OPTS "$JETSON_HOST" "pkill -f 'cpu_logger_jetson\|softirq_logger_jet' 2>/dev/null || true" 2>/dev/null || true
    ssh $SSH_OPTS "$RPI_HOST"    "pkill -f tegrastats 2>/dev/null || true" 2>/dev/null || true
    ssh $SSH_OPTS "$JETSON_HOST" "pkill -f tegrastats 2>/dev/null || true" 2>/dev/null || true
    # Stop latency collectors (cleanup path — best effort)
    LOCAL_DEST_CSV="$RUN_DIR/latency_rpi.csv"        TARGET_HOST="$RPI_HOST"    TARGET_LABEL=rpi    SSH_OPTS="$SSH_OPTS" \
        bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh" 2>/dev/null || true
    LOCAL_DEST_CSV="$RUN_DIR/latency_jetson.csv"    TARGET_HOST="$JETSON_HOST" TARGET_LABEL=jetson SSH_OPTS="$SSH_OPTS" \
        bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh" 2>/dev/null || true
    LOCAL_DEST_CSV="$RUN_DIR/latency_jetson_d6.csv" TARGET_HOST="$JETSON_HOST" TARGET_LABEL=jetson_d6 SSH_OPTS="$SSH_OPTS" \
        bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh" 2>/dev/null || true
    # Stop net-stats and pull partial data
    ssh $SSH_OPTS "$RPI_HOST"    "pkill -f 'collect_net_stats' 2>/dev/null || true" 2>/dev/null || true
    ssh $SSH_OPTS "$JETSON_HOST" "pkill -f 'collect_net_stats' 2>/dev/null || true" 2>/dev/null || true
    ssh $SSH_OPTS "$RPI_HOST"    "pkill -f 'cpu_logger_rpi\|softirq_logger_rpi' 2>/dev/null || true" 2>/dev/null || true
    ssh $SSH_OPTS "$JETSON_HOST" "pkill -f 'cpu_logger_jetson\|softirq_logger_jet\|tegrastats' 2>/dev/null || true" 2>/dev/null || true
    [[ -n "$RUN_DIR" ]] && {
        scp $SSH_OPTS "$RPI_HOST:~/ros2_traces/net_stats_rpi.csv"       "$RUN_DIR/net_stats_rpi.csv"          2>/dev/null || true
        scp $SSH_OPTS "$RPI_HOST:~/almondmatcha_poc/cpu_rpi.csv"        "$RUN_DIR/cpu_rpi.csv"                2>/dev/null || true
        scp $SSH_OPTS "$RPI_HOST:~/almondmatcha_poc/softirq_rpi.csv"    "$RUN_DIR/softirq_rpi.csv"            2>/dev/null || true
        scp $SSH_OPTS "$JETSON_HOST:~/ros2_traces/net_stats_jetson.csv" "$RUN_DIR/net_stats_jetson.csv"       2>/dev/null || true
        scp $SSH_OPTS "$JETSON_HOST:~/almondmatcha_poc/cpu_jetson_tegrastats.txt" \
                                                                         "$RUN_DIR/cpu_jetson_tegrastats.txt"  2>/dev/null || true
        scp $SSH_OPTS "$JETSON_HOST:~/almondmatcha_poc/softirq_jetson.csv" "$RUN_DIR/softirq_jetson.csv"      2>/dev/null || true
        # Copy telemetry relay CSV (base PC — written directly to RUN_DIR by the node)
        log "  Telemetry relay CSV: $RUN_DIR/telemetry_relay.csv"
    }
    stop_ros2_nodes
    ssh $SSH_OPTS -O exit "$RPI_HOST"    2>/dev/null || true
    ssh $SSH_OPTS -O exit "$JETSON_HOST" 2>/dev/null || true
    rm -rf "$SSH_CONTROL_DIR"
    echo ""
    log "Cleanup complete. CSVs may be partial."
    exit 1
}
trap cleanup INT TERM EXIT

# ============================================================================
# Pre-flight checks
# ============================================================================

preflight() {
    log "Running pre-flight checks  [LAB MODE]..."

    command -v tmux    &>/dev/null || die "tmux not installed"
    command -v ssh     &>/dev/null || die "ssh not installed"
    command -v python3 &>/dev/null || die "python3 not installed"
    python3 -c "import serial" 2>/dev/null || die "pyserial missing — run: pip3 install pyserial"

    [[ -d "$PARAMS_DIR" ]] || die "Lab params directory not found: $PARAMS_DIR"
    vlog "  Lab params directory: $PARAMS_DIR"
    for f in mission_command.yaml camera.yaml control.yaml; do
        [[ -f "$PARAMS_DIR/$f" ]] || die "Missing lab params file: $PARAMS_DIR/$f"
        vlog "  Found: $PARAMS_DIR/$f"
    done

    bash -c "
        source /opt/ros/humble/setup.bash
        source '$WORKSPACE/ws_base/install/setup.bash'
        source '$WORKSPACE/common_ifaces/install/setup.bash'
        python3 -c \"from rosidl_runtime_py.utilities import get_message; get_message('msgs_ifaces/msg/SpresenseGNSS')\"
    " 2>/dev/null \
        || die "msgs_ifaces not importable — rebuild common_ifaces: cd $WORKSPACE/common_ifaces && colcon build"
    ok "  msgs_ifaces type support verified"

    if [[ "$SKIP_STM32" == false ]]; then
        if [[ "$CHASSIS_PORT" == "auto" || "$SENSORS_PORT" == "auto" ]]; then
            local detected
            detected=$(python3 -c "import glob; p=sorted(glob.glob('/dev/ttyACM*')); print(' '.join(p))" 2>/dev/null)
            local port_count
            port_count=$(echo "$detected" | wc -w)
            if (( port_count >= 2 )); then
                if [[ "$CHASSIS_PORT" == "auto" ]]; then CHASSIS_PORT=$(echo "$detected" | awk '{print $1}'); fi
                if [[ "$SENSORS_PORT" == "auto" ]]; then SENSORS_PORT=$(echo "$detected" | awk '{print $2}'); fi
                ok "  STM32 ports auto-detected: chassis=$CHASSIS_PORT  sensors=$SENSORS_PORT"
            elif (( port_count == 1 )); then
                warn "  Only one /dev/ttyACM* found: $detected — need two for both STM32 boards"
                die "STM32 serial port auto-detect failed"
            else
                warn "  No /dev/ttyACM* ports found — STM32 boards not connected?"
                die "STM32 serial port auto-detect failed"
            fi
        else
            [[ -e "$CHASSIS_PORT" ]] || die "Chassis serial port $CHASSIS_PORT not found."
            [[ -e "$SENSORS_PORT" ]] || die "Sensors serial port $SENSORS_PORT not found."
            ok "  STM32 ports: chassis=$CHASSIS_PORT  sensors=$SENSORS_PORT"
        fi
    else
        warn "  --skip-stm32: STM32 memory collection will be skipped"
    fi

    log "  Authenticating to RPi ($RPI_HOST) — enter password/passphrase if prompted:"
    ssh $SSH_OPTS -o ConnectTimeout=10 "$RPI_HOST" true \
        || die "Cannot SSH to RPi ($RPI_HOST)"
    ok "  RPi authenticated"

    log "  Authenticating to Jetson ($JETSON_HOST) — enter password/passphrase if prompted:"
    ssh $SSH_OPTS -o ConnectTimeout=10 "$JETSON_HOST" true \
        || die "Cannot SSH to Jetson ($JETSON_HOST)"
    ok "  Jetson authenticated"

    # SCP lab params to Jetson (camera + control) so the Jetson launch script
    # can reference them without navigating src/ directories.
    log "  Uploading lab params to Jetson ($PARAMS_DIR/camera.yaml, control.yaml)..."
    ssh $SSH_OPTS "$JETSON_HOST" "mkdir -p ~/almondmatcha_poc/params/lab" 2>/dev/null
    scp $SSH_OPTS "$PARAMS_DIR/camera.yaml"  "$JETSON_HOST:~/almondmatcha_poc/params/lab/camera.yaml"  \
        || die "Failed to SCP camera.yaml to Jetson"
    scp $SSH_OPTS "$PARAMS_DIR/control.yaml" "$JETSON_HOST:~/almondmatcha_poc/params/lab/control.yaml" \
        || die "Failed to SCP control.yaml to Jetson"
    ok "  Lab params uploaded to Jetson"

    mkdir -p "$RUN_DIR" "$LOG_DIR"

    log "  Run directory   : $RUN_DIR"
    log "  Lab params dir  : $PARAMS_DIR"
    [[ "$VERBOSE" == true ]] && log "  Verbose mode    : ON"

    ok "All pre-flight checks passed"
}

# ============================================================================
# Step 0 — Clean stale DDS participants on ALL machines
# ============================================================================

clean_stale_participants() {
    log "Step 0 — Cleaning stale DDS participants on all machines"

    log "  Cleaning base PC..."
    pkill -f "ros2 run" 2>/dev/null || true
    pkill -f "collect_topic_bw" 2>/dev/null || true
    tmux kill-session -t base_poc 2>/dev/null || true
    ok "  Base PC cleaned"

    log "  Cleaning RPi ($RPI_HOST)..."
    ssh $SSH_OPTS "$RPI_HOST" "
        pkill -f 'ros2 run' 2>/dev/null || true
        pkill -f 'collect_latency' 2>/dev/null || true
        pkill -f 'collect_net_stats' 2>/dev/null || true
        tmux kill-session -t rover_poc 2>/dev/null || true
    " 2>/dev/null || warn "  RPi cleanup had errors (non-fatal)"
    ok "  RPi cleaned"

    log "  Cleaning Jetson ($JETSON_HOST)..."
    ssh $SSH_OPTS "$JETSON_HOST" "
        pkill -f 'ros2 run' 2>/dev/null || true
        pkill -f 'collect_latency' 2>/dev/null || true
        pkill -f 'collect_net_stats' 2>/dev/null || true
        pkill -f 'collect_topic_bw' 2>/dev/null || true
        tmux kill-session -t jetson_poc 2>/dev/null || true
    " 2>/dev/null || warn "  Jetson cleanup had errors (non-fatal)"
    ok "  Jetson cleaned"

    sleep 3
    ok "All stale DDS participants cleaned"
}

# ============================================================================
# Step 1 — Start STM32 memory collector
# ============================================================================

start_stm32_collector() {
    if [[ "$SKIP_STM32" == true ]]; then
        warn "Step 1 — STM32 memory collector SKIPPED (--skip-stm32)"
        return
    fi

    log "Step 1 — Starting STM32 memory collector"
    log "  chassis port : $CHASSIS_PORT"
    log "  sensors port : $SENSORS_PORT"
    log "  output stem  : $RUN_DIR/stm32  →  stm32_chassis/sensors_YYYYMMDD_HHMMSS.csv"

    python3 "$TOOLS_DIR/collect_stm32_memory.py" \
        --chassis "$CHASSIS_PORT" \
        --sensors "$SENSORS_PORT" \
        --out     "$RUN_DIR/stm32" \
        >"$LOG_DIR/stm32_collector.log" 2>&1 &
    STM32_COLLECTOR_PID=$!

    sleep 1
    kill -0 "$STM32_COLLECTOR_PID" 2>/dev/null \
        || die "STM32 collector exited immediately — check $LOG_DIR/stm32_collector.log"

    ok "STM32 collector running (PID $STM32_COLLECTOR_PID)"
}

# ============================================================================
# Step 2 — Launch ROS2 nodes on RPi, Jetson, then base PC
# ============================================================================

launch_ros2_nodes() {
    log "Step 2 — Launching ROS2 nodes  [LAB params: $PARAMS_DIR]"
    log "  Camera: video-file fallback enabled if D415 cannot open"
    log "  (launch output redirected to $LOG_DIR/launch_<host>.log)"

    log "  Launching rover nodes on RPi ($RPI_HOST)..."
    ssh $SSH_OPTS "$RPI_HOST" "SKIP_ATTACH=1 bash ~/almondmatcha/ws_rpi/launch_rover_multi_domain.sh" \
        >"$LOG_DIR/launch_rpi.log" 2>&1 &
    sleep 20
    ok "  RPi launch sent (8 nodes, 2 s stagger = ~16 s)"

    log "  Launching Jetson nodes ($JETSON_HOST) with lab camera+control params..."
    vlog "  Camera params : ~/almondmatcha_poc/params/lab/camera.yaml"
    vlog "  Control params: ~/almondmatcha_poc/params/lab/control.yaml"
    ssh $SSH_OPTS "$JETSON_HOST" "
        export SKIP_ATTACH=1
        export CAMERA_PARAMS_FILE=~/almondmatcha_poc/params/lab/camera.yaml
        export CONTROL_PARAMS_FILE=~/almondmatcha_poc/params/lab/control.yaml
        bash ~/almondmatcha/ws_jetson/launch_jetson_multi_domain.sh
    " >"$LOG_DIR/launch_jetson.log" 2>&1 &
    sleep 10
    ok "  Jetson launch sent (4 nodes with lab params)"

    log "  Launching base PC nodes with lab mission params + telemetry CSV..."
    vlog "  Mission params: $PARAMS_DIR/mission_command.yaml"
    vlog "  Telemetry CSV : $RUN_DIR/telemetry_relay.csv"
    export MISSION_CMD_PARAMS="$PARAMS_DIR/mission_command.yaml"
    export TELEMETRY_CSV_PATH="$RUN_DIR/telemetry_relay.csv"
    SKIP_ATTACH=1 bash "$WORKSPACE/ws_base/launch_base_multi_domain.sh" \
        >"$LOG_DIR/launch_base.log" 2>&1 &
    unset MISSION_CMD_PARAMS TELEMETRY_CSV_PATH
    sleep 5
    ok "  Base PC launch sent (2 nodes + telemetry CSV to $RUN_DIR/telemetry_relay.csv)"

    log "  All ROS2 nodes launched — D5/D4/D6 domains"
    log "  Total: 14 Linux nodes (D5: 10, D6: 2, D4: 2) staggered over ~35 s"
}

# ============================================================================
# Step 3 — Wait until BOTH STM32 topics are discovered and flowing
# ============================================================================

STM32_TOPICS=("/tpc_chassis_imu" "/tpc_chassis_sensors")

wait_stm32_topics() {
    log "Step 3 — Waiting for STM32 topics to be discovered and flowing"
    log "  Required topics: ${STM32_TOPICS[*]}"
    log "  Retries with increasing timeout — press Ctrl+C to abort"

    local start_time
    start_time=$(date +%s)

    local echo_helper
    echo_helper=$(mktemp)
    cat > "$echo_helper" << 'WAITEOF'
#!/bin/bash
source /opt/ros/humble/setup.bash 2>/dev/null
source "$1/ws_base/install/setup.bash" 2>/dev/null
source "$1/common_ifaces/install/setup.bash" 2>/dev/null
export ROS_DOMAIN_ID=5
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$1/ws_base/fastdds_base.xml"
TIMEOUT=$2
OUTDIR=$(mktemp -d)
shift 2
pids=()
for topic in "$@"; do
    (timeout "$TIMEOUT" ros2 topic echo "$topic" --once 2>/dev/null \
        > "$OUTDIR/$(echo "$topic" | tr '/' '_')" ) &
    pids+=($!)
done
wait "${pids[@]}" 2>/dev/null
for topic in "$@"; do
    fname="$OUTDIR/$(echo "$topic" | tr '/' '_')"
    echo "=== $topic ==="
    if [ -s "$fname" ]; then
        echo "OK"
        head -3 "$fname"
    else
        echo "=== TIMEOUT ==="
    fi
done
rm -rf "$OUTDIR"
WAITEOF
    chmod +x "$echo_helper"

    local max_attempts=6
    for attempt in $(seq 1 $max_attempts); do
        local echo_timeout=$(( 6 + attempt * 6 ))
        local elapsed=$(( $(date +%s) - start_time ))
        log "  Attempt $attempt/$max_attempts (parallel echo, timeout ${echo_timeout}s, elapsed ${elapsed}s)..."

        local combined_output
        combined_output=$(bash "$echo_helper" "$WORKSPACE" "$echo_timeout" \
            "${STM32_TOPICS[@]}" 2>/dev/null) || true

        local all_ok=true
        for t in "${STM32_TOPICS[@]}"; do
            local block
            block=$(echo "$combined_output" | sed -n "\\#^=== ${t} ===#,\\#^===#p" | head -5)
            if echo "$block" | grep -q "^OK"; then
                ok "  $t confirmed flowing after ${elapsed}s"
                vlog "  Sample data: $(echo "$block" | grep -v '^OK' | head -2)"
            else
                all_ok=false
                vlog "  $t still waiting..."
            fi
        done

        if $all_ok; then
            echo ""
            ok "  All STM32 topics confirmed — experiment data will be clean from the start"
            rm -f "$echo_helper"
            return 0
        fi

        if (( attempt < max_attempts )); then
            local pending=""
            for t in "${STM32_TOPICS[@]}"; do
                local block
                block=$(echo "$combined_output" | sed -n "\\#^=== ${t} ===#,\\#^===#p" | head -5)
                echo "$block" | grep -q "^OK" || pending+=" $t"
            done
            log "  Not all topics received data — retrying in 3s... pending:${pending}"
            sleep 3
        fi
    done

    rm -f "$echo_helper"
    die "STM32 topics not confirmed after $max_attempts attempts — check STM32 serial output"
}

# ============================================================================
# Step 4 — Start latency collectors on RPi and Jetson
# ============================================================================

start_latency_collectors() {
    log "Step 4 — Starting latency/jitter collectors on RPi and Jetson"

    TARGET_HOST="$RPI_HOST"    TARGET_LABEL=rpi    SSH_OPTS="$SSH_OPTS" \
        bash "$TOOLS_DIR/tracing/start_trace.sh"
    ok "  RPi latency collector started (D5)"

    TARGET_HOST="$JETSON_HOST" TARGET_LABEL=jetson SSH_OPTS="$SSH_OPTS" \
        bash "$TOOLS_DIR/tracing/start_trace.sh"
    ok "  Jetson latency collector started (D5)"

    # Multi-domain: D6 vision topics on Jetson shared memory
    TARGET_HOST="$JETSON_HOST" TARGET_LABEL=jetson_d6 SSH_OPTS="$SSH_OPTS" \
        bash "$TOOLS_DIR/tracing/start_trace_d6.sh"
    ok "  Jetson D6 latency collector started (D6: camera/lane)"
}

# ============================================================================
# Step 5 — Start per-topic bandwidth collector (base PC, local)
# ============================================================================

start_topic_bw_collector() {
    log "Step 5 — Starting per-topic bandwidth collectors (base PC: D5 + D4; Jetson: D6)"
    log "  D5 output: $RUN_DIR/topic_bw.csv"

    bash -c "
        source /opt/ros/humble/setup.bash
        source '$WORKSPACE/ws_base/install/setup.bash'
        source '$WORKSPACE/common_ifaces/install/setup.bash'
        export ROS_DOMAIN_ID=5
        export FASTRTPS_DEFAULT_PROFILES_FILE='$WORKSPACE/ws_base/fastdds_base.xml'
        exec python3 '$TOOLS_DIR/collect_topic_bw.py' \
            --out '$RUN_DIR/topic_bw.csv' \
            --interval 1
    " </dev/null >"$LOG_DIR/topic_bw.log" 2>&1 &
    TOPIC_BW_PID=$!
    sleep 1
    kill -0 "$TOPIC_BW_PID" 2>/dev/null \
        || die "Topic BW (D5) exited immediately — check $LOG_DIR/topic_bw.log"
    ok "  D5 Topic BW collector running (PID $TOPIC_BW_PID)"

    # D4 collector — base PC, no FastDDS XML (no STM32 on D4)
    log "  D4 output: $RUN_DIR/topic_bw_d4.csv"
    bash -c "
        source /opt/ros/humble/setup.bash
        source '$WORKSPACE/ws_base/install/setup.bash'
        source '$WORKSPACE/common_ifaces/install/setup.bash'
        export ROS_DOMAIN_ID=4
        unset FASTRTPS_DEFAULT_PROFILES_FILE
        exec python3 '$TOOLS_DIR/collect_topic_bw.py' \
            --out '$RUN_DIR/topic_bw_d4.csv' \
            --interval 1
    " </dev/null >"$LOG_DIR/topic_bw_d4.log" 2>&1 &
    TOPIC_BW_D4_PID=$!
    sleep 1
    kill -0 "$TOPIC_BW_D4_PID" 2>/dev/null \
        || die "Topic BW (D4) exited immediately — check $LOG_DIR/topic_bw_d4.log"
    ok "  D4 Topic BW collector running (PID $TOPIC_BW_D4_PID)"

    # D6 collector — remote Jetson, shared memory domain
    log "  D6 output: $RUN_DIR/topic_bw_d6.csv  (collected on Jetson)"
    ssh -T $SSH_OPTS "$JETSON_HOST" "
        source /opt/ros/humble/setup.bash
        source ~/almondmatcha/ws_jetson/install/setup.bash 2>/dev/null || true
        source ~/almondmatcha/common_ifaces/install/setup.bash 2>/dev/null || true
        export ROS_DOMAIN_ID=6
        unset FASTRTPS_DEFAULT_PROFILES_FILE
        mkdir -p ~/ros2_traces
        pkill -f 'collect_topic_bw' 2>/dev/null || true
        setsid nohup python3 ~/almondmatcha/ws_base/tools/collect_topic_bw.py \
            --out ~/ros2_traces/topic_bw_d6.csv \
            --interval 1 \
            </dev/null >~/ros2_traces/topic_bw_d6.log 2>&1 &
        disown
        echo [OK] D6 Topic BW collector PID: \$!
    " && ok "  D6 Topic BW collector started on Jetson" \
      || warn "  D6 Topic BW collector failed to start on Jetson (non-fatal)"
}

# ============================================================================
# Step 6 — Start net-stats, CPU, and SoftIRQ collectors
# ============================================================================

start_net_collectors() {
    log "Step 6 — Starting network stats collectors on RPi and Jetson"

    remote_start_net() {
        local host="$1" out="$2" log_f="$3" pid_f="$4"
        ssh -T $SSH_OPTS "$host" "
            mkdir -p ~/ros2_traces
            setsid nohup python3 ~/almondmatcha/ws_base/tools/collect_net_stats.py \
                --out $out \
                </dev/null >$log_f 2>&1 &
            disown
            echo \$! > $pid_f
            echo [OK] net_stats collector PID: \$(cat $pid_f)
        "
    }

    remote_start_net "$RPI_HOST"    "~/ros2_traces/net_stats_rpi.csv"    "~/ros2_traces/net_stats_rpi.log"    "~/ros2_traces/net_stats_rpi.pid"
    remote_start_net "$JETSON_HOST" "~/ros2_traces/net_stats_jetson.csv" "~/ros2_traces/net_stats_jetson.log" "~/ros2_traces/net_stats_jetson.pid"
    ok "  Net-stats collectors started on both SBCs"
}

start_cpu_collectors() {
    log "Step 6b — Starting CPU load loggers on RPi and Jetson"

    ssh -T $SSH_OPTS "$RPI_HOST" '
        mkdir -p ~/almondmatcha_poc
        echo "timestamp,cpu_pct,mem_pct,temp_c" > ~/almondmatcha_poc/cpu_rpi.csv
        exec -a cpu_logger_rpi bash -c "
            while sleep 1; do
                echo \"\$(date +%s),\$(top -bn1 | grep \"Cpu(s)\" | awk \"{print 100-\\\$8}\"),\$(free | awk \"/Mem/{printf \\\"%.2f\\\",\\\$3/\\\$2*100}\"),\$(cat /sys/class/thermal/thermal_zone0/temp | awk \"{print \\\$1/1000}\")\"
            done >> ~/almondmatcha_poc/cpu_rpi.csv
        " </dev/null >~/almondmatcha_poc/cpu_rpi.log 2>&1 &
        disown
        echo $!
    ' > "$LOG_DIR/cpu_rpi_pid.txt" 2>/dev/null || true
    ok "  RPi CPU logger started"

    ssh -T $SSH_OPTS "$JETSON_HOST" '
        mkdir -p ~/almondmatcha_poc
        exec -a cpu_logger_jetson bash -c "
            tegrastats --interval 1000 | \
            while IFS= read -r line; do echo \"\$(date +%s) \$line\"; done
        " </dev/null > ~/almondmatcha_poc/cpu_jetson_tegrastats.txt 2>&1 &
        disown
        echo $!
    ' > "$LOG_DIR/cpu_jetson_pid.txt" 2>/dev/null || true
    ok "  Jetson CPU logger (tegrastats) started"
}

start_softirq_collectors() {
    log "Step 6c — Starting SoftIRQ delta loggers on RPi and Jetson"

    local script
    script='mkdir -p "$(dirname "$1")"
echo "timestamp,NET_RX_delta,NET_TX_delta,SCHED_delta,TIMER_delta" > "$1"
prev_rx=0; prev_tx=0; prev_sched=0; prev_timer=0
while sleep 1; do
  ts=$(date +%s)
  rx=$(awk "/NET_RX:/{s=0;for(i=2;i<=NF;i++)s+=\$i;print s}" /proc/softirqs)
  tx=$(awk "/NET_TX:/{s=0;for(i=2;i<=NF;i++)s+=\$i;print s}" /proc/softirqs)
  sc=$(awk "/SCHED:/{s=0;for(i=2;i<=NF;i++)s+=\$i;print s}"  /proc/softirqs)
  ti=$(awk "/TIMER:/{s=0;for(i=2;i<=NF;i++)s+=\$i;print s}"  /proc/softirqs)
  echo "$ts,$((rx-prev_rx)),$((tx-prev_tx)),$((sc-prev_sched)),$((ti-prev_timer))" >> "$1"
  prev_rx=$rx; prev_tx=$tx; prev_sched=$sc; prev_timer=$ti
done'

    ssh -T $SSH_OPTS "$RPI_HOST" "
        mkdir -p ~/almondmatcha_poc
        exec -a softirq_logger_rpi bash -c '${script//$'\n'/; }' -- ~/almondmatcha_poc/softirq_rpi.csv </dev/null >~/almondmatcha_poc/softirq_rpi.log 2>&1 &
        disown
        echo \$!
    " > "$LOG_DIR/softirq_rpi_pid.txt" 2>/dev/null || true
    ok "  RPi SoftIRQ logger started"

    ssh -T $SSH_OPTS "$JETSON_HOST" "
        mkdir -p ~/almondmatcha_poc
        exec -a softirq_logger_jetson bash -c '${script//$'\n'/; }' -- ~/almondmatcha_poc/softirq_jetson.csv </dev/null >~/almondmatcha_poc/softirq_jetson.log 2>&1 &
        disown
        echo \$!
    " > "$LOG_DIR/softirq_jetson_pid.txt" 2>/dev/null || true
    ok "  Jetson SoftIRQ logger started"
}

# ============================================================================
# Step 7 — Wait for the run duration (with live verbose dashboard)
# ============================================================================

wait_for_run() {
    local bar_width=40
    local DASH=10  # lines in the live-refresh dashboard block

    echo ""
    echo -e "${BOLD}=====================================================================${NC}"
    echo -e "${GREEN}  LAB EXPERIMENT RUNNING — ${RUN_DURATION}s measurement window${NC}"
    echo -e "${BOLD}=====================================================================${NC}"
    echo ""
    echo "  Mode          : LABORATORY (camera fallback enabled, verbose ON)"
    echo "  Params dir    : $PARAMS_DIR"
    echo "  Run directory : $RUN_DIR"
    echo "  Telemetry CSV : $RUN_DIR/telemetry_relay.csv  (live, written by monitoring node)"
    echo ""
    echo "  Drive the rover / send mission commands now."
    echo "  Press Ctrl-C at any time to stop early and collect CSVs."
    echo ""

    for (( i=0; i<DASH; i++ )); do echo ""; done

    local elapsed=0 bar filled empty pct i
    local ch_line se_line ch_used ch_free ch_n se_used se_free se_n stm32_status bw_status
    local hz_captured=false
    local hz_midpoint=$(( RUN_DURATION / 2 ))
    local telem_rows=0

    while (( elapsed < RUN_DURATION )); do
        sleep 1
        elapsed=$(( elapsed + 1 ))

        if [[ "$hz_captured" == false ]] && (( elapsed >= hz_midpoint )); then
            hz_captured=true
            log "  Mid-run: capturing ros2 topic hz report..."
            capture_hz_report &
        fi

        filled=$(( elapsed * bar_width / RUN_DURATION ))
        empty=$(( bar_width - filled ))
        bar=""
        for (( i=0; i<filled; i++ )); do bar+="█"; done
        for (( i=0; i<empty;  i++ )); do bar+="░"; done
        pct=$(( elapsed * 100 / RUN_DURATION ))

        ch_line=$(grep '\[chassis\]' "$LOG_DIR/stm32_collector.log" 2>/dev/null | tail -1) || ch_line=""
        se_line=$(grep '\[sensors\]' "$LOG_DIR/stm32_collector.log" 2>/dev/null | tail -1) || se_line=""
        ch_used=$(printf '%s' "$ch_line" | grep -oP 'used=\s*\K\S+') || ch_used="--"
        ch_free=$(printf '%s' "$ch_line" | grep -oP 'free=\s*\K\S+') || ch_free="--"
        ch_n=$(grep -c '\[chassis\]' "$LOG_DIR/stm32_collector.log" 2>/dev/null) || ch_n=0
        se_used=$(printf '%s' "$se_line" | grep -oP 'used=\s*\K\S+') || se_used="--"
        se_free=$(printf '%s' "$se_line" | grep -oP 'free=\s*\K\S+') || se_free="--"
        se_n=$(grep -c '\[sensors\]' "$LOG_DIR/stm32_collector.log" 2>/dev/null) || se_n=0

        # Count telemetry relay CSV rows (live data quality indicator)
        telem_rows=$(wc -l < "$RUN_DIR/telemetry_relay.csv" 2>/dev/null || echo 0)
        telem_rows=$(( telem_rows - 1 ))  # subtract header
        (( telem_rows < 0 )) && telem_rows=0

        if [[ -n "$STM32_COLLECTOR_PID" ]]; then
            kill -0 "$STM32_COLLECTOR_PID" 2>/dev/null \
                && stm32_status="\033[0;32m● running\033[0m" \
                || stm32_status="\033[0;31m● DIED\033[0m"
        else
            stm32_status="\033[1;33m● skipped\033[0m"
        fi
        kill -0 "$TOPIC_BW_PID" 2>/dev/null \
            && bw_status="\033[0;32m● running\033[0m" \
            || bw_status="\033[0;31m● DIED\033[0m"

        printf "\033[%dA\r" "$DASH"
        printf "  Progress:%-61s\n" ""
        printf "  [%-${bar_width}s] %3d%%  %ds / %ds%-20s\n" "$bar" "$pct" "$elapsed" "$RUN_DURATION" ""
        printf "%-70s\n" ""
        printf "  STM32 memory (latest):%-48s\n" ""
        printf "    chassis  used=%-8s  free=%-8s  (%4d samples)%-10s\n" "$ch_used" "$ch_free" "$ch_n" ""
        printf "    sensors  used=%-8s  free=%-8s  (%4d samples)%-10s\n" "$se_used" "$se_free" "$se_n" ""
        printf "%-70s\n" ""
        printf "  Telemetry relay CSV rows: %-8d (tpc_telemetry_relay @ ~5 Hz)%-5s\n" "$telem_rows" ""
        printf "  Collectors:%-59s\n" ""
        printf "    STM32 %b  BW %b %b  D6(Jetson)\033[0;32m\u25cf\033[0m  RPi lat \033[0;32m\u25cf\033[0m  Jetson lat \033[0;32m\u25cf\033[0m   \n" \
               "$stm32_status" "$bw_status" ""
    done

    echo ""
    log "Run duration complete"
}

# ============================================================================
# Step 7b — Capture ros2 topic hz report (called mid-run from wait_for_run)
# ============================================================================

capture_hz_report() {
    local topics_d5="/tpc_chassis_imu /tpc_chassis_sensors /tpc_rover_ctrl_cmd"
    local topics_d6="/tpc_rover_d415_rgb /tpc_rover_d415_depth /tpc_rover_lane_info"

    local _run_hz_d5
    _run_hz_d5() {
        local host="$1" outfile="$2" ws_path="$3"
        ssh -T $SSH_OPTS "$host" "
            source /opt/ros/humble/setup.bash 2>/dev/null
            source ${ws_path}/install/setup.bash 2>/dev/null
            export ROS_DOMAIN_ID=5
            for topic in ${topics_d5}; do
                echo \"--- \$topic ---\"
                timeout 10 ros2 topic hz \$topic --window 50 2>/dev/null || echo \"(no messages)\"
            done
        " > "$outfile" 2>/dev/null || true
    }

    local _run_hz_d6
    _run_hz_d6() {
        local outfile="$1"
        ssh -T $SSH_OPTS "$JETSON_HOST" "
            source /opt/ros/humble/setup.bash 2>/dev/null
            source ~/almondmatcha/ws_jetson/install/setup.bash 2>/dev/null
            export ROS_DOMAIN_ID=6
            for topic in ${topics_d6}; do
                echo \"--- \$topic ---\"
                timeout 10 ros2 topic hz \$topic --window 50 2>/dev/null || echo \"(no messages)\"
            done
        " > "$outfile" 2>/dev/null || true
    }

    _run_hz_d5 "$RPI_HOST"    "$RUN_DIR/hz_report_rpi.txt"        "~/almondmatcha/ws_rpi"    &
    _run_hz_d5 "$JETSON_HOST" "$RUN_DIR/hz_report_jetson.txt"     "~/almondmatcha/ws_jetson" &
    _run_hz_d6                "$RUN_DIR/hz_report_jetson_d6.txt" &
    wait
    ok "  hz reports saved to $RUN_DIR/hz_report_{rpi,jetson,jetson_d6}.txt"
}

# ============================================================================
# Step 8 — Stop collectors and pull CSVs
# ============================================================================

stop_and_collect() {
    log "Step 8 — Stopping all collectors and pulling CSVs"

    mkdir -p "$LOG_DIR/rpi" "$LOG_DIR/jetson"

    [[ -n "$STM32_COLLECTOR_PID" ]] && kill "$STM32_COLLECTOR_PID" 2>/dev/null || true
    [[ -n "$TOPIC_BW_PID"        ]] && kill "$TOPIC_BW_PID"        2>/dev/null || true
    [[ -n "$TOPIC_BW_D4_PID"    ]] && kill "$TOPIC_BW_D4_PID"    2>/dev/null || true
    ssh $SSH_OPTS "$JETSON_HOST" "pkill -f 'collect_topic_bw' 2>/dev/null || true" 2>/dev/null || true
    sleep 0.5
    local f
    f=$(ls "$RUN_DIR"/stm32_chassis_*.csv 2>/dev/null | tail -1) && [[ -n "$f" ]] && mv "$f" "$RUN_DIR/stm32_chassis.csv" || true
    f=$(ls "$RUN_DIR"/stm32_sensors_*.csv 2>/dev/null | tail -1) && [[ -n "$f" ]] && mv "$f" "$RUN_DIR/stm32_sensors.csv" || true
    ok "  Local collectors stopped"

    # Log telemetry CSV row count
    local trows
    trows=$(wc -l < "$RUN_DIR/telemetry_relay.csv" 2>/dev/null || echo 1)
    trows=$(( trows - 1 ))
    ok "  Telemetry relay CSV: $RUN_DIR/telemetry_relay.csv  ($trows rows)"
    # Redundant copy to LOG_DIR (belt-and-suspenders)
    cp "$RUN_DIR/telemetry_relay.csv" "$LOG_DIR/telemetry_relay_backup.csv" 2>/dev/null || true

    log "  Stopping and pulling from RPi and Jetson in parallel..."

    (
        LOCAL_DEST_CSV="$RUN_DIR/latency_rpi.csv" TARGET_HOST="$RPI_HOST" \
            TARGET_LABEL=rpi SSH_OPTS="$SSH_OPTS" \
            bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh"

        ssh -T $SSH_OPTS "$RPI_HOST" "
            if [ -f ~/ros2_traces/net_stats_rpi.pid ]; then
                kill \$(cat ~/ros2_traces/net_stats_rpi.pid) 2>/dev/null || true
                rm -f ~/ros2_traces/net_stats_rpi.pid
            fi
            pkill -f cpu_logger_rpi     2>/dev/null || true
            pkill -f softirq_logger_rpi 2>/dev/null || true
        "
        scp $SSH_OPTS "$RPI_HOST:~/ros2_traces/net_stats_rpi.csv"       "$RUN_DIR/net_stats_rpi.csv"     2>/dev/null || true
        scp $SSH_OPTS "$RPI_HOST:~/almondmatcha_poc/cpu_rpi.csv"        "$RUN_DIR/cpu_rpi.csv"           2>/dev/null || true
        scp $SSH_OPTS "$RPI_HOST:~/almondmatcha_poc/softirq_rpi.csv"    "$RUN_DIR/softirq_rpi.csv"       2>/dev/null || true
        scp $SSH_OPTS "$RPI_HOST:~/ros2_traces/poc_*.log"               "$LOG_DIR/rpi/"                  2>/dev/null || true
        echo "[parallel-rpi] done"
    ) &
    RPI_PULL_PID=$!

    (
        LOCAL_DEST_CSV="$RUN_DIR/latency_jetson.csv" TARGET_HOST="$JETSON_HOST" \
            TARGET_LABEL=jetson SSH_OPTS="$SSH_OPTS" \
            bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh"

        # Stop D6 latency collector and pull CSV
        LOCAL_DEST_CSV="$RUN_DIR/latency_jetson_d6.csv" TARGET_HOST="$JETSON_HOST" \
            TARGET_LABEL=jetson_d6 SSH_OPTS="$SSH_OPTS" \
            bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh" 2>/dev/null || true

        ssh -T $SSH_OPTS "$JETSON_HOST" "
            if [ -f ~/ros2_traces/net_stats_jetson.pid ]; then
                kill \$(cat ~/ros2_traces/net_stats_jetson.pid) 2>/dev/null || true
                rm -f ~/ros2_traces/net_stats_jetson.pid
            fi
            pkill -f cpu_logger_jetson     2>/dev/null || true
            pkill -f softirq_logger_jetson 2>/dev/null || true
            pkill -f tegrastats             2>/dev/null || true
        "
        scp $SSH_OPTS "$JETSON_HOST:~/ros2_traces/net_stats_jetson.csv"           "$RUN_DIR/net_stats_jetson.csv"       2>/dev/null || true
        scp $SSH_OPTS "$JETSON_HOST:~/almondmatcha_poc/cpu_jetson_tegrastats.txt" "$RUN_DIR/cpu_jetson_tegrastats.txt"  2>/dev/null || true
        scp $SSH_OPTS "$JETSON_HOST:~/almondmatcha_poc/softirq_jetson.csv"        "$RUN_DIR/softirq_jetson.csv"         2>/dev/null || true
        # Pull D6 topic BW CSV
        scp $SSH_OPTS "$JETSON_HOST:~/ros2_traces/topic_bw_d6.csv"                "$RUN_DIR/topic_bw_d6.csv"            2>/dev/null \
            || warn "  topic_bw_d6.csv not found on Jetson (non-fatal)"
        scp $SSH_OPTS "$JETSON_HOST:~/ros2_traces/poc_*.log"                      "$LOG_DIR/jetson/"                    2>/dev/null || true
        echo "[parallel-jetson] done"
    ) &
    JETSON_PULL_PID=$!

    wait "$RPI_PULL_PID"    && ok "  RPi:    net-stats + latency + logs pulled" \
                            || warn "  RPi pull had errors — check files"
    wait "$JETSON_PULL_PID" && ok "  Jetson: net-stats + latency + logs pulled" \
                            || warn "  Jetson pull had errors — check files"

    stop_ros2_nodes

    ssh $SSH_OPTS -O exit "$RPI_HOST"    2>/dev/null || true
    ssh $SSH_OPTS -O exit "$JETSON_HOST" 2>/dev/null || true
    rm -rf "$SSH_CONTROL_DIR"
}

# ============================================================================
# Step 9b — Post-run analysis
# ============================================================================

run_post_analysis() {
    log "Step 9b — Running post-run analysis (merge + stats + chart)..."
    bash "$TOOLS_DIR/post_run.sh" "$RUN_DIR" \
        && ok "  Post-run analysis complete" \
        || { warn "  post_run.sh had errors — raw CSVs are intact, re-run manually:"
             warn "    bash ws_base/tools/post_run.sh $RUN_DIR"; }
}

# ============================================================================
# Step 9 — Print summary
# ============================================================================

print_summary() {
    echo ""
    echo -e "${BOLD}=====================================================================${NC}"
    echo -e "${GREEN}  LAB EXPERIMENT COMPLETE${NC}"
    echo -e "${BOLD}=====================================================================${NC}"
    echo ""
    echo "  Run directory: $RUN_DIR"
    echo "  Mode: LABORATORY | Branch: multi-domain | Domains: D5/D4/D6 | Duration: ${RUN_DURATION}s"
    echo ""
    echo "  Raw CSVs:"
    for f in latency_rpi.csv latency_jetson.csv latency_jetson_d6.csv \
              net_stats_rpi.csv net_stats_jetson.csv \
              topic_bw.csv topic_bw_d4.csv topic_bw_d6.csv \
              stm32_chassis.csv stm32_sensors.csv telemetry_relay.csv \
              cpu_rpi.csv softirq_rpi.csv softirq_jetson.csv; do
        [[ -f "$RUN_DIR/$f" ]] && echo "    $RUN_DIR/$f"
    done
    echo ""
    echo "  Processed results:"
    for f in merged_all.csv merged_flat.csv latency_summary.csv unified_timeline.png; do
        [[ -f "$RUN_DIR/$f" ]] && echo "    $RUN_DIR/$f"
    done
    echo ""
    echo "  Logs: $LOG_DIR/"
    echo ""
    echo "  Re-run post-processing:"
    echo "    bash ws_base/tools/post_run.sh $RUN_DIR"
    echo ""
}

# ============================================================================
# Main
# ============================================================================

main() {
    # Disarm EXIT trap for normal completion so cleanup() is not called
    # (cleanup is only for abnormal exits — normal path runs print_summary).
    trap - EXIT

    echo ""
    echo -e "${BOLD}  Multi-Domain POC — LABORATORY MODE${NC}"
    echo -e "  Camera fallback: enabled | Params: $PARAMS_DIR"
    echo -e "  Branch: multi-domain | Domain: D5 | Duration: ${RUN_DURATION}s"
    echo ""

    preflight
    clean_stale_participants
    start_stm32_collector

    if [[ "$SKIP_LAUNCH" == false ]]; then
        launch_ros2_nodes
        wait_stm32_topics
    else
        warn "  --skip-launch set: skipping ROS2 node launch"
    fi

    start_latency_collectors
    start_topic_bw_collector
    start_net_collectors
    start_cpu_collectors
    start_softirq_collectors
    wait_for_run
    stop_and_collect
    run_post_analysis
    print_summary
}

main
