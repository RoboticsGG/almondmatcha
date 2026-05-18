#!/bin/bash
# launch_poc_field.sh — Single-Domain POC Experiment Launcher  [ON-FIELD MODE]
#
# On-field differences vs launch_poc_lab.sh:
#   • Camera is MANDATORY — no video-file fallback (real sensor required)
#   • Ctrl-C at ANY point triggers a clean emergency stop across all machines
#   • set -e is disabled: individual errors do not abort the run
#   • All invoked processes (local + remote) are tracked and killed on exit
#   • Params loaded from ws_base/params/field/ — edit gains/targets before deployment
#
# Executes the complete single-domain POC measurement sequence in order:
#   1. Clean stale DDS participants on all machines
#   2. Start STM32 memory collector (background)
#   3. Launch ROS2 nodes on RPi, Jetson, and base PC
#      • Camera: live D415 ONLY — node exits with error if D415 not present
#      • Params: loaded from ws_base/params/field/
#   4. Wait for STM32 topics to be discovered and flowing
#   5. Start latency collectors on RPi and Jetson
#   6. Start net/CPU/SoftIRQ collectors
#   7. Wait for the run duration — Ctrl-C = EMERGENCY STOP
#   8. Stop all collectors and pull CSVs (including telemetry_relay.csv)
#
# Emergency stop procedure:
#   Press Ctrl-C at ANY time.  The script will:
#     1. Kill all local background collectors (STM32, BW, etc.)
#     2. SSH to RPi and Jetson to kill all ROS2 nodes and collectors
#     3. Pull whatever partial CSVs already exist
#     4. Exit with code 1
#
# Usage:
#   bash ws_base/launch_poc_field.sh
#   bash ws_base/launch_poc_field.sh --duration 600
#   bash ws_base/launch_poc_field.sh --skip-stm32
#
# Output files (all on base PC under a single per-run directory):
#   ws_base/runs/single_domain/run_NNN/latency_rpi.csv
#   ws_base/runs/single_domain/run_NNN/latency_jetson.csv
#   ws_base/runs/single_domain/run_NNN/net_stats_rpi.csv
#   ws_base/runs/single_domain/run_NNN/net_stats_jetson.csv
#   ws_base/runs/single_domain/run_NNN/topic_bw.csv
#   ws_base/runs/single_domain/run_NNN/stm32_chassis.csv
#   ws_base/runs/single_domain/run_NNN/stm32_sensors.csv
#   ws_base/runs/single_domain/run_NNN/telemetry_relay.csv   ← base PC telemetry log
#   ws_base/runs/single_domain/run_NNN/cpu_rpi.csv
#   ws_base/runs/single_domain/run_NNN/cpu_jetson_tegrastats.txt
#   ws_base/runs/single_domain/run_NNN/softirq_rpi.csv
#   ws_base/runs/single_domain/run_NNN/softirq_jetson.csv
#   ws_base/runs/single_domain/run_NNN/logs/

# NOTE: set -e intentionally omitted — individual failures should not abort a
# live field run.  All critical failures use explicit exit via die().
set -uo pipefail

# ============================================================================
# Configuration — edit these if your network addresses change
# ============================================================================

RPI_HOST="curry@192.168.1.1"
JETSON_HOST="yupi@192.168.1.5"

CHASSIS_PORT="auto"
SENSORS_PORT="auto"

TOOLS_DIR="$HOME/almondmatcha/ws_base/tools"
WORKSPACE="$HOME/almondmatcha"

POC_RUN_BASE="$WORKSPACE/ws_base/runs/single_domain"
PARAMS_DIR="$WORKSPACE/ws_base/params/field"

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
        *) die "Unknown argument: $1" ;;
    esac
done

# ============================================================================
# Tracked background PID list — all local background processes
# ============================================================================
# Used by cleanup() to kill all local spawned processes cleanly.

BACKGROUND_PIDS=()
STM32_COLLECTOR_PID=""
TOPIC_BW_PID=""

# ============================================================================
# Helper — Stop ROS2 node tmux sessions on all machines
# ============================================================================

stop_ros2_nodes() {
    log "  Stopping ROS2 node tmux sessions on all machines..."

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
# Emergency cleanup — triggered on SIGINT (Ctrl-C), SIGTERM, or EXIT
# ============================================================================

_CLEANED_UP=false

cleanup() {
    if $_CLEANED_UP; then return; fi
    _CLEANED_UP=true

    echo ""
    echo -e "${RED}${BOLD}╔══════════════════════════════════════════════╗${NC}"
    echo -e "${RED}${BOLD}║         EMERGENCY STOP INITIATED             ║${NC}"
    echo -e "${RED}${BOLD}║  Stopping all nodes and collectors...        ║${NC}"
    echo -e "${RED}${BOLD}╚══════════════════════════════════════════════╝${NC}"
    echo ""

    # ── Kill all tracked local background processes ───────────────────────────
    for pid in "${BACKGROUND_PIDS[@]}"; do
        kill -TERM "$pid" 2>/dev/null || true
    done
    [[ -n "$STM32_COLLECTOR_PID" ]] && kill -TERM "$STM32_COLLECTOR_PID" 2>/dev/null || true
    [[ -n "$TOPIC_BW_PID"        ]] && kill -TERM "$TOPIC_BW_PID"        2>/dev/null || true

    # Give local processes a moment to flush before we continue
    sleep 1

    # Kill any remaining ros2-related processes on base PC
    pkill -TERM -f "ros2 run"         2>/dev/null || true
    pkill -TERM -f "collect_topic_bw" 2>/dev/null || true

    # ── Stop collectors and ROS2 nodes on RPi and Jetson ─────────────────────
    log "  Killing all remote processes on RPi and Jetson..."
    ssh $SSH_OPTS "$RPI_HOST" "
        pkill -TERM -f 'ros2 run'           2>/dev/null || true
        pkill -TERM -f 'collect_net_stats'  2>/dev/null || true
        pkill -TERM -f 'cpu_logger_rpi'     2>/dev/null || true
        pkill -TERM -f 'softirq_logger_rpi' 2>/dev/null || true
        pkill -TERM -f 'collect_latency'    2>/dev/null || true
        tmux kill-session -t rover_poc      2>/dev/null || true
    " 2>/dev/null || warn "  RPi kill had errors (non-fatal)"

    ssh $SSH_OPTS "$JETSON_HOST" "
        pkill -TERM -f 'ros2 run'              2>/dev/null || true
        pkill -TERM -f 'collect_net_stats'     2>/dev/null || true
        pkill -TERM -f 'cpu_logger_jetson'     2>/dev/null || true
        pkill -TERM -f 'softirq_logger_jetson' 2>/dev/null || true
        pkill -TERM -f 'collect_latency'       2>/dev/null || true
        pkill -TERM -f 'collect_topic_bw'      2>/dev/null || true
        pkill -TERM -f 'tegrastats'            2>/dev/null || true
        tmux kill-session -t jetson_poc        2>/dev/null || true
    " 2>/dev/null || warn "  Jetson kill had errors (non-fatal)"

    # ── Stop latency collectors and pull (best effort) ────────────────────────
    LOCAL_DEST_CSV="$RUN_DIR/latency_rpi.csv"        TARGET_HOST="$RPI_HOST"    TARGET_LABEL=rpi       SSH_OPTS="$SSH_OPTS" \
        bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh" 2>/dev/null || true
    LOCAL_DEST_CSV="$RUN_DIR/latency_jetson.csv"    TARGET_HOST="$JETSON_HOST" TARGET_LABEL=jetson    SSH_OPTS="$SSH_OPTS" \
        bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh" 2>/dev/null || true

    # ── Pull partial CSVs from SBCs ───────────────────────────────────────────
    [[ -n "${RUN_DIR:-}" ]] && {
        scp $SSH_OPTS "$RPI_HOST:~/ros2_traces/net_stats_rpi.csv"       "$RUN_DIR/net_stats_rpi.csv"          2>/dev/null || true
        scp $SSH_OPTS "$RPI_HOST:~/almondmatcha_poc/cpu_rpi.csv"        "$RUN_DIR/cpu_rpi.csv"                2>/dev/null || true
        scp $SSH_OPTS "$RPI_HOST:~/almondmatcha_poc/softirq_rpi.csv"    "$RUN_DIR/softirq_rpi.csv"            2>/dev/null || true
        scp $SSH_OPTS "$JETSON_HOST:~/ros2_traces/net_stats_jetson.csv" "$RUN_DIR/net_stats_jetson.csv"       2>/dev/null || true
        scp $SSH_OPTS "$JETSON_HOST:~/almondmatcha_poc/cpu_jetson_tegrastats.txt" \
                                                                         "$RUN_DIR/cpu_jetson_tegrastats.txt"  2>/dev/null || true
        scp $SSH_OPTS "$JETSON_HOST:~/almondmatcha_poc/softirq_jetson.csv" "$RUN_DIR/softirq_jetson.csv"      2>/dev/null || true
        # Telemetry relay CSV was written directly to RUN_DIR by the node
        [[ -f "$RUN_DIR/telemetry_relay.csv" ]] \
            && log "  Telemetry relay CSV saved: $RUN_DIR/telemetry_relay.csv" \
            || warn "  Telemetry relay CSV not found (monitoring node may not have started)"
    }

    # ── Close SSH control sockets ─────────────────────────────────────────────
    ssh $SSH_OPTS -O exit "$RPI_HOST"    2>/dev/null || true
    ssh $SSH_OPTS -O exit "$JETSON_HOST" 2>/dev/null || true
    rm -rf "$SSH_CONTROL_DIR"

    echo ""
    echo -e "${YELLOW}[$(date +%H:%M:%S)] Emergency stop complete.${NC}"
    echo -e "${YELLOW}  Partial CSVs saved to: ${RUN_DIR:-<not yet created>}${NC}"
    echo -e "${YELLOW}  All remote processes have been terminated.${NC}"
    echo ""
    exit 1
}

trap cleanup INT TERM EXIT

# ============================================================================
# Pre-flight checks
# ============================================================================

preflight() {
    log "Running pre-flight checks  [FIELD MODE]..."

    command -v tmux    &>/dev/null || die "tmux not installed"
    command -v ssh     &>/dev/null || die "ssh not installed"
    command -v python3 &>/dev/null || die "python3 not installed"
    python3 -c "import serial" 2>/dev/null || die "pyserial missing — run: pip3 install pyserial"

    [[ -d "$PARAMS_DIR" ]] || die "Field params directory not found: $PARAMS_DIR"
    for f in mission_command.yaml camera.yaml control.yaml; do
        [[ -f "$PARAMS_DIR/$f" ]] || die "Missing field params file: $PARAMS_DIR/$f"
    done
    ok "  Field params verified: $PARAMS_DIR"

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

    # SCP field params to Jetson.
    # Fallback: if SCP fails (e.g. slow link), check whether cached params already
    # exist on Jetson from a prior run.  If so, warn and continue — the operator
    # can abort manually if the cached params are stale.  If no cache exists either,
    # abort because running with unknown params is unsafe.
    log "  Uploading field params to Jetson..."
    ssh $SSH_OPTS "$JETSON_HOST" "mkdir -p ~/almondmatcha_poc/params/field" 2>/dev/null
    _scp_ok=true
    scp $SSH_OPTS "$PARAMS_DIR/camera.yaml"  "$JETSON_HOST:~/almondmatcha_poc/params/field/camera.yaml"  \
        || { warn "  SCP camera.yaml failed"; _scp_ok=false; }
    scp $SSH_OPTS "$PARAMS_DIR/control.yaml" "$JETSON_HOST:~/almondmatcha_poc/params/field/control.yaml" \
        || { warn "  SCP control.yaml failed"; _scp_ok=false; }
    if [[ "$_scp_ok" == false ]]; then
        # Check whether a cached copy already exists on Jetson
        if ssh $SSH_OPTS "$JETSON_HOST" \
            "test -f ~/almondmatcha_poc/params/field/camera.yaml && test -f ~/almondmatcha_poc/params/field/control.yaml" \
            2>/dev/null; then
            warn "  SCP failed — using cached params already on Jetson (from prior run)"
            warn "  *** Verify gains are correct before proceeding ***"
        else
            die "SCP failed and no cached params found on Jetson — cannot continue safely"
        fi
    else
        ok "  Field params uploaded to Jetson"
    fi

    mkdir -p "$RUN_DIR" "$LOG_DIR"

    log "  Run directory  : $RUN_DIR"
    log "  Field params   : $PARAMS_DIR"
    log "  Camera mode    : LIVE D415 ONLY (no fallback)"

    ok "All pre-flight checks passed"
}

# ============================================================================
# Step 0 — Clean stale DDS participants
# ============================================================================

clean_stale_participants() {
    log "Step 0 — Cleaning stale DDS participants on all machines"

    pkill -f "ros2 run" 2>/dev/null || true
    pkill -f "collect_topic_bw" 2>/dev/null || true
    tmux kill-session -t base_poc 2>/dev/null || true

    ssh $SSH_OPTS "$RPI_HOST" "
        pkill -f 'ros2 run' 2>/dev/null || true
        pkill -f 'collect_latency' 2>/dev/null || true
        pkill -f 'collect_net_stats' 2>/dev/null || true
        tmux kill-session -t rover_poc 2>/dev/null || true
    " 2>/dev/null || warn "  RPi cleanup had errors (non-fatal)"

    ssh $SSH_OPTS "$JETSON_HOST" "
        pkill -f 'ros2 run' 2>/dev/null || true
        pkill -f 'collect_latency' 2>/dev/null || true
        pkill -f 'collect_net_stats' 2>/dev/null || true
        pkill -f 'collect_topic_bw' 2>/dev/null || true
        tmux kill-session -t jetson_poc 2>/dev/null || true
    " 2>/dev/null || warn "  Jetson cleanup had errors (non-fatal)"

    sleep 3
    ok "All stale DDS participants cleaned"
}

# ============================================================================
# Step 1 — Start STM32 memory collector
# ============================================================================

start_stm32_collector() {
    if [[ "$SKIP_STM32" == true ]]; then
        warn "Step 1 — STM32 memory collector SKIPPED"
        return
    fi

    log "Step 1 — Starting STM32 memory collector"

    python3 "$TOOLS_DIR/collect_stm32_memory.py" \
        --chassis "$CHASSIS_PORT" \
        --sensors "$SENSORS_PORT" \
        --out     "$RUN_DIR/stm32" \
        >"$LOG_DIR/stm32_collector.log" 2>&1 &
    STM32_COLLECTOR_PID=$!
    BACKGROUND_PIDS+=("$STM32_COLLECTOR_PID")

    sleep 1
    if ! kill -0 "$STM32_COLLECTOR_PID" 2>/dev/null; then
        warn "STM32 collector exited immediately — check $LOG_DIR/stm32_collector.log"
        warn "Continuing without STM32 data (use --skip-stm32 to suppress)"
        STM32_COLLECTOR_PID=""
    else
        ok "STM32 collector running (PID $STM32_COLLECTOR_PID)"
    fi
}

# ============================================================================
# Step 2 — Launch ROS2 nodes
# ============================================================================

launch_ros2_nodes() {
    log "Step 2 — Launching ROS2 nodes  [FIELD params: $PARAMS_DIR]"
    log "  Camera: LIVE D415 ONLY — node will exit with error if D415 absent"

    log "  Launching rover nodes on RPi ($RPI_HOST)..."
    ssh $SSH_OPTS "$RPI_HOST" "SKIP_ATTACH=1 bash ~/almondmatcha/ws_rpi/launch_rover_single_domain.sh" \
        >"$LOG_DIR/launch_rpi.log" 2>&1 || warn "  RPi launch returned non-zero"
    sleep 20
    ok "  RPi launch sent"

    log "  Launching Jetson nodes with field camera+control params (live camera, no fallback)..."
    ssh $SSH_OPTS "$JETSON_HOST" "
        export SKIP_ATTACH=1
        export CAMERA_PARAMS_FILE=~/almondmatcha_poc/params/field/camera.yaml
        export CONTROL_PARAMS_FILE=~/almondmatcha_poc/params/field/control.yaml
        bash ~/almondmatcha/ws_jetson/launch_jetson_single_domain.sh
    " >"$LOG_DIR/launch_jetson.log" 2>&1 || warn "  Jetson launch returned non-zero"
    sleep 10
    ok "  Jetson launch sent (field params, camera-only)"

    log "  Launching base PC nodes with field mission params + telemetry CSV..."
    export MISSION_CMD_PARAMS="$PARAMS_DIR/mission_command.yaml"
    export TELEMETRY_CSV_PATH="$RUN_DIR/telemetry_relay.csv"
    SKIP_ATTACH=1 bash "$WORKSPACE/ws_base/launch_base_single_domain.sh" \
        >"$LOG_DIR/launch_base.log" 2>&1 || warn "  Base PC launch returned non-zero"
    unset MISSION_CMD_PARAMS TELEMETRY_CSV_PATH
    sleep 5
    ok "  Base PC launch sent (telemetry CSV: $RUN_DIR/telemetry_relay.csv)"
}

# ============================================================================
# Step 3 — Wait until BOTH STM32 topics are discovered and flowing
# ============================================================================

STM32_TOPICS=("/tpc_chassis_imu" "/tpc_chassis_sensors")

wait_stm32_topics() {
    log "Step 3 — Waiting for STM32 topics to be discovered and flowing"
    log "  Press Ctrl-C for emergency stop at any time"

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
    if [ -s "$fname" ]; then echo "OK"; head -3 "$fname"; else echo "=== TIMEOUT ==="; fi
done
rm -rf "$OUTDIR"
WAITEOF
    chmod +x "$echo_helper"

    local max_attempts=6
    for attempt in $(seq 1 $max_attempts); do
        local echo_timeout=$(( 6 + attempt * 6 ))
        local elapsed=$(( $(date +%s) - start_time ))
        log "  Attempt $attempt/$max_attempts (timeout ${echo_timeout}s, elapsed ${elapsed}s)..."

        local combined_output
        combined_output=$(bash "$echo_helper" "$WORKSPACE" "$echo_timeout" \
            "${STM32_TOPICS[@]}" 2>/dev/null) || true

        local all_ok=true
        for t in "${STM32_TOPICS[@]}"; do
            local block
            block=$(echo "$combined_output" | sed -n "\\#^=== ${t} ===#,\\#^===#p" | head -5)
            if echo "$block" | grep -q "^OK"; then
                ok "  $t confirmed flowing"
            else
                all_ok=false
            fi
        done

        if $all_ok; then
            ok "  All STM32 topics confirmed"
            rm -f "$echo_helper"
            return 0
        fi

        if (( attempt < max_attempts )); then
            local pending=""
            for t in "${STM32_TOPICS[@]}"; do
                block=$(echo "$combined_output" | sed -n "\\#^=== ${t} ===#,\\#^===#p" | head -5)
                echo "$block" | grep -q "^OK" || pending+=" $t"
            done
            warn "  Pending topics:${pending} — retrying in 3s  (Ctrl-C to abort)"
            sleep 3
        fi
    done

    rm -f "$echo_helper"
    warn "STM32 topics not confirmed after $max_attempts attempts"
    warn "Continuing anyway — data quality may be reduced (check stm32 serial output)"
}

# ============================================================================
# Step 4 — Start latency collectors
# ============================================================================

start_latency_collectors() {
    log "Step 4 — Starting latency collectors on RPi and Jetson"

    TARGET_HOST="$RPI_HOST"    TARGET_LABEL=rpi    SSH_OPTS="$SSH_OPTS" \
        bash "$TOOLS_DIR/tracing/start_trace.sh" || warn "  RPi latency collector failed to start"
    ok "  RPi latency collector started (D5)"

    TARGET_HOST="$JETSON_HOST" TARGET_LABEL=jetson SSH_OPTS="$SSH_OPTS" \
        bash "$TOOLS_DIR/tracing/start_trace.sh" || warn "  Jetson latency collector failed to start"
    ok "  Jetson latency collector started (D5)"
}

# ============================================================================
# Step 5 — Start per-topic bandwidth collector
# ============================================================================

start_topic_bw_collector() {
    log "Step 5 — Starting per-topic bandwidth collector (base PC: D5)"

    # D5 collector
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
    BACKGROUND_PIDS+=("$TOPIC_BW_PID")
    sleep 1
    if ! kill -0 "$TOPIC_BW_PID" 2>/dev/null; then
        warn "Topic BW (D5) collector exited immediately — check $LOG_DIR/topic_bw.log"
        TOPIC_BW_PID=""
    else
        ok "  D5 Topic BW collector running (PID $TOPIC_BW_PID)"
    fi
}

# ============================================================================
# Step 6 — Start net-stats, CPU, SoftIRQ collectors
# ============================================================================

start_net_collectors() {
    log "Step 6 — Starting network stats collectors on RPi and Jetson"

    remote_start_net() {
        local host="$1" out="$2" log_f="$3" pid_f="$4"
        ssh -T $SSH_OPTS "$host" "
            mkdir -p ~/ros2_traces
            setsid nohup python3 ~/almondmatcha/ws_base/tools/collect_net_stats.py \
                --out $out </dev/null >$log_f 2>&1 &
            disown
            echo \$! > $pid_f
        " 2>/dev/null || warn "  net_stats start failed on $host"
    }

    remote_start_net "$RPI_HOST"    "~/ros2_traces/net_stats_rpi.csv"    "~/ros2_traces/net_stats_rpi.log"    "~/ros2_traces/net_stats_rpi.pid"
    remote_start_net "$JETSON_HOST" "~/ros2_traces/net_stats_jetson.csv" "~/ros2_traces/net_stats_jetson.log" "~/ros2_traces/net_stats_jetson.pid"
    ok "  Net-stats collectors started"
}

start_cpu_collectors() {
    log "Step 6b — Starting CPU load loggers"

    ssh -T $SSH_OPTS "$RPI_HOST" '
        mkdir -p ~/almondmatcha_poc
        echo "timestamp,cpu_pct,mem_pct,temp_c" > ~/almondmatcha_poc/cpu_rpi.csv
        exec -a cpu_logger_rpi bash -c "
            while sleep 1; do
                echo \"\$(date +%s),\$(top -bn1 | grep \"Cpu(s)\" | awk \"{print 100-\\\$8}\"),\$(free | awk \"/Mem/{printf \\\"%.2f\\\",\\\$3/\\\$2*100}\"),\$(cat /sys/class/thermal/thermal_zone0/temp | awk \"{print \\\$1/1000}\")\"
            done >> ~/almondmatcha_poc/cpu_rpi.csv
        " </dev/null >~/almondmatcha_poc/cpu_rpi.log 2>&1 &
        disown; echo $!
    ' > "$LOG_DIR/cpu_rpi_pid.txt" 2>/dev/null || warn "  RPi CPU logger failed"
    ok "  RPi CPU logger started"

    ssh -T $SSH_OPTS "$JETSON_HOST" '
        mkdir -p ~/almondmatcha_poc
        exec -a cpu_logger_jetson bash -c "
            tegrastats --interval 1000 | while IFS= read -r line; do echo \"\$(date +%s) \$line\"; done
        " </dev/null > ~/almondmatcha_poc/cpu_jetson_tegrastats.txt 2>&1 &
        disown; echo $!
    ' > "$LOG_DIR/cpu_jetson_pid.txt" 2>/dev/null || warn "  Jetson CPU logger failed"
    ok "  Jetson CPU logger started"
}

start_softirq_collectors() {
    log "Step 6c — Starting SoftIRQ delta loggers"

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
        disown; echo \$!
    " > "$LOG_DIR/softirq_rpi_pid.txt" 2>/dev/null || warn "  RPi SoftIRQ logger failed"
    ok "  RPi SoftIRQ logger started"

    ssh -T $SSH_OPTS "$JETSON_HOST" "
        mkdir -p ~/almondmatcha_poc
        exec -a softirq_logger_jetson bash -c '${script//$'\n'/; }' -- ~/almondmatcha_poc/softirq_jetson.csv </dev/null >~/almondmatcha_poc/softirq_jetson.log 2>&1 &
        disown; echo \$!
    " > "$LOG_DIR/softirq_jetson_pid.txt" 2>/dev/null || warn "  Jetson SoftIRQ logger failed"
    ok "  Jetson SoftIRQ logger started"
}

# ============================================================================
# Step 7 — Wait for run duration (Ctrl-C = emergency stop)
# ============================================================================

wait_for_run() {
    local bar_width=40
    local DASH=7

    echo ""
    echo -e "${BOLD}=====================================================================${NC}"
    echo -e "${GREEN}  FIELD RUN IN PROGRESS — ${RUN_DURATION}s measurement window${NC}"
    echo -e "${BOLD}=====================================================================${NC}"
    echo ""
    echo "  Mode          : ON-FIELD (live D415 camera, no fallback)"
    echo "  Params dir    : $PARAMS_DIR"
    echo "  Run directory : $RUN_DIR"
    echo ""
    echo -e "  ${RED}${BOLD}Ctrl-C = EMERGENCY STOP${NC} — cleanly stops all nodes and collectors"
    echo ""

    for (( i=0; i<DASH; i++ )); do echo ""; done

    local elapsed=0 bar filled empty pct i
    local stm32_status bw_status telem_rows

    while (( elapsed < RUN_DURATION )); do
        sleep 1
        elapsed=$(( elapsed + 1 ))

        filled=$(( elapsed * bar_width / RUN_DURATION ))
        empty=$(( bar_width - filled ))
        bar=""
        for (( i=0; i<filled; i++ )); do bar+="█"; done
        for (( i=0; i<empty;  i++ )); do bar+="░"; done
        pct=$(( elapsed * 100 / RUN_DURATION ))

        telem_rows=$(wc -l < "$RUN_DIR/telemetry_relay.csv" 2>/dev/null || echo 1)
        telem_rows=$(( telem_rows - 1 ))
        (( telem_rows < 0 )) && telem_rows=0

        if [[ -n "$STM32_COLLECTOR_PID" ]]; then
            kill -0 "$STM32_COLLECTOR_PID" 2>/dev/null \
                && stm32_status="\033[0;32m● running\033[0m" \
                || stm32_status="\033[0;31m● DIED\033[0m"
        else
            stm32_status="\033[1;33m● skipped\033[0m"
        fi
        if [[ -n "${TOPIC_BW_PID:-}" ]]; then
            kill -0 "$TOPIC_BW_PID" 2>/dev/null \
                && bw_status="\033[0;32m● running\033[0m" \
                || bw_status="\033[0;31m● DIED\033[0m"
        else
            bw_status="\033[1;33m● failed\033[0m"
        fi

        printf "\033[%dA\r" "$DASH"
        printf "  Progress:%-61s\n" ""
        printf "  [%-${bar_width}s] %3d%%  %ds / %ds%-20s\n" "$bar" "$pct" "$elapsed" "$RUN_DURATION" ""
        printf "%-70s\n" ""
        printf "  Telemetry relay rows: %-8d (tpc_telemetry_relay @ ~5 Hz)%-7s\n" "$telem_rows" ""
        printf "  Collectors:%-59s\n" ""
        printf "    STM32 %b  BW %b  Ctrl-C = EMERGENCY STOP              \n" \
               "$stm32_status" "$bw_status"
        printf "%-70s\n" ""
    done

    echo ""
    log "Run duration complete — collecting data..."
}

# ============================================================================
# Step 8 — Stop collectors and pull CSVs
# ============================================================================

stop_and_collect() {
    log "Step 8 — Stopping all collectors and pulling CSVs"

    mkdir -p "$LOG_DIR/rpi" "$LOG_DIR/jetson"

    [[ -n "$STM32_COLLECTOR_PID" ]] && kill "$STM32_COLLECTOR_PID" 2>/dev/null || true
    [[ -n "${TOPIC_BW_PID:-}"    ]] && kill "$TOPIC_BW_PID"        2>/dev/null || true
    sleep 0.5
    local f
    f=$(ls "$RUN_DIR"/stm32_chassis_*.csv 2>/dev/null | tail -1) && [[ -n "$f" ]] && mv "$f" "$RUN_DIR/stm32_chassis.csv" || true
    f=$(ls "$RUN_DIR"/stm32_sensors_*.csv 2>/dev/null | tail -1) && [[ -n "$f" ]] && mv "$f" "$RUN_DIR/stm32_sensors.csv" || true
    ok "  Local collectors stopped"

    # Log telemetry CSV status
    local trows
    trows=$(wc -l < "$RUN_DIR/telemetry_relay.csv" 2>/dev/null || echo 1)
    trows=$(( trows - 1 ))
    ok "  Telemetry relay CSV: $RUN_DIR/telemetry_relay.csv  ($trows rows)"
    # Redundant copy to LOG_DIR
    cp "$RUN_DIR/telemetry_relay.csv" "$LOG_DIR/telemetry_relay_backup.csv" 2>/dev/null || true

    log "  Stopping and pulling from RPi and Jetson in parallel..."

    (
        LOCAL_DEST_CSV="$RUN_DIR/latency_rpi.csv" TARGET_HOST="$RPI_HOST" \
            TARGET_LABEL=rpi SSH_OPTS="$SSH_OPTS" \
            bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh" || true

        ssh -T $SSH_OPTS "$RPI_HOST" "
            if [ -f ~/ros2_traces/net_stats_rpi.pid ]; then
                kill \$(cat ~/ros2_traces/net_stats_rpi.pid) 2>/dev/null || true
                rm -f ~/ros2_traces/net_stats_rpi.pid
            fi
            pkill -f cpu_logger_rpi     2>/dev/null || true
            pkill -f softirq_logger_rpi 2>/dev/null || true
        " 2>/dev/null || true
        scp $SSH_OPTS "$RPI_HOST:~/ros2_traces/net_stats_rpi.csv"       "$RUN_DIR/net_stats_rpi.csv"  2>/dev/null || true
        scp $SSH_OPTS "$RPI_HOST:~/almondmatcha_poc/cpu_rpi.csv"        "$RUN_DIR/cpu_rpi.csv"        2>/dev/null || true
        scp $SSH_OPTS "$RPI_HOST:~/almondmatcha_poc/softirq_rpi.csv"    "$RUN_DIR/softirq_rpi.csv"    2>/dev/null || true
        scp $SSH_OPTS "$RPI_HOST:~/ros2_traces/poc_*.log"               "$LOG_DIR/rpi/"              2>/dev/null || true
        echo "[parallel-rpi] done"
    ) &
    RPI_PULL_PID=$!

    (
        LOCAL_DEST_CSV="$RUN_DIR/latency_jetson.csv" TARGET_HOST="$JETSON_HOST" \
            TARGET_LABEL=jetson SSH_OPTS="$SSH_OPTS" \
            bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh" || true

        ssh -T $SSH_OPTS "$JETSON_HOST" "
            if [ -f ~/ros2_traces/net_stats_jetson.pid ]; then
                kill \$(cat ~/ros2_traces/net_stats_jetson.pid) 2>/dev/null || true
                rm -f ~/ros2_traces/net_stats_jetson.pid
            fi
            pkill -f cpu_logger_jetson     2>/dev/null || true
            pkill -f softirq_logger_jetson 2>/dev/null || true
            pkill -f tegrastats             2>/dev/null || true
        " 2>/dev/null || true
        scp $SSH_OPTS "$JETSON_HOST:~/ros2_traces/net_stats_jetson.csv"           "$RUN_DIR/net_stats_jetson.csv"       2>/dev/null || true
        scp $SSH_OPTS "$JETSON_HOST:~/almondmatcha_poc/cpu_jetson_tegrastats.txt" "$RUN_DIR/cpu_jetson_tegrastats.txt"  2>/dev/null || true
        scp $SSH_OPTS "$JETSON_HOST:~/almondmatcha_poc/softirq_jetson.csv"        "$RUN_DIR/softirq_jetson.csv"         2>/dev/null || true
        scp $SSH_OPTS "$JETSON_HOST:~/ros2_traces/poc_*.log"                      "$LOG_DIR/jetson/"                    2>/dev/null || true
        echo "[parallel-jetson] done"
    ) &
    JETSON_PULL_PID=$!

    wait "$RPI_PULL_PID"    && ok "  RPi:    data pulled" || warn "  RPi pull had errors"
    wait "$JETSON_PULL_PID" && ok "  Jetson: data pulled" || warn "  Jetson pull had errors"

    stop_ros2_nodes
    ssh $SSH_OPTS -O exit "$RPI_HOST"    2>/dev/null || true
    ssh $SSH_OPTS -O exit "$JETSON_HOST" 2>/dev/null || true
    rm -rf "$SSH_CONTROL_DIR"
}

# ============================================================================
# Step 9b — Post-run analysis
# ============================================================================

run_post_analysis() {
    log "Step 9b — Running post-run analysis..."
    bash "$TOOLS_DIR/post_run.sh" "$RUN_DIR" \
        && ok "  Post-run analysis complete" \
        || warn "  post_run.sh had errors — re-run manually: bash ws_base/tools/post_run.sh $RUN_DIR"
}

# ============================================================================
# Step 9 — Print summary
# ============================================================================

print_summary() {
    echo ""
    echo -e "${BOLD}=====================================================================${NC}"
    echo -e "${GREEN}  FIELD RUN COMPLETE${NC}"
    echo -e "${BOLD}=====================================================================${NC}"
    echo ""
    echo "  Run directory : $RUN_DIR"
    echo "  Mode: ON-FIELD | Branch: single-domain | Domain: D5 | Duration: ${RUN_DURATION}s"
    echo ""
    echo "  Raw CSVs:"
    for f in latency_rpi.csv latency_jetson.csv net_stats_rpi.csv net_stats_jetson.csv \
              topic_bw.csv stm32_chassis.csv stm32_sensors.csv telemetry_relay.csv \
              cpu_rpi.csv softirq_rpi.csv softirq_jetson.csv; do
        [[ -f "$RUN_DIR/$f" ]] && echo "    $RUN_DIR/$f"
    done
    echo ""
    echo "  Adjust gains for next run:"
    echo "    nano $PARAMS_DIR/control.yaml"
    echo ""
    echo "  Re-run post-processing:"
    echo "    bash ws_base/tools/post_run.sh $RUN_DIR"
    echo ""
}

# ============================================================================
# Main
# ============================================================================

main() {
    # Disarm EXIT trap for normal completion
    trap - EXIT

    echo ""
    echo -e "${BOLD}  Single-Domain POC — ON-FIELD MODE${NC}"
    echo -e "${RED}  Camera: LIVE D415 ONLY (no video fallback)${NC}"
    echo -e "  Params: $PARAMS_DIR"
    echo -e "  Branch: single-domain | Domain: D5 | Duration: ${RUN_DURATION}s"
    echo -e "  ${RED}${BOLD}Ctrl-C at any time = EMERGENCY STOP${NC}"
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
