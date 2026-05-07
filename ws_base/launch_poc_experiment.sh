#!/bin/bash
# launch_poc_experiment.sh — Full POC experiment launcher (run on base PC)
#
# Executes the complete single-domain POC measurement sequence in order:
#   1. Clean stale DDS participants on all machines
#   2. Start STM32 memory collector (background)
#   3. Launch ROS2 nodes on RPi, Jetson, and base PC
#   4. Wait for STM32 topics to be discovered and flowing
#   5. Start latency collectors on RPi and Jetson
#   6. Start net-stats collectors on RPi and Jetson (background SSH)
#   6b. Start CPU load loggers on RPi and Jetson (background SSH)
#   6c. Start SoftIRQ delta loggers on RPi and Jetson (background SSH)
#   7. Wait for the run duration (with mid-run ros2 topic hz report)
#   8. Stop all collectors and pull CSVs to base PC
#
# STM32 boards are assumed to be always powered on. No reset step is needed.
#
# Usage:
#   bash ws_base/launch_poc_experiment.sh
#   bash ws_base/launch_poc_experiment.sh --duration 600   # 10-minute run (default: 300s)
#   bash ws_base/launch_poc_experiment.sh --skip-launch    # skip ROS2 node launch (collectors only)
#   bash ws_base/launch_poc_experiment.sh --skip-stm32     # skip STM32 serial collection
#
# Output files (all on base PC under a single per-run directory):
#   ws_base/runs/single_domain/run_NNN/latency_rpi.csv
#   ws_base/runs/single_domain/run_NNN/latency_jetson.csv
#   ws_base/runs/single_domain/run_NNN/net_stats_rpi.csv
#   ws_base/runs/single_domain/run_NNN/net_stats_jetson.csv
#   ws_base/runs/single_domain/run_NNN/topic_bw.csv
#   ws_base/runs/single_domain/run_NNN/stm32_chassis.csv
#   ws_base/runs/single_domain/run_NNN/stm32_sensors.csv
#   ws_base/runs/single_domain/run_NNN/cpu_rpi.csv
#   ws_base/runs/single_domain/run_NNN/cpu_jetson_tegrastats.txt
#   ws_base/runs/single_domain/run_NNN/softirq_rpi.csv
#   ws_base/runs/single_domain/run_NNN/softirq_jetson.csv
#   ws_base/runs/single_domain/run_NNN/hz_report_rpi.txt
#   ws_base/runs/single_domain/run_NNN/hz_report_jetson.txt
#   ws_base/runs/single_domain/run_NNN/merged_all.csv    ← time-bucketed union of all above
#   ws_base/runs/single_domain/run_NNN/logs/             ← all sub-process logs

set -euo pipefail

# ============================================================================
# Configuration — edit these if your network addresses change
# ============================================================================

RPI_HOST="curry@192.168.1.1"
JETSON_HOST="yupi@192.168.1.5"

# STM32 serial ports — leave as 'auto' to let collect_stm32_memory.py detect them.
# Override with --chassis / --sensors if auto-detect picks the wrong mapping.
CHASSIS_PORT="auto"
SENSORS_PORT="auto"

TOOLS_DIR="$HOME/almondmatcha/ws_base/tools"
WORKSPACE="$HOME/almondmatcha"

# All experiment output lands in a single numbered run directory.
# run_NNN is auto-incremented — each launch creates the next available number.
# Data is separated by branch: runs/single_domain/ vs runs/multi_domain/
# so switching branches doesn't overwrite or mix experiment data.
POC_RUN_BASE="$WORKSPACE/ws_base/runs/single_domain"

_next_run_dir() {
    local last
    last=$(ls -d "${POC_RUN_BASE}"/run_* 2>/dev/null \
           | grep -oP 'run_\K[0-9]+' | sort -n | tail -1)
    # Strip leading zeros to prevent bash interpreting as octal (008 is invalid octal)
    last=$(( 10#${last:-0} ))
    printf '%s/run_%03d' "$POC_RUN_BASE" "$(( last + 1 ))"
}

RUN_DIR="$(_next_run_dir)"
LOG_DIR="$RUN_DIR/logs"

RUN_DURATION=300   # seconds — default 5 minutes
SKIP_LAUNCH=false
SKIP_STM32=false

# SSH ControlMaster: authenticate once, reuse connection for all background ssh calls
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
pause(){ echo -e "${YELLOW}$*${NC}"; read -rp "    Press ENTER when ready... " _; }

# ============================================================================
# Argument parsing
# ============================================================================

while [[ $# -gt 0 ]]; do
    case "$1" in
        --duration)  RUN_DURATION="$2"; shift 2 ;;
        --skip-launch) SKIP_LAUNCH=true; shift ;;
        --skip-stm32)  SKIP_STM32=true; shift ;;
        --chassis)   CHASSIS_PORT="$2"; shift 2 ;;
        --sensors)   SENSORS_PORT="$2"; shift 2 ;;
        *) die "Unknown argument: $1" ;;
    esac
done

# ============================================================================
# Cleanup handler — kill background jobs on Ctrl-C or exit
# ============================================================================

STM32_COLLECTOR_PID=""
TOPIC_BW_PID=""
CPU_RPI_LOGPID=""
CPU_JETSON_LOGPID=""
SOFTIRQ_RPI_LOGPID=""
SOFTIRQ_JETSON_LOGPID=""

cleanup() {
    echo ""
    warn "Interrupted — stopping all background collectors..."
    [[ -n "$STM32_COLLECTOR_PID"    ]] && kill "$STM32_COLLECTOR_PID"    2>/dev/null || true
    [[ -n "$TOPIC_BW_PID"           ]] && kill "$TOPIC_BW_PID"           2>/dev/null || true
    # Stop CPU / SoftIRQ loggers on SBCs
    ssh $SSH_OPTS "$RPI_HOST"    "pkill -f 'cpu_logger_rpi\|softirq_logger_rpi'   2>/dev/null || true" 2>/dev/null || true
    ssh $SSH_OPTS "$JETSON_HOST" "pkill -f 'cpu_logger_jetson\|softirq_logger_jet' 2>/dev/null || true" 2>/dev/null || true
    ssh $SSH_OPTS "$RPI_HOST"    "pkill -f tegrastats 2>/dev/null || true" 2>/dev/null || true
    ssh $SSH_OPTS "$JETSON_HOST" "pkill -f tegrastats 2>/dev/null || true" 2>/dev/null || true
    # Stop latency collectors (cleanup path — best effort)
    LOCAL_DEST_CSV="$RUN_DIR/latency_rpi.csv"        TARGET_HOST="$RPI_HOST"    TARGET_LABEL=rpi        SSH_OPTS="$SSH_OPTS" \
        bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh" 2>/dev/null || true
    LOCAL_DEST_CSV="$RUN_DIR/latency_jetson.csv"     TARGET_HOST="$JETSON_HOST" TARGET_LABEL=jetson     SSH_OPTS="$SSH_OPTS" \
        bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh" 2>/dev/null || true
    # Stop net-stats, CPU, SoftIRQ collectors on SBCs (prevent orphaned processes)
    ssh $SSH_OPTS "$RPI_HOST"    "pkill -f 'collect_net_stats' 2>/dev/null || true" 2>/dev/null || true
    ssh $SSH_OPTS "$JETSON_HOST" "pkill -f 'collect_net_stats' 2>/dev/null || true" 2>/dev/null || true
    ssh $SSH_OPTS "$RPI_HOST"    "pkill -f 'cpu_logger_rpi\|softirq_logger_rpi' 2>/dev/null || true" 2>/dev/null || true
    ssh $SSH_OPTS "$JETSON_HOST" "pkill -f 'cpu_logger_jetson\|softirq_logger_jet\|tegrastats' 2>/dev/null || true" 2>/dev/null || true
    # Close SSH control sockets
    ssh $SSH_OPTS -O exit "$RPI_HOST"    2>/dev/null || true
    ssh $SSH_OPTS -O exit "$JETSON_HOST" 2>/dev/null || true
    rm -rf "$SSH_CONTROL_DIR"
    echo ""
    log "Cleanup complete. CSVs may be partial."
    exit 1
}
trap cleanup INT TERM

# ============================================================================
# Pre-flight checks
# ============================================================================

preflight() {
    log "Running pre-flight checks..."

    command -v tmux    &>/dev/null || die "tmux not installed"
    command -v ssh     &>/dev/null || die "ssh not installed"
    command -v python3 &>/dev/null || die "python3 not installed"
    python3 -c "import serial" 2>/dev/null || die "pyserial missing — run: pip3 install pyserial"

    # Verify custom message types are importable by the base PC Python environment.
    # collect_topic_bw.py silently skips topics whose types cannot be loaded; if
    # msgs_ifaces is missing here every STM32/GNSS/chassis topic produces a WARN
    # and is absent from topic_bw.csv for the entire run.
    bash -c "
        source /opt/ros/humble/setup.bash
        source '$WORKSPACE/ws_base/install/setup.bash'
        source '$WORKSPACE/common_ifaces/install/setup.bash'
        python3 -c \"from rosidl_runtime_py.utilities import get_message; get_message('msgs_ifaces/msg/SpresenseGNSS')\"
    " 2>/dev/null \
        || die "msgs_ifaces not importable — rebuild common_ifaces: cd $WORKSPACE/common_ifaces && colcon build"
    ok "  msgs_ifaces type support verified"

    # STM32 serial port check
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
                log "  Verify with: minicom -b 115200 -D $CHASSIS_PORT  (should show 'chassis' JSON)"
            elif (( port_count == 1 )); then
                warn "  Only one /dev/ttyACM* found: $detected — need two for both STM32 boards"
                warn "  Run with --skip-stm32 to skip STM32 collection, or plug in both boards"
                die "STM32 serial port auto-detect failed"
            else
                warn "  No /dev/ttyACM* ports found — STM32 boards not connected?"
                warn "  Run with --skip-stm32 to skip STM32 collection"
                die "STM32 serial port auto-detect failed"
            fi
        else
            [[ -e "$CHASSIS_PORT" ]] || die "Chassis serial port $CHASSIS_PORT not found. Is the board plugged in?"
            [[ -e "$SENSORS_PORT" ]] || die "Sensors serial port $SENSORS_PORT not found. Is the board plugged in?"
            ok "  STM32 ports: chassis=$CHASSIS_PORT  sensors=$SENSORS_PORT"
        fi
    else
        warn "  --skip-stm32: STM32 memory collection will be skipped"
    fi

    # Open persistent SSH control sockets — prompts for password/passphrase once per host.
    # All subsequent SSH calls in this script reuse these sockets without re-authenticating.
    log "  Authenticating to RPi ($RPI_HOST) — enter password/passphrase if prompted:"
    ssh $SSH_OPTS -o ConnectTimeout=10 "$RPI_HOST" true \
        || die "Cannot SSH to RPi ($RPI_HOST) — check connection and credentials"
    ok "  RPi authenticated"

    log "  Authenticating to Jetson ($JETSON_HOST) — enter password/passphrase if prompted:"
    ssh $SSH_OPTS -o ConnectTimeout=10 "$JETSON_HOST" true \
        || die "Cannot SSH to Jetson ($JETSON_HOST) — check connection and credentials"
    ok "  Jetson authenticated"

    mkdir -p "$RUN_DIR" "$LOG_DIR"

    log "  Run directory   : $RUN_DIR"

    ok "All pre-flight checks passed"
}

# ============================================================================
# Step 0 — Clean stale DDS participants on ALL machines
# ============================================================================
# WHY: zombie ros2 nodes / collector processes from a previous run create DDS
#      participants that occupy the STM32's limited participant table (MAX=20).
#      If stale participants are present when the boards boot, the STM32 may
#      refuse to accept new (real) participants, causing topic discovery failure.
#      The ros2 daemon is intentionally NOT killed — it must keep running with
#      FASTRTPS_DEFAULT_PROFILES_FILE already set from the user's shell.

clean_stale_participants() {
    log "Step 0 — Cleaning stale DDS participants on all machines"

    # ── Base PC cleanup ────────────────────────────────────────────────────────
    log "  Cleaning base PC..."
    pkill -f "ros2 run" 2>/dev/null || true
    pkill -f "collect_topic_bw" 2>/dev/null || true
    tmux kill-session -t base_poc 2>/dev/null || true
    ok "  Base PC cleaned"

    # ── RPi cleanup (over SSH) ─────────────────────────────────────────────────
    log "  Cleaning RPi ($RPI_HOST)..."
    ssh $SSH_OPTS "$RPI_HOST" "
        pkill -f 'ros2 run' 2>/dev/null || true
        pkill -f 'collect_latency' 2>/dev/null || true
        pkill -f 'collect_net_stats' 2>/dev/null || true
        tmux kill-session -t rover_poc 2>/dev/null || true
    " 2>/dev/null || warn "  RPi cleanup had errors (non-fatal)"
    ok "  RPi cleaned"

    # ── Jetson cleanup (over SSH) ──────────────────────────────────────────────
    log "  Cleaning Jetson ($JETSON_HOST)..."
    ssh $SSH_OPTS "$JETSON_HOST" "
        pkill -f 'ros2 run' 2>/dev/null || true
        pkill -f 'collect_latency' 2>/dev/null || true
        pkill -f 'collect_net_stats' 2>/dev/null || true
        pkill -f 'collect_topic_bw' 2>/dev/null || true
        tmux kill-session -t jetson_poc 2>/dev/null || true
    " 2>/dev/null || warn "  Jetson cleanup had errors (non-fatal)"
    ok "  Jetson cleaned"

    # Let DDS process participant departures so STM32 boards (if already running)
    # can free slots before we reset them.
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
    log "Step 2 — Launching ROS2 nodes"
    log "  (launch output redirected to $LOG_DIR/launch_<host>.log)"

    log "  Launching rover nodes on RPi ($RPI_HOST)..."
    ssh $SSH_OPTS "$RPI_HOST" "SKIP_ATTACH=1 bash ~/almondmatcha/ws_rpi/launch_rover_single_domain.sh" \
        >"$LOG_DIR/launch_rpi.log" 2>&1 &
    # RPi launches 8 nodes with 2 s stagger each (≈16 s total).
    # Wait long enough for all RPi nodes to finish their SPDP announcements
    # before adding Jetson participants — prevents overlapping discovery bursts
    # that would saturate the STM32 thread pool queue (40 slots).
    sleep 20
    ok "  RPi launch sent (8 nodes, 2 s stagger = ~16 s)"

    log "  Launching Jetson nodes ($JETSON_HOST)..."
    ssh $SSH_OPTS "$JETSON_HOST" "SKIP_ATTACH=1 bash ~/almondmatcha/ws_jetson/launch_jetson_single_domain.sh" \
        >"$LOG_DIR/launch_jetson.log" 2>&1 &
    # Jetson launches 4 nodes with 3+2+2 s stagger (≈7 s total).
    # Wait for all Jetson nodes to announce before adding base PC nodes.
    sleep 10
    ok "  Jetson launch sent (4 nodes, 3+2+2 s stagger = ~7 s)"

    log "  Launching base PC nodes..."
    SKIP_ATTACH=1 bash "$WORKSPACE/ws_base/launch_base_single_domain.sh" \
        >"$LOG_DIR/launch_base.log" 2>&1 &
    # Base PC launches 2 nodes with minimal delay.
    # Give them time to announce and settle before probing STM32 topics.
    sleep 5
    ok "  Base PC launch sent (2 nodes)"

    log "  All ROS2 nodes launched — D5 Linux participants visible to STM32 discovery"
    log "  Total: 14 Linux nodes staggered over ~35 s"
}

# ============================================================================
# Step 3 — Wait until BOTH STM32 topics are discovered and flowing
# ============================================================================
# WHY: embeddedRTPS on the STM32 has static participant tables (MAX=20).
#      SEDP matching may take 5-30s depending on how many Linux participants
#      the STM32 must process.  Starting collectors before topics are confirmed
#      wastes the measurement window on "waiting for STM32 data" messages.
#
# HOW: Check both topics in parallel using ros2 topic echo --once.
#      Each attempt creates background processes for all topics simultaneously,
#      sharing a single discovery window instead of sequential per-topic
#      ros2 topic hz calls that would create separate ephemeral DDS participants
#      and churn the STM32's already-busy discovery queues.

STM32_TOPICS=("/tpc_chassis_imu" "/tpc_chassis_sensors")

wait_stm32_topics() {
    log "Step 3 — Waiting for STM32 topics to be discovered and flowing"
    log "  Required topics: ${STM32_TOPICS[*]}"
    log "  Retries with increasing timeout — press Ctrl+C to abort"

    local start_time
    start_time=$(date +%s)

    # Create a helper script that checks all topics in parallel.
    # Each ros2 topic echo runs in the background so all topics share one
    # SPDP→SEDP discovery cycle, avoiding sequential participant churn on
    # the STM32 boards.
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

    # Up to 6 attempts with increasing timeout: 12/18/24/30/36/42s
    # These timeouts are longer than check_connectivity because during a
    # full POC launch the STM32 is processing SPDP/SEDP from 14+ new
    # Linux participants simultaneously.
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
            else
                all_ok=false
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
    log "Step 4 — Starting latency/jitter collectors on RPi and Jetson (after STM32 topics confirmed)"

    TARGET_HOST="$RPI_HOST"    TARGET_LABEL=rpi    SSH_OPTS="$SSH_OPTS" \
        bash "$TOOLS_DIR/tracing/start_trace.sh"
    ok "  RPi latency collector started"

    TARGET_HOST="$JETSON_HOST" TARGET_LABEL=jetson SSH_OPTS="$SSH_OPTS" \
        bash "$TOOLS_DIR/tracing/start_trace.sh"
    ok "  Jetson latency collector started"
}

# ============================================================================
# Step 5 — Start per-topic bandwidth collector (base PC, local)
# ============================================================================

start_topic_bw_collector() {
    log "Step 5 — Starting per-topic bandwidth collector (base PC, D5)"

    log "  Output: $RUN_DIR/topic_bw.csv"
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
        || die "Topic BW collector exited immediately — check $LOG_DIR/topic_bw.log"
    ok "  Topic BW collector running (PID $TOPIC_BW_PID)"
}

# ============================================================================
# Step 6 — Start net-stats collectors on RPi and Jetson (SSH + background)
# ============================================================================

start_net_collectors() {
    log "Step 6 — Starting network stats collectors on RPi and Jetson"

    # -T: disable PTY allocation — SSH exits as soon as the remote shell exits.
    # Without -T, SSH holds the connection open waiting for the PTY to be fully
    # released, which never happens while the backgrounded python is running.
    # disown: removes the job from the shell's job table before exit, preventing
    # the shell from waiting for or signalling the child on exit.
    remote_start_net() {
        local host="$1" out="$2" log_f="$3" pid_f="$4"
        # -T: no PTY → SSH closes as soon as remote shell exits (no PTY to keep open)
        # setsid: child gets its own session + no controlling terminal at all
        # disown: removes job from shell table before exit (belt and suspenders)
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

    remote_start_net "$RPI_HOST" \
        "~/ros2_traces/net_stats_rpi.csv" \
        "~/ros2_traces/net_stats_rpi.log" \
        "~/ros2_traces/net_stats_rpi.pid"

    remote_start_net "$JETSON_HOST" \
        "~/ros2_traces/net_stats_jetson.csv" \
        "~/ros2_traces/net_stats_jetson.log" \
        "~/ros2_traces/net_stats_jetson.pid"

    ok "  Net-stats collectors started on both SBCs"
}

# ============================================================================
# Step 6b — Start CPU load loggers on RPi and Jetson
# ============================================================================
# RPi:    top-based one-liner → cpu_rpi.csv   (timestamp,cpu_pct,mem_pct,temp_c)
# Jetson: tegrastats raw log  → cpu_jetson_tegrastats.txt  (prepended UNIX ts)
# Both logged at 1-second intervals via background remote shell loops.
# A named setsid wrapper is used so pkill can target exactly these processes.

start_cpu_collectors() {
    log "Step 6b — Starting CPU load loggers on RPi and Jetson"

    # ── RPi ───────────────────────────────────────────────────────────────────
    ssh -T $SSH_OPTS "$RPI_HOST" '
        mkdir -p ~/almondmatcha_poc
        echo "timestamp,cpu_pct,mem_pct,temp_c" > ~/almondmatcha_poc/cpu_rpi.csv
        # Named via exec so pkill -f cpu_logger_rpi matches exactly this shell
        exec -a cpu_logger_rpi bash -c "
            while sleep 1; do
                echo \"\$(date +%s),\$(top -bn1 | grep \"Cpu(s)\" | awk \"{print 100-\\\$8}\"),\$(free | awk \"/Mem/{printf \\\"%.2f\\\",\\\$3/\\\$2*100}\"),\$(cat /sys/class/thermal/thermal_zone0/temp | awk \"{print \\\$1/1000}\")\"
            done >> ~/almondmatcha_poc/cpu_rpi.csv
        " </dev/null >~/almondmatcha_poc/cpu_rpi.log 2>&1 &
        disown
        echo \$!
    ' > "$LOG_DIR/cpu_rpi_pid.txt" 2>/dev/null || true
    ok "  RPi CPU logger started"

    # ── Jetson ────────────────────────────────────────────────────────────────
    ssh -T $SSH_OPTS "$JETSON_HOST" '
        mkdir -p ~/almondmatcha_poc
        # Named via exec so pkill -f cpu_logger_jetson matches exactly this shell
        exec -a cpu_logger_jetson bash -c "
            tegrastats --interval 1000 | \
            while IFS= read -r line; do echo \"\$(date +%s) \$line\"; done
        " </dev/null > ~/almondmatcha_poc/cpu_jetson_tegrastats.txt 2>&1 &
        disown
        echo \$!
    ' > "$LOG_DIR/cpu_jetson_pid.txt" 2>/dev/null || true
    ok "  Jetson CPU logger (tegrastats) started"
}

# ============================================================================
# Step 6c — Start SoftIRQ delta loggers on RPi and Jetson
# ============================================================================
# Polls /proc/softirqs at 1-second intervals and records per-second deltas
# for NET_RX, NET_TX, SCHED, TIMER across all CPU columns.
# High NET_RX_delta without proportional rx_bps increase = demux overhead.

start_softirq_collectors() {
    log "Step 6c — Starting SoftIRQ delta loggers on RPi and Jetson"

    # Shared remote script body — same for both hosts
    # $1 = output CSV path; process is named softirq_logger_<tag> for pkill
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

    # ── RPi ───────────────────────────────────────────────────────────────────
    ssh -T $SSH_OPTS "$RPI_HOST" "
        mkdir -p ~/almondmatcha_poc
        exec -a softirq_logger_rpi bash -c '${script//$'\n'/; }' -- ~/almondmatcha_poc/softirq_rpi.csv </dev/null >~/almondmatcha_poc/softirq_rpi.log 2>&1 &
        disown
        echo \$!
    " > "$LOG_DIR/softirq_rpi_pid.txt" 2>/dev/null || true
    ok "  RPi SoftIRQ logger started"

    # ── Jetson ────────────────────────────────────────────────────────────────
    ssh -T $SSH_OPTS "$JETSON_HOST" "
        mkdir -p ~/almondmatcha_poc
        exec -a softirq_logger_jetson bash -c '${script//$'\n'/; }' -- ~/almondmatcha_poc/softirq_jetson.csv </dev/null >~/almondmatcha_poc/softirq_jetson.log 2>&1 &
        disown
        echo \$!
    " > "$LOG_DIR/softirq_jetson_pid.txt" 2>/dev/null || true
    ok "  Jetson SoftIRQ logger started"
}

# ============================================================================
# Step 7 — Wait for the run duration
# ============================================================================

wait_for_run() {
    local bar_width=40
    local DASH=9  # lines in the live-refresh dashboard block

    echo ""
    echo -e "${BOLD}======================================================${NC}"
    echo -e "${GREEN}  EXPERIMENT RUNNING — ${RUN_DURATION}s measurement window${NC}"
    echo -e "${BOLD}======================================================${NC}"
    echo ""
    echo "  Drive the rover / send mission commands now."
    echo "  Press Ctrl-C at any time to stop early and collect CSVs."
    echo "  A ros2 topic hz report will be captured at the mid-point."
    echo ""

    # Blank placeholder lines so the first cursor-up has something to overwrite
    for (( i=0; i<DASH; i++ )); do echo ""; done

    local elapsed=0 bar filled empty pct i
    local ch_line se_line ch_used ch_free ch_n se_used se_free se_n stm32_status bw_status
    local hz_captured=false
    local hz_midpoint=$(( RUN_DURATION / 2 ))

    while (( elapsed < RUN_DURATION )); do
        sleep 1
        elapsed=$(( elapsed + 1 ))

        # Capture hz report once at the mid-point of the run
        if [[ "$hz_captured" == false ]] && (( elapsed >= hz_midpoint )); then
            hz_captured=true
            log "  Mid-run: capturing ros2 topic hz report from RPi and Jetson..."
            capture_hz_report &
        fi

        # --- Progress bar ---
        filled=$(( elapsed * bar_width / RUN_DURATION ))
        empty=$(( bar_width - filled ))
        bar=""
        for (( i=0; i<filled; i++ )); do bar+="█"; done
        for (( i=0; i<empty;  i++ )); do bar+="░"; done
        pct=$(( elapsed * 100 / RUN_DURATION ))

        # --- STM32 latest values parsed from collector log ---
        ch_line=$(grep '\[chassis\]' "$LOG_DIR/stm32_collector.log" 2>/dev/null | tail -1) || ch_line=""
        se_line=$(grep '\[sensors\]' "$LOG_DIR/stm32_collector.log" 2>/dev/null | tail -1) || se_line=""
        ch_used=$(printf '%s' "$ch_line" | grep -oP 'used=\s*\K\S+') || ch_used="--"
        ch_free=$(printf '%s' "$ch_line" | grep -oP 'free=\s*\K\S+') || ch_free="--"
        ch_n=$(grep -c '\[chassis\]' "$LOG_DIR/stm32_collector.log" 2>/dev/null) || ch_n=0
        se_used=$(printf '%s' "$se_line" | grep -oP 'used=\s*\K\S+') || se_used="--"
        se_free=$(printf '%s' "$se_line" | grep -oP 'free=\s*\K\S+') || se_free="--"
        se_n=$(grep -c '\[sensors\]' "$LOG_DIR/stm32_collector.log" 2>/dev/null) || se_n=0

        # --- Collector health ---
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

        # --- Overwrite dashboard in place (cursor up DASH lines, carriage return, rewrite) ---
        printf "\033[%dA\r" "$DASH"
        printf "  Progress:%-61s\n" ""
        printf "  [%-${bar_width}s] %3d%%  %ds / %ds%-20s\n" "$bar" "$pct" "$elapsed" "$RUN_DURATION" ""
        printf "%-70s\n" ""
        printf "  STM32 memory (latest):%-48s\n" ""
        printf "    chassis  used=%-8s  free=%-8s  (%4d samples)%-10s\n" \
               "$ch_used" "$ch_free" "$ch_n" ""
        printf "    sensors  used=%-8s  free=%-8s  (%4d samples)%-10s\n" \
               "$se_used" "$se_free" "$se_n" ""
        printf "%-70s\n" ""
        printf "  Collectors:%-59s\n" ""
        printf "    STM32 %b  BW %b  RPi lat \033[0;32m●\033[0m  Jetson lat \033[0;32m●\033[0m   \n" \
               "$stm32_status" "$bw_status"
    done

    echo ""
    log "Run duration complete"
}

# ============================================================================
# Step 7b — Capture ros2 topic hz report (called mid-run from wait_for_run)
# ============================================================================
# Runs on both RPi and Jetson in parallel. Each topic gets a 10-second window.
# Output is a plain text file — one stanza per topic showing rate, min/max delta,
# and std dev. Stored alongside the CSVs for reference.

capture_hz_report() {
    local topics="/tpc_chassis_imu /tpc_chassis_sensors /tpc_rover_ctrl_cmd /tpc_rover_d415_rgb /tpc_rover_d415_depth"

    local _run_hz
    _run_hz() {
        local host="$1" outfile="$2" ws_path="$3"
        ssh -T $SSH_OPTS "$host" "
            source /opt/ros/humble/setup.bash 2>/dev/null
            source ${ws_path}/install/setup.bash 2>/dev/null
            export ROS_DOMAIN_ID=5
            for topic in ${topics}; do
                echo \"--- \$topic ---\"
                timeout 10 ros2 topic hz \$topic --window 50 2>/dev/null || echo \"(no messages)\"
            done
        " > "$outfile" 2>/dev/null || true
    }

    _run_hz "$RPI_HOST"    "$RUN_DIR/hz_report_rpi.txt"    "~/almondmatcha/ws_rpi" &
    _run_hz "$JETSON_HOST" "$RUN_DIR/hz_report_jetson.txt" "~/almondmatcha/ws_jetson" &
    wait
    ok "  hz reports saved to $RUN_DIR/hz_report_{rpi,jetson}.txt"
}

# ============================================================================
# Step 8 — Stop collectors and pull CSVs
# ============================================================================

stop_and_collect() {
    log "Step 8 — Stopping all collectors and pulling CSVs"

    mkdir -p "$LOG_DIR/rpi" "$LOG_DIR/jetson"

    # ── Stop local collectors immediately (no network wait) ──────────────────
    [[ -n "$STM32_COLLECTOR_PID" ]] && kill "$STM32_COLLECTOR_PID" 2>/dev/null || true
    [[ -n "$TOPIC_BW_PID"        ]] && kill "$TOPIC_BW_PID"        2>/dev/null || true
    # Ensure hz report background job is done
    wait 2>/dev/null || true
    sleep 0.5   # allow final flush before STM32 CSV rename
    local f
    f=$(ls "$RUN_DIR"/stm32_chassis_*.csv 2>/dev/null | tail -1) && [[ -n "$f" ]] && mv "$f" "$RUN_DIR/stm32_chassis.csv" || true
    f=$(ls "$RUN_DIR"/stm32_sensors_*.csv 2>/dev/null | tail -1) && [[ -n "$f" ]] && mv "$f" "$RUN_DIR/stm32_sensors.csv" || true
    ok "  Local collectors stopped"

    # ── Stop + pull from RPi and Jetson in parallel ──────────────────────────
    # Each sub-shell: (1) stops net-stats, (2) pulls net-stats CSV,
    # (3) pulls node logs — all over the same ControlMaster socket.
    # Latency stop+collect scripts run in parallel too.
    log "  Stopping and pulling from RPi and Jetson in parallel..."

    (
        # Stop latency collector and pull latency CSV
        LOCAL_DEST_CSV="$RUN_DIR/latency_rpi.csv" TARGET_HOST="$RPI_HOST" \
            TARGET_LABEL=rpi SSH_OPTS="$SSH_OPTS" \
            bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh"

        # Stop net-stats and pull CSV
        ssh -T $SSH_OPTS "$RPI_HOST" "
            if [ -f ~/ros2_traces/net_stats_rpi.pid ]; then
                kill \$(cat ~/ros2_traces/net_stats_rpi.pid) 2>/dev/null || true
                rm -f ~/ros2_traces/net_stats_rpi.pid
            fi
            # Stop CPU and SoftIRQ loggers
            pkill -f cpu_logger_rpi     2>/dev/null || true
            pkill -f softirq_logger_rpi 2>/dev/null || true
        "
        scp $SSH_OPTS "$RPI_HOST:~/ros2_traces/net_stats_rpi.csv" \
            "$RUN_DIR/net_stats_rpi.csv" 2>/dev/null || true
        scp $SSH_OPTS "$RPI_HOST:~/almondmatcha_poc/cpu_rpi.csv" \
            "$RUN_DIR/cpu_rpi.csv" 2>/dev/null || true
        scp $SSH_OPTS "$RPI_HOST:~/almondmatcha_poc/softirq_rpi.csv" \
            "$RUN_DIR/softirq_rpi.csv" 2>/dev/null || true

        # Pull node logs
        scp $SSH_OPTS "$RPI_HOST:~/ros2_traces/poc_*.log" \
            "$LOG_DIR/rpi/" 2>/dev/null || true

        echo "[parallel-rpi] done"
    ) &
    RPI_PULL_PID=$!

    (
        # Stop latency collector and pull CSV
        LOCAL_DEST_CSV="$RUN_DIR/latency_jetson.csv" TARGET_HOST="$JETSON_HOST" \
            TARGET_LABEL=jetson SSH_OPTS="$SSH_OPTS" \
            bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh"

        # Stop net-stats and pull CSV
        ssh -T $SSH_OPTS "$JETSON_HOST" "
            if [ -f ~/ros2_traces/net_stats_jetson.pid ]; then
                kill \$(cat ~/ros2_traces/net_stats_jetson.pid) 2>/dev/null || true
                rm -f ~/ros2_traces/net_stats_jetson.pid
            fi
            # Stop CPU and SoftIRQ loggers
            pkill -f cpu_logger_jetson     2>/dev/null || true
            pkill -f softirq_logger_jetson 2>/dev/null || true
            pkill -f tegrastats             2>/dev/null || true
        "
        scp $SSH_OPTS "$JETSON_HOST:~/ros2_traces/net_stats_jetson.csv" \
            "$RUN_DIR/net_stats_jetson.csv" 2>/dev/null || true
        scp $SSH_OPTS "$JETSON_HOST:~/almondmatcha_poc/cpu_jetson_tegrastats.txt" \
            "$RUN_DIR/cpu_jetson_tegrastats.txt" 2>/dev/null || true
        scp $SSH_OPTS "$JETSON_HOST:~/almondmatcha_poc/softirq_jetson.csv" \
            "$RUN_DIR/softirq_jetson.csv" 2>/dev/null || true

        # Pull node logs
        scp $SSH_OPTS "$JETSON_HOST:~/ros2_traces/poc_*.log" \
            "$LOG_DIR/jetson/" 2>/dev/null || true

        echo "[parallel-jetson] done"
    ) &
    JETSON_PULL_PID=$!

    # Wait for both pulls to finish
    wait "$RPI_PULL_PID"    && ok "  RPi:    net-stats + latency + logs pulled" \
                            || warn "  RPi pull had errors — check files"
    wait "$JETSON_PULL_PID" && ok "  Jetson: net-stats + latency + logs pulled" \
                            || warn "  Jetson pull had errors — check files"

    # ── Close SSH control sockets ────────────────────────────────────────────
    ssh $SSH_OPTS -O exit "$RPI_HOST"    2>/dev/null || true
    ssh $SSH_OPTS -O exit "$JETSON_HOST" 2>/dev/null || true
    rm -rf "$SSH_CONTROL_DIR"

    # ── Build the time-bucketed merged CSV from all individual CSVs ──────────
    log "  Building merged_all.csv..."
    python3 "$TOOLS_DIR/merge_run_csv.py" --run-dir "$RUN_DIR" \
        && ok "  merged_all.csv written" \
        || warn "  merge_run_csv.py failed — individual CSVs are still intact"
}

# ============================================================================
# Step 9 — Print summary
# ============================================================================

print_summary() {
    echo ""
    echo -e "${BOLD}======================================================${NC}"
    echo -e "${GREEN}  EXPERIMENT COMPLETE — all files in run directory${NC}"
    echo -e "${BOLD}======================================================${NC}"
    echo ""
    echo "  Run directory: $RUN_DIR"
    echo ""
    echo "  Data files:"
    echo "    $RUN_DIR/latency_rpi.csv"
    echo "    $RUN_DIR/latency_jetson.csv"
    echo "    $RUN_DIR/net_stats_rpi.csv"
    echo "    $RUN_DIR/net_stats_jetson.csv"
    echo "    $RUN_DIR/topic_bw.csv"
    echo "    $RUN_DIR/stm32_chassis.csv"
    echo "    $RUN_DIR/stm32_sensors.csv"
    echo "    $RUN_DIR/cpu_rpi.csv"
    echo "    $RUN_DIR/cpu_jetson_tegrastats.txt"
    echo "    $RUN_DIR/softirq_rpi.csv"
    echo "    $RUN_DIR/softirq_jetson.csv"
    echo "    $RUN_DIR/hz_report_rpi.txt"
    echo "    $RUN_DIR/hz_report_jetson.txt"
    echo ""
    echo "  Merged (time-bucketed union):"
    echo "    $RUN_DIR/merged_all.csv"
    echo ""
    echo "  Logs:"
    echo "    $LOG_DIR/stm32_collector.log"
    echo "    $LOG_DIR/raw_serial_chassis.log  ← full STM32 serial stream"
    echo "    $LOG_DIR/raw_serial_sensors.log  ← full STM32 serial stream"
    echo "    $LOG_DIR/topic_bw.log"
    echo "    $LOG_DIR/rpi/poc_*.log       ← per-node logs from RPi"
    echo "    $LOG_DIR/jetson/poc_*.log    ← per-node logs from Jetson"
    echo ""
    echo "  Analyze — all-machine latency/jitter summary (RPi + Jetson):"
    echo "    python3 ws_base/tools/tracing/analyze_latency.py \\"
    echo "        --csv $RUN_DIR/latency_rpi.csv \\"
    echo "              $RUN_DIR/latency_jetson.csv \\"
    echo "        --out-dir $RUN_DIR/"
    echo ""
    echo "  Analyze — unified timeline (all sources, one command):"
    echo "    python3 ws_base/tools/tracing/analyze_latency.py --merge \\"
    echo "        --run-dir $RUN_DIR \\"
    echo "        --out-dir $RUN_DIR/"
    echo ""
}

# ============================================================================
# Main
# ============================================================================

main() {
    echo ""
    echo -e "${BOLD}  Single-Domain POC Experiment Launcher${NC}"
    echo -e "  Branch: single-domain | Domain: D5 | Duration: ${RUN_DURATION}s"
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
    print_summary
}

main
