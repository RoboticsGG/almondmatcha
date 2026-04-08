#!/bin/bash
# launch_poc_experiment.sh — Full POC experiment launcher (run on base PC)
#
# Executes the complete single-domain POC measurement sequence in order:
#   1. Prompt to confirm STM32 boards are powered OFF
#   2. Start STM32 memory collector (background)
#   3. Prompt to power-cycle STM32 boards
#   4. Launch ROS2 nodes on RPi, Jetson, and base PC
#   5. Start latency collectors on RPi and Jetson
#   6. Start net-stats collectors on RPi and Jetson (background SSH)
#   7. Wait for the run duration
#   8. Stop all collectors and pull CSVs to base PC
#
# Usage:
#   bash ws_base/launch_poc_experiment.sh
#   bash ws_base/launch_poc_experiment.sh --duration 600   # 10-minute run (default: 300s)
#   bash ws_base/launch_poc_experiment.sh --skip-launch    # skip ROS2 node launch (collectors only)
#
# Output files (all on base PC under a single per-run directory):
#   ws_base/tools/poc_run/run_NNN/latency_rpi.csv
#   ws_base/tools/poc_run/run_NNN/latency_jetson.csv
#   ws_base/tools/poc_run/run_NNN/net_stats_rpi.csv
#   ws_base/tools/poc_run/run_NNN/net_stats_jetson.csv
#   ws_base/tools/poc_run/run_NNN/topic_bw.csv
#   ws_base/tools/poc_run/run_NNN/stm32_chassis.csv
#   ws_base/tools/poc_run/run_NNN/stm32_sensors.csv
#   ws_base/tools/poc_run/run_NNN/merged_all.csv    ← time-bucketed union of all above
#   ws_base/tools/poc_run/run_NNN/logs/             ← all sub-process logs

set -euo pipefail

# ============================================================================
# Configuration — edit these if your network addresses change
# ============================================================================

RPI_HOST="curry@192.168.1.1"
JETSON_HOST="yupi@192.168.1.5"

CHASSIS_PORT="/dev/ttyACM1"   # verify with: minicom -b 115200 -D /dev/ttyACM1
SENSORS_PORT="/dev/ttyACM0"   # verify with: minicom -b 115200 -D /dev/ttyACM0

TOOLS_DIR="$HOME/almondmatcha/ws_base/tools"
WORKSPACE="$HOME/almondmatcha"

# All experiment output lands in a single numbered run directory.
# run_NNN is auto-incremented — each launch creates the next available number.
POC_RUN_BASE="$WORKSPACE/ws_base/tools/poc_run"

_next_run_dir() {
    local last
    last=$(ls -d "${POC_RUN_BASE}"/run_* 2>/dev/null \
           | grep -oP 'run_\K[0-9]+' | sort -n | tail -1)
    printf '%s/run_%03d' "$POC_RUN_BASE" "$(( ${last:-0} + 1 ))"
}

RUN_DIR="$(_next_run_dir)"
LOG_DIR="$RUN_DIR/logs"

RUN_DURATION=300   # seconds — default 5 minutes
SKIP_LAUNCH=false

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

cleanup() {
    echo ""
    warn "Interrupted — stopping all background collectors..."
    [[ -n "$STM32_COLLECTOR_PID" ]] && kill "$STM32_COLLECTOR_PID" 2>/dev/null || true
    [[ -n "$TOPIC_BW_PID"        ]] && kill "$TOPIC_BW_PID"        2>/dev/null || true
    # Stop latency collectors (cleanup path — best effort)
    LOCAL_DEST_CSV="$RUN_DIR/latency_rpi.csv"    TARGET_HOST="$RPI_HOST"    TARGET_LABEL=rpi    SSH_OPTS="$SSH_OPTS" \
        bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh" 2>/dev/null || true
    LOCAL_DEST_CSV="$RUN_DIR/latency_jetson.csv" TARGET_HOST="$JETSON_HOST" TARGET_LABEL=jetson SSH_OPTS="$SSH_OPTS" \
        bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh" 2>/dev/null || true
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

    [[ -e "$CHASSIS_PORT" ]] || die "Chassis serial port $CHASSIS_PORT not found. Is the board plugged in?"
    [[ -e "$SENSORS_PORT" ]] || die "Sensors serial port $SENSORS_PORT not found. Is the board plugged in?"

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
        tmux kill-session -t jetson_poc 2>/dev/null || true
    " 2>/dev/null || warn "  Jetson cleanup had errors (non-fatal)"
    ok "  Jetson cleaned"

    # Let DDS process participant departures so STM32 boards (if already running)
    # can free slots before we reset them.
    sleep 3
    ok "All stale DDS participants cleaned — STM32 participant table will be empty after reset"
}

# ============================================================================
# Step 1 — Start STM32 memory collector (before boards power on)
# ============================================================================

start_stm32_collector() {
    log "Step 1 — Starting STM32 memory collector"
    log "  chassis port : $CHASSIS_PORT  (node=chassis)"
    log "  sensors port : $SENSORS_PORT  (node=sensors)"
    log "  output stem  : $RUN_DIR/stm32  →  stm32_chassis/sensors_YYYYMMDD_HHMMSS.csv"

    python3 "$TOOLS_DIR/stm32_serial/collect_stm32_memory.py" \
        --chassis "$CHASSIS_PORT" \
        --sensors "$SENSORS_PORT" \
        --out     "$RUN_DIR/stm32" \
        >"$LOG_DIR/stm32_collector.log" 2>&1 &
    STM32_COLLECTOR_PID=$!

    sleep 1
    kill -0 "$STM32_COLLECTOR_PID" 2>/dev/null \
        || die "STM32 collector exited immediately — check serial ports"

    ok "STM32 collector running (PID $STM32_COLLECTOR_PID)"
}

# ============================================================================
# Step 2 — Power-cycle STM32 boards
# ============================================================================

# ============================================================================
# Step 2 — Reset STM32 boards (automated via ST-Link, or manual fallback)
# ============================================================================

reset_stm32_boards() {
    log "Step 2 — Resetting STM32 boards"

    # Try automated reset via ST-Link (st-flash from stlink-tools package).
    # NUCLEO boards expose an ST-Link debugger over USB that can trigger a
    # hardware reset without manual intervention — ideal for remote operation.
    if command -v st-flash &>/dev/null; then
        log "  st-flash found — attempting automated hardware reset..."
        if st-flash --reset >/dev/null 2>&1; then
            ok "  STM32 boards reset via ST-Link"
            log "  Boards are now in SPDP discovery phase (SPDP_RESEND=500ms)"
            return 0
        fi
        warn "  st-flash --reset failed — falling back to manual reset"
    fi

    # Manual fallback — user must physically press RESET on the boards.
    echo ""
    echo -e "${BOLD}======================================================${NC}"
    echo -e "${YELLOW}  ACTION REQUIRED: Reset both STM32 boards NOW${NC}"
    echo -e "${BOLD}======================================================${NC}"
    echo ""
    echo "  Press the BLACK RESET button on both NUCLEO boards"
    echo "  (or unplug/replug USB power)."
    echo ""
    echo "  The serial collector is already running — after reset you should"
    echo "  see ts_ms values < 10000 in the collector output."
    echo ""
    echo -e "${RED}  ⚠  Press ENTER immediately after resetting — do NOT wait.${NC}"
    echo "     The script will verify the boards are alive before launching nodes."
    echo ""
    pause "  Reset both boards, then press ENTER"
}

# ============================================================================
# Step 2b — Verify STM32 boards booted and are freshly running
# ============================================================================

wait_stm32_boot() {
    log "  Verifying STM32 boards are alive via serial..."

    local timeout=30
    local start_time
    start_time=$(date +%s)

    # Snapshot current line counts so we can detect NEW serial data after reset
    local chassis_before sensors_before
    chassis_before=$(grep -c '\[chassis\]' "$LOG_DIR/stm32_collector.log" 2>/dev/null || echo 0)
    sensors_before=$(grep -c '\[sensors\]' "$LOG_DIR/stm32_collector.log" 2>/dev/null || echo 0)

    local chassis_ok=false
    local sensors_ok=false

    while (( $(date +%s) - start_time < timeout )); do
        local chassis_now sensors_now
        chassis_now=$(grep -c '\[chassis\]' "$LOG_DIR/stm32_collector.log" 2>/dev/null || echo 0)
        sensors_now=$(grep -c '\[sensors\]' "$LOG_DIR/stm32_collector.log" 2>/dev/null || echo 0)

        if (( chassis_now > chassis_before )) && ! $chassis_ok; then
            chassis_ok=true
            local chassis_ts
            chassis_ts=$(grep '\[chassis\]' "$LOG_DIR/stm32_collector.log" | tail -1 | grep -oP 'ts=\K[0-9]+' || echo "?")
            ok "  Chassis board: alive (ts=${chassis_ts}ms)"
            if [[ "$chassis_ts" != "?" ]] && (( chassis_ts > 60000 )); then
                warn "  Chassis ts=${chassis_ts}ms > 60s — board may NOT be freshly reset!"
            fi
        fi

        if (( sensors_now > sensors_before )) && ! $sensors_ok; then
            sensors_ok=true
            local sensors_ts
            sensors_ts=$(grep '\[sensors\]' "$LOG_DIR/stm32_collector.log" | tail -1 | grep -oP 'ts=\K[0-9]+' || echo "?")
            ok "  Sensors board: alive (ts=${sensors_ts}ms)"
            if [[ "$sensors_ts" != "?" ]] && (( sensors_ts > 60000 )); then
                warn "  Sensors ts=${sensors_ts}ms > 60s — board may NOT be freshly reset!"
            fi
        fi

        if $chassis_ok && $sensors_ok; then
            ok "  Both STM32 boards confirmed alive — proceeding to launch"
            sleep 1
            return 0
        fi

        sleep 0.5
    done

    # Timeout — report what's missing
    $chassis_ok || warn "  Chassis board: NO serial data after ${timeout}s!"
    $sensors_ok || warn "  Sensors board: NO serial data after ${timeout}s!"
    die "  Could not confirm both STM32 boards — aborting experiment"
}

# ============================================================================
# Step 3 — Launch ROS2 nodes on RPi, Jetson, then base PC
# ============================================================================

launch_ros2_nodes() {
    log "Step 3 — Launching ROS2 nodes (STM32 boards confirmed alive)"
    log "  (launch output redirected to $LOG_DIR/launch_<host>.log)"

    log "  Launching rover nodes on RPi ($RPI_HOST)..."
    ssh $SSH_OPTS "$RPI_HOST" "SKIP_ATTACH=1 bash ~/almondmatcha/ws_rpi/launch_rover_single_domain.sh" \
        >"$LOG_DIR/launch_rpi.log" 2>&1 &
    sleep 1
    ok "  RPi launch sent"

    log "  Launching Jetson nodes ($JETSON_HOST)..."
    ssh $SSH_OPTS "$JETSON_HOST" "SKIP_ATTACH=1 bash ~/almondmatcha/ws_jetson/launch_jetson_single_domain.sh" \
        >"$LOG_DIR/launch_jetson.log" 2>&1 &
    sleep 1
    ok "  Jetson launch sent"

    log "  Launching base PC nodes..."
    SKIP_ATTACH=1 bash "$WORKSPACE/ws_base/launch_base_single_domain.sh" \
        >"$LOG_DIR/launch_base.log" 2>&1 &
    sleep 2
    ok "  Base PC launch sent"

    log "  All ROS2 nodes launched — Linux participants now visible to STM32 discovery"
}

# ============================================================================
# Step 4 — Start latency collectors on RPi and Jetson
# ============================================================================

start_latency_collectors() {
    log "Step 4 — Starting latency/jitter collectors on RPi and Jetson"

    TARGET_HOST="$RPI_HOST"    TARGET_LABEL=rpi    SSH_OPTS="$SSH_OPTS" \
        bash "$TOOLS_DIR/tracing/start_trace.sh"
    ok "  RPi latency collector started"

    TARGET_HOST="$JETSON_HOST" TARGET_LABEL=jetson SSH_OPTS="$SSH_OPTS" \
        bash "$TOOLS_DIR/tracing/start_trace.sh"
    ok "  Jetson latency collector started"
}

# ============================================================================
# Step 4b — Start per-topic bandwidth collector (base PC, local)
# ============================================================================

start_topic_bw_collector() {
    log "Step 4b — Starting per-topic bandwidth collector (base PC)"
    log "  output: $RUN_DIR/topic_bw.csv"

    # Source ROS2 env and exec python3 directly (exec replaces bash → PID is python3).
    # FASTRTPS profile pins DDS to the base PC ethernet NIC (192.168.1.4).
    # Source order: humble → ws_base → common_ifaces (last wins on AMENT_PREFIX_PATH)
    # so msgs_ifaces from common_ifaces always overlays any stale ws_base copy.
    bash -c "
        source /opt/ros/humble/setup.bash
        source '$WORKSPACE/ws_base/install/setup.bash'
        source '$WORKSPACE/common_ifaces/install/setup.bash'
        export ROS_DOMAIN_ID=5
        export FASTRTPS_DEFAULT_PROFILES_FILE='$WORKSPACE/ws_base/fastdds_base.xml'
        exec python3 '$TOOLS_DIR/monitoring/collect_topic_bw.py' \
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
# Step 5 — Start net-stats collectors on RPi and Jetson (SSH + background)
# ============================================================================

start_net_collectors() {
    log "Step 5 — Starting network stats collectors on RPi and Jetson"

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
            setsid nohup python3 ~/almondmatcha/ws_base/tools/monitoring/collect_net_stats.py \
                --iface eth0 --out $out \
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
# Step 6 — Wait for the run duration
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
    echo ""

    # Blank placeholder lines so the first cursor-up has something to overwrite
    for (( i=0; i<DASH; i++ )); do echo ""; done

    local elapsed=0 bar filled empty pct i
    local ch_line se_line ch_used ch_free ch_n se_used se_free se_n stm32_status

    while (( elapsed < RUN_DURATION )); do
        sleep 1
        elapsed=$(( elapsed + 1 ))

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
        kill -0 "$STM32_COLLECTOR_PID" 2>/dev/null \
            && stm32_status="\033[0;32m● running\033[0m" \
            || stm32_status="\033[0;31m● DIED\033[0m"
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
        printf "    STM32 %b  Topic BW %b  RPi lat \033[0;32m●\033[0m  Jetson lat \033[0;32m●\033[0m   \n" \
               "$stm32_status" "$bw_status"
    done

    echo ""
    log "Run duration complete"
}

# ============================================================================
# Step 7 — Stop collectors and pull CSVs
# ============================================================================

stop_and_collect() {
    log "Step 7 — Stopping all collectors and pulling CSVs"

    mkdir -p "$LOG_DIR/rpi" "$LOG_DIR/jetson"

    # ── Stop local collectors immediately (no network wait) ──────────────────
    [[ -n "$STM32_COLLECTOR_PID" ]] && kill "$STM32_COLLECTOR_PID" 2>/dev/null || true
    [[ -n "$TOPIC_BW_PID"        ]] && kill "$TOPIC_BW_PID"        2>/dev/null || true
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

        # Stop net-stats and pull CSV in one SSH round-trip
        ssh -T $SSH_OPTS "$RPI_HOST" "
            if [ -f ~/ros2_traces/net_stats_rpi.pid ]; then
                kill \$(cat ~/ros2_traces/net_stats_rpi.pid) 2>/dev/null || true
                rm -f ~/ros2_traces/net_stats_rpi.pid
            fi
        "
        scp $SSH_OPTS "$RPI_HOST:~/ros2_traces/net_stats_rpi.csv" \
            "$RUN_DIR/net_stats_rpi.csv" 2>/dev/null || true

        # Pull node logs
        scp $SSH_OPTS "$RPI_HOST:~/ros2_traces/poc_*.log" \
            "$LOG_DIR/rpi/" 2>/dev/null || true

        echo "[parallel-rpi] done"
    ) &
    RPI_PULL_PID=$!

    (
        # Stop latency collector and pull latency CSV
        LOCAL_DEST_CSV="$RUN_DIR/latency_jetson.csv" TARGET_HOST="$JETSON_HOST" \
            TARGET_LABEL=jetson SSH_OPTS="$SSH_OPTS" \
            bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh"

        # Stop net-stats and pull CSV in one SSH round-trip
        ssh -T $SSH_OPTS "$JETSON_HOST" "
            if [ -f ~/ros2_traces/net_stats_jetson.pid ]; then
                kill \$(cat ~/ros2_traces/net_stats_jetson.pid) 2>/dev/null || true
                rm -f ~/ros2_traces/net_stats_jetson.pid
            fi
        "
        scp $SSH_OPTS "$JETSON_HOST:~/ros2_traces/net_stats_jetson.csv" \
            "$RUN_DIR/net_stats_jetson.csv" 2>/dev/null || true

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
    python3 "$TOOLS_DIR/poc_run/merge_run_csv.py" --run-dir "$RUN_DIR" \
        && ok "  merged_all.csv written" \
        || warn "  merge_run_csv.py failed — individual CSVs are still intact"
}

# ============================================================================
# Step 8 — Print summary
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
    echo ""
    echo "  Merged (time-bucketed union):"
    echo "    $RUN_DIR/merged_all.csv"
    echo ""
    echo "  Logs:"
    echo "    $LOG_DIR/stm32_collector.log"
    echo "    $LOG_DIR/topic_bw.log"
    echo "    $LOG_DIR/rpi/poc_*.log       ← per-node logs from RPi"
    echo "    $LOG_DIR/jetson/poc_*.log    ← per-node logs from Jetson"
    echo ""
    echo "  Analyze — jitter/latency summary:"
    echo "    python3 ws_base/tools/tracing/analyze_latency.py \\"
    echo "        --poc $RUN_DIR/latency_rpi.csv"
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
    echo -e "  Branch: single-domain | Domain ID: 5 | Duration: ${RUN_DURATION}s"
    echo ""

    preflight
    clean_stale_participants
    start_stm32_collector
    reset_stm32_boards
    wait_stm32_boot

    if [[ "$SKIP_LAUNCH" == false ]]; then
        launch_ros2_nodes
    else
        warn "  --skip-launch set: skipping ROS2 node launch"
    fi

    start_latency_collectors
    start_topic_bw_collector
    start_net_collectors
    wait_for_run
    stop_and_collect
    print_summary
}

main
