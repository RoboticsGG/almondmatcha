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
# Output files (all on base PC):
#   ~/ros2_traces/stm32_memory_poc.csv
#   ws_base/tools/tracing/data/poc_latency_rpi.csv
#   ws_base/tools/tracing/data/poc_latency_jetson.csv
#   ws_base/tools/monitoring/data/poc_net_stats_rpi.csv
#   ws_base/tools/monitoring/data/poc_net_stats_jetson.csv

set -euo pipefail

# ============================================================================
# Configuration — edit these if your network addresses change
# ============================================================================

RPI_HOST="curry@192.168.1.1"
JETSON_HOST="yupi@192.168.1.5"

CHASSIS_PORT="/dev/ttyACM1"   # verify with: minicom -b 115200 -D /dev/ttyACM1
SENSORS_PORT="/dev/ttyACM0"   # verify with: minicom -b 115200 -D /dev/ttyACM0

STM32_OUT="$HOME/ros2_traces/stm32_memory_poc.csv"
LATENCY_DATA_DIR="$HOME/almondmatcha/ws_base/tools/tracing/data"
NET_DATA_DIR="$HOME/almondmatcha/ws_base/tools/monitoring/data"

TOOLS_DIR="$HOME/almondmatcha/ws_base/tools"
WORKSPACE="$HOME/almondmatcha"

RUN_DURATION=300   # seconds — default 5 minutes
SKIP_LAUNCH=false

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
NET_RPI_PID=""
NET_JETSON_PID=""

cleanup() {
    echo ""
    warn "Interrupted — stopping all background collectors..."
    [[ -n "$STM32_COLLECTOR_PID" ]] && kill "$STM32_COLLECTOR_PID" 2>/dev/null || true
    [[ -n "$NET_RPI_PID"         ]] && kill "$NET_RPI_PID"         2>/dev/null || true
    [[ -n "$NET_JETSON_PID"      ]] && kill "$NET_JETSON_PID"      2>/dev/null || true
    # Stop remote latency collectors
    TARGET_HOST="$RPI_HOST"    TARGET_LABEL=rpi    \
        bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh" 2>/dev/null || true
    TARGET_HOST="$JETSON_HOST" TARGET_LABEL=jetson \
        bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh" 2>/dev/null || true
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

    [[ -e "$CHASSIS_PORT" ]] || die "Chassis serial port $CHASSIS_PORT not found. Is the board plugged in?"
    [[ -e "$SENSORS_PORT" ]] || die "Sensors serial port $SENSORS_PORT not found. Is the board plugged in?"

    ssh -o ConnectTimeout=5 -o BatchMode=yes "$RPI_HOST" true 2>/dev/null \
        || die "Cannot SSH to RPi ($RPI_HOST) — check connection and SSH keys"
    ssh -o ConnectTimeout=5 -o BatchMode=yes "$JETSON_HOST" true 2>/dev/null \
        || die "Cannot SSH to Jetson ($JETSON_HOST) — check connection and SSH keys"

    mkdir -p "$HOME/ros2_traces" "$LATENCY_DATA_DIR" "$NET_DATA_DIR"

    ok "All pre-flight checks passed"
}

# ============================================================================
# Step 1 — Start STM32 memory collector (before boards power on)
# ============================================================================

start_stm32_collector() {
    log "Step 1 — Starting STM32 memory collector"
    log "  chassis port : $CHASSIS_PORT  (node=chassis)"
    log "  sensors port : $SENSORS_PORT  (node=sensors)"
    log "  output CSV   : $STM32_OUT"

    python3 "$TOOLS_DIR/stm32_serial/collect_stm32_memory.py" \
        --chassis "$CHASSIS_PORT" \
        --sensors "$SENSORS_PORT" \
        --out     "$STM32_OUT" &
    STM32_COLLECTOR_PID=$!

    sleep 1
    kill -0 "$STM32_COLLECTOR_PID" 2>/dev/null \
        || die "STM32 collector exited immediately — check serial ports"

    ok "STM32 collector running (PID $STM32_COLLECTOR_PID)"
}

# ============================================================================
# Step 2 — Power-cycle STM32 boards
# ============================================================================

prompt_power_cycle() {
    echo ""
    echo -e "${BOLD}======================================================${NC}"
    echo -e "${YELLOW}  ACTION REQUIRED: Power-cycle both STM32 boards NOW${NC}"
    echo -e "${BOLD}======================================================${NC}"
    echo ""
    echo "  The collector is running. The most critical heap data is"
    echo "  captured in the 10-second RTPS discovery window right after"
    echo "  the boards boot — missing this window = missing the heap peak."
    echo ""
    pause "  Unplug and replug power to both STM32 boards, then press ENTER"
    log "Boards power-cycled — discovery window has started (t=0)"
}

# ============================================================================
# Step 3 — Launch ROS2 nodes on RPi, Jetson, then base PC
# ============================================================================

launch_ros2_nodes() {
    log "Step 3 — Launching ROS2 nodes (boards are in 10s discovery wait)"

    log "  Launching rover nodes on RPi ($RPI_HOST)..."
    ssh "$RPI_HOST" "bash ~/almondmatcha/ws_rpi/launch_rover_single_domain.sh" &
    sleep 1
    ok "  RPi launch sent"

    log "  Launching Jetson nodes ($JETSON_HOST)..."
    ssh "$JETSON_HOST" "bash ~/almondmatcha/ws_jetson/launch_jetson_single_domain.sh" &
    sleep 1
    ok "  Jetson launch sent"

    log "  Launching base PC nodes..."
    bash "$WORKSPACE/ws_base/launch_base_single_domain.sh" &
    sleep 2
    ok "  Base PC launch sent"

    log "  All ROS2 nodes launched — Linux participants now visible to STM32 discovery"
}

# ============================================================================
# Step 4 — Start latency collectors on RPi and Jetson
# ============================================================================

start_latency_collectors() {
    log "Step 4 — Starting latency/jitter collectors on RPi and Jetson"

    TARGET_HOST="$RPI_HOST"    TARGET_LABEL=rpi \
        bash "$TOOLS_DIR/tracing/start_trace.sh"
    ok "  RPi latency collector started"

    TARGET_HOST="$JETSON_HOST" TARGET_LABEL=jetson \
        bash "$TOOLS_DIR/tracing/start_trace.sh"
    ok "  Jetson latency collector started"
}

# ============================================================================
# Step 5 — Start net-stats collectors on RPi and Jetson (SSH + background)
# ============================================================================

start_net_collectors() {
    log "Step 5 — Starting network stats collectors on RPi and Jetson"

    ssh "$RPI_HOST" \
        "mkdir -p ~/ros2_traces && nohup python3 ~/almondmatcha/ws_base/tools/monitoring/collect_net_stats.py \
            --iface eth0 --out ~/ros2_traces/net_stats_rpi.csv \
            </dev/null >~/ros2_traces/net_stats_rpi.log 2>&1 &
         echo \$! > ~/ros2_traces/net_stats_rpi.pid
         echo [OK] net_stats collector PID: \$(cat ~/ros2_traces/net_stats_rpi.pid)" &
    NET_RPI_PID=$!

    ssh "$JETSON_HOST" \
        "mkdir -p ~/ros2_traces && nohup python3 ~/almondmatcha/ws_base/tools/monitoring/collect_net_stats.py \
            --iface eth0 --out ~/ros2_traces/net_stats_jetson.csv \
            </dev/null >~/ros2_traces/net_stats_jetson.log 2>&1 &
         echo \$! > ~/ros2_traces/net_stats_jetson.pid
         echo [OK] net_stats collector PID: \$(cat ~/ros2_traces/net_stats_jetson.pid)" &
    NET_JETSON_PID=$!

    wait "$NET_RPI_PID" "$NET_JETSON_PID"
    ok "  Net-stats collectors started on both SBCs"
}

# ============================================================================
# Step 6 — Wait for the run duration
# ============================================================================

wait_for_run() {
    echo ""
    echo -e "${BOLD}======================================================${NC}"
    echo -e "${GREEN}  EXPERIMENT RUNNING — ${RUN_DURATION}s measurement window${NC}"
    echo -e "${BOLD}======================================================${NC}"
    echo ""
    echo "  Drive the rover / send mission commands now."
    echo "  Press Ctrl-C at any time to stop early and collect CSVs."
    echo ""

    local elapsed=0
    while (( elapsed < RUN_DURATION )); do
        sleep 10
        elapsed=$(( elapsed + 10 ))
        local remaining=$(( RUN_DURATION - elapsed ))
        log "  Running... ${elapsed}s elapsed, ${remaining}s remaining"
    done

    log "Run duration complete"
}

# ============================================================================
# Step 7 — Stop collectors and pull CSVs
# ============================================================================

stop_and_collect() {
    log "Step 7 — Stopping all collectors and pulling CSVs"

    # Stop latency collectors
    TARGET_HOST="$RPI_HOST"    TARGET_LABEL=rpi \
        bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh"
    TARGET_HOST="$JETSON_HOST" TARGET_LABEL=jetson \
        bash "$TOOLS_DIR/tracing/stop_and_collect_trace.sh"

    # Stop net-stats collectors on SBCs
    ssh "$RPI_HOST" \
        "if [ -f ~/ros2_traces/net_stats_rpi.pid ]; then
             kill \$(cat ~/ros2_traces/net_stats_rpi.pid) 2>/dev/null || true
             rm -f ~/ros2_traces/net_stats_rpi.pid
             echo '[OK] net_stats stopped on RPi'
         fi"
    ssh "$JETSON_HOST" \
        "if [ -f ~/ros2_traces/net_stats_jetson.pid ]; then
             kill \$(cat ~/ros2_traces/net_stats_jetson.pid) 2>/dev/null || true
             rm -f ~/ros2_traces/net_stats_jetson.pid
             echo '[OK] net_stats stopped on Jetson'
         fi"

    # Pull net-stats CSVs
    log "  Pulling net-stats CSVs..."
    scp "$RPI_HOST:~/ros2_traces/net_stats_rpi.csv"       "$NET_DATA_DIR/poc_net_stats_rpi.csv"
    scp "$JETSON_HOST:~/ros2_traces/net_stats_jetson.csv" "$NET_DATA_DIR/poc_net_stats_jetson.csv"
    ok "  Net-stats CSVs pulled"

    # Stop STM32 collector
    [[ -n "$STM32_COLLECTOR_PID" ]] && kill "$STM32_COLLECTOR_PID" 2>/dev/null || true
    ok "  STM32 collector stopped"
}

# ============================================================================
# Step 8 — Print summary
# ============================================================================

print_summary() {
    echo ""
    echo -e "${BOLD}======================================================${NC}"
    echo -e "${GREEN}  EXPERIMENT COMPLETE — CSV files collected${NC}"
    echo -e "${BOLD}======================================================${NC}"
    echo ""
    echo "  STM32 heap/stack:"
    echo "    $STM32_OUT"
    echo ""
    echo "  Latency / jitter:"
    echo "    $LATENCY_DATA_DIR/poc_latency_rpi.csv"
    echo "    $LATENCY_DATA_DIR/poc_latency_jetson.csv"
    echo ""
    echo "  Network stats:"
    echo "    $NET_DATA_DIR/poc_net_stats_rpi.csv"
    echo "    $NET_DATA_DIR/poc_net_stats_jetson.csv"
    echo ""
    echo "  Next — analyze results:"
    echo ""
    echo "    # Jitter/latency summary"
    echo "    python3 ws_base/tools/tracing/analyze_latency.py \\"
    echo "        --poc $LATENCY_DATA_DIR/poc_latency_rpi.csv"
    echo ""
    echo "    # Unified timeline (all sources)"
    echo "    python3 ws_base/tools/tracing/analyze_latency.py --merge \\"
    echo "        --latency-rpi    $LATENCY_DATA_DIR/poc_latency_rpi.csv \\"
    echo "        --latency-jetson $LATENCY_DATA_DIR/poc_latency_jetson.csv \\"
    echo "        --stm32          $STM32_OUT \\"
    echo "        --net-rpi        $NET_DATA_DIR/poc_net_stats_rpi.csv \\"
    echo "        --net-jetson     $NET_DATA_DIR/poc_net_stats_jetson.csv \\"
    echo "        --out-dir        ws_base/tools/tracing/results/"
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
    start_stm32_collector
    prompt_power_cycle

    if [[ "$SKIP_LAUNCH" == false ]]; then
        launch_ros2_nodes
    else
        warn "  --skip-launch set: skipping ROS2 node launch"
    fi

    start_latency_collectors
    start_net_collectors
    wait_for_run
    stop_and_collect
    print_summary
}

main
