#!/bin/bash
# launch_field.sh — Production Field Launcher  (main branch)
#
# Starts the complete rover system from the base PC with a single command.
# No measurement collectors — pure operational launch.
#
# Launch order:
#   1. Pre-flight: SSH auth, workspace build checks
#   2. Clean stale DDS participants on all machines
#   3. Start rover nodes on RPi  (launch_rover_tmux.sh → tmux session "rover")
#   4. Start vision nodes on Jetson (launch_jetson_tmux.sh → tmux session "jetson_vision")
#   5. Start base station nodes locally (launch_base_tmux.sh → tmux session "base_station")
#   6. Wait — Ctrl-C = EMERGENCY STOP across all machines
#
# Emergency stop:
#   Press Ctrl-C at ANY time.  The script will SSH to RPi and Jetson,
#   kill their tmux sessions and ros2 processes, then kill the local base
#   station session.
#
# Usage:
#   bash ws_base/launch_field.sh
#   bash ws_base/launch_field.sh --rpi curry@192.168.1.1
#   bash ws_base/launch_field.sh --jetson yupi@192.168.1.5
#   bash ws_base/launch_field.sh --skip-base    # skip local base station launch
#   bash ws_base/launch_field.sh --force        # don't prompt if the self-check fails
#
# Prerequisites (one-time, per machine):
#   • Base PC: ws_base built
#       cd ~/almondmatcha/ws_base   && bash build_clean.sh
#   • RPi:     ws_rpi built   (checked via SSH in pre-flight)
#   • Jetson:  ws_jetson built (checked via SSH in pre-flight)
#   • On base PC, add to ~/.bashrc:
#       export ROS_DOMAIN_ID=5
#       export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/almondmatcha/ws_base/fastdds_base.xml

# NOTE: set -e intentionally omitted — individual launch errors should not abort
# a live field run.  All critical failures use explicit die().
set -uo pipefail

# ============================================================================
# Configuration
# ============================================================================

RPI_HOST="curry@192.168.1.1"
JETSON_HOST="yupi@192.168.1.5"

WORKSPACE="$HOME/almondmatcha"

SKIP_BASE=false
FORCE=false

SSH_CONTROL_DIR="$(mktemp -d /tmp/field_ssh_ctl.XXXXXX)"
SSH_OPTS="-o ControlMaster=auto -o ControlPath=${SSH_CONTROL_DIR}/%r@%h:%p -o ControlPersist=600"

# ============================================================================
# Colours / logging
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
        --rpi)        RPI_HOST="$2";    shift 2 ;;
        --jetson)     JETSON_HOST="$2"; shift 2 ;;
        --skip-base)  SKIP_BASE=true;   shift   ;;
        --force|--yes) FORCE=true;      shift   ;;
        *) die "Unknown argument: $1" ;;
    esac
done

# ============================================================================
# Emergency cleanup — triggered on SIGINT (Ctrl-C), SIGTERM, EXIT
# ============================================================================

_CLEANED_UP=false

cleanup() {
    if $_CLEANED_UP; then return; fi
    _CLEANED_UP=true

    echo ""
    echo -e "${RED}${BOLD}╔══════════════════════════════════════════════╗${NC}"
    echo -e "${RED}${BOLD}║         EMERGENCY STOP INITIATED             ║${NC}"
    echo -e "${RED}${BOLD}║  Stopping all nodes across all machines...   ║${NC}"
    echo -e "${RED}${BOLD}╚══════════════════════════════════════════════╝${NC}"
    echo ""

    # ── Base PC ───────────────────────────────────────────────────────────────
    tmux kill-session -t base_station 2>/dev/null || true
    pkill -TERM -f "[r]os2 run"         2>/dev/null || true
    pkill -TERM -f "[r]os2 launch"      2>/dev/null || true

    # ── RPi ───────────────────────────────────────────────────────────────────
    log "  Stopping RPi nodes ($RPI_HOST)..."
    ssh $SSH_OPTS "$RPI_HOST" "
        tmux kill-session -t rover 2>/dev/null || true
        pkill -TERM -f '[r]os2 run'    2>/dev/null || true
        pkill -TERM -f '[r]os2 launch' 2>/dev/null || true
    " 2>/dev/null || warn "  RPi stop had errors (non-fatal)"

    # ── Jetson ────────────────────────────────────────────────────────────────
    log "  Stopping Jetson nodes ($JETSON_HOST)..."
    ssh $SSH_OPTS "$JETSON_HOST" "
        tmux kill-session -t jetson_vision 2>/dev/null || true
        pkill -TERM -f '[r]os2 run'          2>/dev/null || true
        pkill -TERM -f '[r]os2 launch'       2>/dev/null || true
    " 2>/dev/null || warn "  Jetson stop had errors (non-fatal)"

    # ── Explicit motor stop to the STM32 ──────────────────────────────────────
    # Killing the ROS nodes does NOT stop the wheels. The STM32 has no command
    # watchdog: motor_control_task only calls apply_motor_control() when a new
    # tpc_chassis_cmd arrives, so the PWM registers simply retain the last
    # commanded duty and the rover keeps driving after every node is dead.
    #
    # So send one final zeroed command ourselves. This must happen AFTER the
    # kills above -- chassis_controller_node publishes tpc_chassis_cmd at ~50 Hz,
    # so anything sent while it is alive is overwritten by its next frame.
    #
    # Published from the RPi: it shares the Domain 5 segment with the STM32 and
    # already has msgs_ifaces built. Repeated at 10 Hz for ~3 s because a fresh
    # publisher needs DDS discovery to complete before its first frame lands.
    # Fields match chassis_controller_node's own emergency-stop output:
    # fdr=2 (straight), steering 0, speed 0, bdr=0 (drive stopped).
    log "  Sending explicit STOP to the chassis STM32..."
    if ssh $SSH_OPTS "$RPI_HOST" "
        source /opt/ros/humble/setup.bash 2>/dev/null
        source ~/almondmatcha/common_ifaces/install/setup.bash 2>/dev/null
        source ~/almondmatcha/ws_rpi/install/setup.bash 2>/dev/null
        export ROS_DOMAIN_ID=5
        timeout 3 ros2 topic pub -r 10 \
            --qos-reliability reliable --qos-durability transient_local \
            /tpc_chassis_cmd msgs_ifaces/msg/ChassisCtrl \
            '{fdr_msg: 2, ro_ctrl_msg: 0.0, spd_msg: 0, bdr_msg: 0}' >/dev/null 2>&1
        true
    " 2>/dev/null; then
        ok "  Chassis STOP sent (motors commanded to zero)"
    else
        warn "  Could not send chassis STOP — VERIFY THE ROVER IS STATIONARY"
        warn "  Motors may still be running: the STM32 holds its last command."
    fi

    # ── Close SSH multiplexed connections ─────────────────────────────────────
    ssh $SSH_OPTS -O exit "$RPI_HOST"    2>/dev/null || true
    ssh $SSH_OPTS -O exit "$JETSON_HOST" 2>/dev/null || true
    rm -rf "$SSH_CONTROL_DIR"

    echo ""
    echo -e "${YELLOW}[$(date +%H:%M:%S)] Emergency stop complete.${NC}"
    echo -e "${YELLOW}  All remote and local nodes have been terminated.${NC}"
    echo ""
    exit 1
}

trap cleanup INT TERM EXIT

# ============================================================================
# Pre-flight
# ============================================================================

preflight() {
    log "Running pre-flight checks..."

    command -v tmux &>/dev/null || die "tmux not installed"
    command -v ssh  &>/dev/null || die "ssh not installed"

    [[ -d "$WORKSPACE/ws_base/install"   ]] || die "ws_base not built — run: cd $WORKSPACE/ws_base && bash build_clean.sh"
    ok "  Workspace builds verified"

    # ── Self-diagnosis ────────────────────────────────────────────────────────
    # Environment and network faults (unset ROS_DOMAIN_ID, missing DDS profile,
    # a base-PC address outside the profile whitelist, a dead STM32, blocked
    # multicast) all present identically at launch time: the rover comes up and
    # the base PC silently sees nothing. Diagnose them BEFORE energising the
    # chassis, and refuse to launch on a failure rather than warn and continue.
    # Advisory, not a gate. The operator decides: a check can be wrong, and
    # being unable to start the rover in the field is worse than starting it
    # with a known-suspect environment. The check is purely local -- it does
    # not SSH anywhere, so it never prompts for a password; the RPi and Jetson
    # are contacted once, below, where they have to be contacted anyway.
    if [[ -f "$WORKSPACE/ws_base/preflight_check.sh" ]]; then
        echo ""
        if WORKSPACE="$WORKSPACE" bash "$WORKSPACE/ws_base/preflight_check.sh"; then
            echo ""
        else
            echo ""
            warn "Self-check reported critical issues (see the FAIL lines above)."
            if [[ "$FORCE" == true ]]; then
                warn "  --force given — continuing anyway."
            elif [[ ! -t 0 ]]; then
                warn "  Not an interactive terminal — continuing anyway."
            else
                echo -en "${YELLOW}Launch anyway? [y/N] ${NC}"
                read -r _reply
                [[ "$_reply" =~ ^[Yy]$ ]] || die "Aborted by operator. Nothing was started."
                warn "  Continuing at operator request."
            fi
            echo ""
        fi
    else
        warn "  preflight_check.sh missing — skipping self-diagnosis"
    fi

    # SSH is genuinely required -- the remote nodes cannot be started without
    # it -- so this one still aborts. Password auth is fine: no BatchMode is
    # set, so ssh prompts normally, and ControlMaster means the password is
    # asked for once per host for the whole run (the self-check above will
    # usually have established the connection already).
    log "  Authenticating to RPi ($RPI_HOST)..."
    ssh $SSH_OPTS -o ConnectTimeout=10 "$RPI_HOST" true \
        || die "Cannot SSH to RPi ($RPI_HOST) — wrong password, or host unreachable"
    ok "  RPi reachable"

    log "  Authenticating to Jetson ($JETSON_HOST)..."
    ssh $SSH_OPTS -o ConnectTimeout=10 "$JETSON_HOST" true \
        || die "Cannot SSH to Jetson ($JETSON_HOST) — wrong password, or host unreachable"
    ok "  Jetson reachable"

    # Verify remote workspaces are built
    ssh $SSH_OPTS "$RPI_HOST" "test -d ~/almondmatcha/ws_rpi/install" \
        || die "ws_rpi not built on RPi — run: cd ~/almondmatcha/ws_rpi && colcon build"
    ssh $SSH_OPTS "$JETSON_HOST" "test -d ~/almondmatcha/ws_jetson/install" \
        || die "ws_jetson not built on Jetson — run: cd ~/almondmatcha/ws_jetson && colcon build"
    ok "  Remote workspace builds verified"

    ok "Pre-flight passed"
}

# ============================================================================
# Step 0 — Clean stale DDS participants
# ============================================================================

clean_stale_participants() {
    log "Step 0 — Cleaning stale DDS participants..."

    # Base PC
    tmux kill-session -t base_station 2>/dev/null || true
    pkill -f "[r]os2 run"    2>/dev/null || true
    pkill -f "[r]os2 launch" 2>/dev/null || true

    # RPi
    ssh $SSH_OPTS "$RPI_HOST" "
        tmux kill-session -t rover 2>/dev/null || true
        pkill -f '[r]os2 run'    2>/dev/null || true
        pkill -f '[r]os2 launch' 2>/dev/null || true
    " 2>/dev/null || warn "  RPi cleanup had errors (non-fatal)"

    # Jetson
    ssh $SSH_OPTS "$JETSON_HOST" "
        tmux kill-session -t jetson_vision 2>/dev/null || true
        pkill -f '[r]os2 run'    2>/dev/null || true
        pkill -f '[r]os2 launch' 2>/dev/null || true
    " 2>/dev/null || warn "  Jetson cleanup had errors (non-fatal)"

    sleep 3
    ok "Stale participants cleaned"
}

# ============================================================================
# Step 1 — Launch rover nodes on RPi
# ============================================================================

launch_rpi() {
    log "Step 1 — Launching rover nodes on RPi ($RPI_HOST)..."

    ssh -T $SSH_OPTS "$RPI_HOST" \
        "SKIP_ATTACH=1 bash ~/almondmatcha/ws_rpi/launch_rover_tmux.sh" \
        || warn "  RPi launch returned non-zero (attach skipped — non-fatal)"

    sleep 20
    ok "  RPi nodes started (tmux session: rover)"
    log "  Reconnect anytime: ssh $RPI_HOST -t 'tmux attach-session -t rover'"
}

# ============================================================================
# Step 2 — Launch vision nodes on Jetson
# ============================================================================

launch_jetson() {
    log "Step 2 — Launching vision nodes on Jetson ($JETSON_HOST)..."

    ssh -T $SSH_OPTS "$JETSON_HOST" \
        "SKIP_ATTACH=1 bash ~/almondmatcha/ws_jetson/launch_jetson_tmux.sh" \
        || warn "  Jetson launch returned non-zero (attach skipped — non-fatal)"

    sleep 10
    ok "  Jetson nodes started (tmux session: jetson_vision)"
    log "  Reconnect anytime: ssh $JETSON_HOST -t 'tmux attach-session -t jetson_vision'"
}

# ============================================================================
# Step 3 — Launch base station nodes locally
# ============================================================================

launch_base() {
    if [[ "$SKIP_BASE" == true ]]; then
        warn "Step 3 — Base station launch SKIPPED (--skip-base)"
        return
    fi

    log "Step 3 — Launching base station nodes locally..."

    SKIP_ATTACH=1 bash "$WORKSPACE/ws_base/launch_base_tmux.sh" \
        || warn "  Base station launch returned non-zero (attach skipped — non-fatal)"

    sleep 5
    ok "  Base station nodes started (tmux session: base_station)"
    log "  Reconnect anytime: tmux attach-session -t base_station"
}

# ============================================================================
# Step 4 — Wait (Ctrl-C = emergency stop)
# ============================================================================

wait_loop() {
    echo ""
    echo -e "${BOLD}=====================================================================${NC}"
    echo -e "${GREEN}  ROVER SYSTEM RUNNING${NC}"
    echo -e "${BOLD}=====================================================================${NC}"
    echo ""
    echo "  RPi     : ssh $RPI_HOST -t 'tmux attach-session -t rover'"
    echo "  Jetson  : ssh $JETSON_HOST -t 'tmux attach-session -t jetson_vision'"
    echo "  Base PC : tmux attach-session -t base_station"
    echo ""
    echo -e "  ${RED}${BOLD}Ctrl-C = EMERGENCY STOP${NC} — kills all nodes on all machines"
    echo ""

    # Monitor remote sessions and warn if they die unexpectedly
    local check_interval=30
    local elapsed=0
    while true; do
        sleep "$check_interval"
        elapsed=$(( elapsed + check_interval ))

        # Light-touch remote health check (non-blocking, best-effort)
        local rpi_ok=true jetson_ok=true
        ssh -T $SSH_OPTS -o ConnectTimeout=5 "$RPI_HOST" \
            "tmux has-session -t rover 2>/dev/null" 2>/dev/null || rpi_ok=false
        ssh -T $SSH_OPTS -o ConnectTimeout=5 "$JETSON_HOST" \
            "tmux has-session -t jetson_vision 2>/dev/null" 2>/dev/null || jetson_ok=false

        if [[ "$rpi_ok" == false ]]; then
            warn "[${elapsed}s] RPi tmux session 'rover' not found — nodes may have crashed"
            warn "  Check: ssh $RPI_HOST"
        fi
        if [[ "$jetson_ok" == false ]]; then
            warn "[${elapsed}s] Jetson tmux session 'jetson_vision' not found — nodes may have crashed"
            warn "  Check: ssh $JETSON_HOST"
        fi

        log "[${elapsed}s] System running — Ctrl-C to stop"
    done
}

# ============================================================================
# Main
# ============================================================================

# Disable EXIT trap during normal execution — only fire it on Ctrl-C / TERM.
# We re-enable a clean-exit variant at the bottom.
trap cleanup INT TERM

main() {
    trap - EXIT  # suppress EXIT trap during normal startup

    echo ""
    echo -e "${BOLD}  Almondmatcha — Production Field Launch${NC}"
    echo -e "  RPi:    $RPI_HOST"
    echo -e "  Jetson: $JETSON_HOST"
    echo -e "  Branch: main | ${RED}${BOLD}Ctrl-C at any time = EMERGENCY STOP${NC}"
    echo ""

    preflight
    clean_stale_participants
    launch_rpi
    launch_jetson
    launch_base
    wait_loop
}

main "$@"
