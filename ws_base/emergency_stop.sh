#!/bin/bash
# emergency_stop.sh — Stop the rover from anywhere on the base PC
#
# The Ctrl-C emergency stop lives in the terminal running launch_field.sh. If
# you are attached to the base_station tmux session instead — which is where an
# operator normally watches from — Ctrl-C only kills the node in the focused
# pane and the rover keeps driving. This script is the same stop, callable
# without that terminal.
#
#   bash ws_base/emergency_stop.sh
#   bash ws_base/emergency_stop.sh --rpi curry@192.168.1.1 --jetson yupi@192.168.1.5
#
# Intended to be aliased on the base PC so it is one word from any shell:
#   echo "alias rover-stop='bash \$HOME/almondmatcha/ws_base/emergency_stop.sh'" >> ~/.bashrc
#
# Order matters. Nodes are killed FIRST, then a zeroed command is sent: the
# STM32 has no command watchdog — motor_control_task only acts when a new
# tpc_chassis_cmd arrives, so the PWM registers hold the last commanded duty
# and the rover keeps driving after every node is dead. And chassis_controller
# publishes at ~50 Hz, so a stop sent while it is alive is overwritten by its
# next frame.

set -uo pipefail

RPI_HOST="${RPI_HOST:-curry@192.168.1.1}"
JETSON_HOST="${JETSON_HOST:-yupi@192.168.1.5}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --rpi)     RPI_HOST="$2";    shift 2 ;;
        --jetson)  JETSON_HOST="$2"; shift 2 ;;
        *) echo "Unknown argument: $1" >&2; exit 2 ;;
    esac
done

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BOLD='\033[1m'; NC='\033[0m'
log()  { echo -e "\033[0;36m[$(date +%H:%M:%S)]${NC} $*"; }
ok()   { echo -e "${GREEN}[$(date +%H:%M:%S)] OK${NC} $*"; }
warn() { echo -e "${YELLOW}[$(date +%H:%M:%S)] WARN${NC} $*"; }

# Reuse launch_field.sh's multiplexed connections when it set them up, so a
# password-authenticated operator is not prompted again mid-emergency.
SSH_OPTS="${SSH_OPTS:--o ConnectTimeout=5}"

echo ""
echo -e "${RED}${BOLD}╔══════════════════════════════════════════════╗${NC}"
echo -e "${RED}${BOLD}║         EMERGENCY STOP INITIATED             ║${NC}"
echo -e "${RED}${BOLD}╚══════════════════════════════════════════════╝${NC}"
echo ""

# ── Base PC ─────────────────────────────────────────────────────────────────
# '[r]os2 run' rather than 'ros2 run': the pattern matches any command line
# containing it, including this shell's own, so the unbracketed form makes the
# script kill itself before reaching the lines below.
tmux kill-session -t base_station 2>/dev/null || true
pkill -TERM -f "[r]os2 run"    2>/dev/null || true
pkill -TERM -f "[r]os2 launch" 2>/dev/null || true
ok "  Base PC nodes stopped"

# ── RPi ─────────────────────────────────────────────────────────────────────
log "  Stopping RPi nodes ($RPI_HOST)..."
ssh $SSH_OPTS "$RPI_HOST" "
    tmux kill-session -t rover 2>/dev/null || true
    pkill -TERM -f '[r]os2 run'    2>/dev/null || true
    pkill -TERM -f '[r]os2 launch' 2>/dev/null || true
" 2>/dev/null && ok "  RPi nodes stopped" || warn "  RPi stop had errors (non-fatal)"

# ── Jetson ──────────────────────────────────────────────────────────────────
log "  Stopping Jetson nodes ($JETSON_HOST)..."
ssh $SSH_OPTS "$JETSON_HOST" "
    tmux kill-session -t jetson_vision 2>/dev/null || true
    pkill -TERM -f '[r]os2 run'    2>/dev/null || true
    pkill -TERM -f '[r]os2 launch' 2>/dev/null || true
" 2>/dev/null && ok "  Jetson nodes stopped" || warn "  Jetson stop had errors (non-fatal)"

# ── Explicit motor stop ─────────────────────────────────────────────────────
# Published from the RPi: it shares the Domain 5 segment with the STM32 and
# already has msgs_ifaces built. Repeated at 10 Hz for ~3 s because a fresh
# publisher needs DDS discovery to complete before its first frame lands.
# Fields match chassis_controller_node's own emergency-stop output.
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
    warn "  Cut the main power switch if the rover is still moving."
fi

echo ""
echo -e "${YELLOW}[$(date +%H:%M:%S)] Emergency stop complete.${NC}"
echo ""
