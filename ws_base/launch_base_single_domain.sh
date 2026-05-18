#!/bin/bash
# Base Station — Single-Domain POC Launch Script
# POC: both nodes on ROS_DOMAIN_ID=5 (mission_monitoring was D4 in baseline)
#
# Changes vs launch_base_tmux.sh:
#   Pane 1  mission_monitoring_node_pc  D4 → D5

set -e

SESSION_NAME="base_poc"
WS_PATH="$HOME/almondmatcha/ws_base"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[0;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; NC='\033[0m'

check_prerequisites() {
    echo -e "${CYAN}[PRE-FLIGHT] Single-Domain POC${NC}"
    command -v tmux  &>/dev/null || { echo -e "${RED}[ERROR] tmux not found${NC}"; exit 1; }
    command -v ros2  &>/dev/null || { echo -e "${RED}[ERROR] ros2 not found${NC}"; exit 1; }
    [ -d "$WS_PATH/install" ]   || { echo -e "${RED}[ERROR] $WS_PATH/install missing — run colcon build first${NC}"; exit 1; }
    echo -e "${GREEN}[OK] prerequisites met${NC}"
}

main() {
    check_prerequisites

    if tmux has-session -t $SESSION_NAME 2>/dev/null; then
        echo -e "${YELLOW}[WARN] killing existing session '$SESSION_NAME'${NC}"
        tmux kill-session -t $SESSION_NAME
        sleep 0.5
    fi

    # Kill any stale ros2 run / collector processes from a previous session.
    # These zombie DDS participants consume STM32 participant table slots (MAX=20).
    # Do NOT restart the ros2 daemon — it must keep FASTRTPS_DEFAULT_PROFILES_FILE.
    pkill -f "ros2 run" 2>/dev/null || true
    pkill -f "collect_topic_bw" 2>/dev/null || true
    sleep 1

    tmux new-session -d -s $SESSION_NAME -x 220 -y 50
    tmux set-option -t $SESSION_NAME -g mouse on
    tmux set-option -t $SESSION_NAME -g pane-border-status top
    tmux set-option -t $SESSION_NAME -g pane-border-format " [#{pane_index}] #{pane_title} "
    tmux set-option -t $SESSION_NAME -g pane-border-style fg=colour220        # yellow = POC
    tmux set-option -t $SESSION_NAME -g pane-active-border-style fg=colour196 # red = POC

    tmux split-window -t $SESSION_NAME -h
    sleep 0.3

    # Pane 0 (left): Mission Command — D5 (unchanged)
    # MISSION_CMD_PARAMS: override params file path (set by POC scripts for lab/field configs)
    local cmd_params="${MISSION_CMD_PARAMS:-src/mission_control/config/params.yaml}"
    tmux select-pane -t $SESSION_NAME:0.0 -T "MISSION COMMAND [D5]"
    tmux send-keys -t $SESSION_NAME:0.0 "cd $WS_PATH && source $HOME/almondmatcha/common_ifaces/install/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=5 && export FASTRTPS_DEFAULT_PROFILES_FILE=$WS_PATH/fastdds_base.xml" C-m
    tmux send-keys -t $SESSION_NAME:0.0 "clear && echo -e '\\e[1;36m>>> MISSION COMMAND [POC D5] <<<\\e[0m'" C-m
    tmux send-keys -t $SESSION_NAME:0.0 "ros2 run mission_control mission_command_node --ros-args --params-file $cmd_params" C-m

    # Pane 1 (right): Mission Monitoring — D5 (was D4)
    # tpc_telemetry_relay is now published on D5 so subscription still works.
    # TELEMETRY_CSV_PATH: when set, node writes CSV directly to that path (used by POC scripts)
    local mon_cmd="ros2 run mission_control mission_monitoring_node_pc"
    if [[ -n "${TELEMETRY_CSV_PATH:-}" ]]; then
        mon_cmd="ros2 run mission_control mission_monitoring_node_pc --ros-args -p csv_path:=${TELEMETRY_CSV_PATH}"
    fi
    tmux select-pane -t $SESSION_NAME:0.1 -T "MISSION MONITORING [D5 POC]"
    tmux send-keys -t $SESSION_NAME:0.1 "cd $WS_PATH && source $HOME/almondmatcha/common_ifaces/install/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=5 && export FASTRTPS_DEFAULT_PROFILES_FILE=$WS_PATH/fastdds_base.xml" C-m
    tmux send-keys -t $SESSION_NAME:0.1 "clear && echo -e '\\e[1;35m>>> MISSION MONITORING [POC D5 — was D4] <<<\\e[0m'" C-m
    tmux send-keys -t $SESSION_NAME:0.1 "$mon_cmd" C-m

    echo ""
    echo -e "${GREEN}Single-domain POC session '${SESSION_NAME}' started.${NC}"
    echo -e "${YELLOW}Both nodes on ROS_DOMAIN_ID=5 (monitoring was D4 in baseline).${NC}"
    echo -e "${BLUE}Reconnect: tmux attach-session -t ${SESSION_NAME}${NC}"
    echo ""

    [ -z "${SKIP_ATTACH:-}" ] && tmux attach-session -t $SESSION_NAME
}

main
