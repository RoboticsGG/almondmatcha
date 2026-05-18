#!/bin/bash
# Base Station — Multi-Domain POC Launch Script
# Multi-domain topology (same as main branch):
#   Pane 0  mission_command_node       D5  (sends actions/services to RPi)
#   Pane 1  mission_monitoring_node_pc D4  (subscribes tpc_telemetry_relay from RPi)

set -e

SESSION_NAME="base_poc"
WS_PATH="$HOME/almondmatcha/ws_base"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[0;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; NC='\033[0m'

check_prerequisites() {
    echo -e "${CYAN}[PRE-FLIGHT] Multi-Domain POC${NC}"
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
    tmux set-option -t $SESSION_NAME -g pane-border-style fg=colour33         # blue = multi-domain
    tmux set-option -t $SESSION_NAME -g pane-active-border-style fg=colour27  # dark blue

    tmux split-window -t $SESSION_NAME -h
    sleep 0.3

    # Pane 0 (left): Mission Command — D5
    tmux select-pane -t $SESSION_NAME:0.0 -T "MISSION COMMAND [D5]"
    tmux send-keys -t $SESSION_NAME:0.0 "cd $WS_PATH && source $HOME/almondmatcha/common_ifaces/install/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=5 && export FASTRTPS_DEFAULT_PROFILES_FILE=$WS_PATH/fastdds_base.xml" C-m
    tmux send-keys -t $SESSION_NAME:0.0 "clear && echo -e '\\e[1;36m>>> MISSION COMMAND [D5] <<<\\e[0m'" C-m
    tmux send-keys -t $SESSION_NAME:0.0 "ros2 run mission_control mission_command_node --ros-args --params-file ${MISSION_CMD_PARAMS:-src/mission_control/config/params.yaml}" C-m

    # Pane 1 (right): Mission Monitoring — D4
    # Subscribes to tpc_telemetry_relay published by mission_monitoring_node_rpi on D4.
    # No FASTRTPS_DEFAULT_PROFILES_FILE needed — D4 has no STM32 participants.
    tmux select-pane -t $SESSION_NAME:0.1 -T "MISSION MONITORING [D4]"
    tmux send-keys -t $SESSION_NAME:0.1 "cd $WS_PATH && source $HOME/almondmatcha/common_ifaces/install/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=4" C-m
    tmux send-keys -t $SESSION_NAME:0.1 "clear && echo -e '\\e[1;35m>>> MISSION MONITORING [D4 — telemetry relay] <<<\\e[0m'" C-m
    tmux send-keys -t $SESSION_NAME:0.1 "ros2 run mission_control mission_monitoring_node_pc${TELEMETRY_CSV_PATH:+ --ros-args -p csv_path:=${TELEMETRY_CSV_PATH}}" C-m

    echo ""
    echo -e "${GREEN}Multi-domain POC session '${SESSION_NAME}' started.${NC}"
    echo -e "${YELLOW}mission_command: D5 | mission_monitoring: D4${NC}"
    echo -e "${BLUE}Reconnect: tmux attach-session -t ${SESSION_NAME}${NC}"
    echo ""

    [ -z "${SKIP_ATTACH:-}" ] && tmux attach-session -t $SESSION_NAME
}

main
