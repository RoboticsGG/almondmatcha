#!/bin/bash
# ws_base Mission Control - GNU Screen Launcher
# Domain 6: Base station command & monitoring

set -e
SESSION="base_station"
WS="$HOME/almondmatcha/ws_base"

# Colors
R='\033[0;31m'; G='\033[0;32m'; Y='\033[0;33m'
B='\033[0;34m'; C='\033[0;36m'; N='\033[0m'

err() { echo -e "${R}[ERROR]${N} $1" >&2; exit 1; }
ok() { echo -e "${G}[OK]${N} $1"; }
info() { echo -e "${B}[INFO]${N} $1"; }

# Pre-flight checks
command -v screen >/dev/null 2>&1 || err "Install: sudo apt install screen"
command -v ros2 >/dev/null 2>&1 || err "ROS 2 not found"
[ -d "$WS" ] || err "Workspace not found: $WS"
[ -d "$WS/install" ] || err "Build first: cd $WS && colcon build"

ok "Prerequisites met"

# Kill existing session
screen -S $SESSION -X quit 2>/dev/null || true
info "Creating session: $SESSION"

# Create session
screen -d -m -S $SESSION -t "Mission_Cmd"
sleep 0.3

# Window 0: Command node
screen -S $SESSION -X send-keys "cd $WS && source install/setup.bash && export ROS_DOMAIN_ID=5" C-m
sleep 0.2
screen -S $SESSION -X send-keys "clear && echo -e '\\e[1;36m>>> MISSION COMMAND (Domain 5) <<<\\e[0m'" C-m
sleep 0.2
screen -S $SESSION -X send-keys "ros2 run mission_control mission_command_node" C-m

# Window 1: Monitoring node
screen -S $SESSION -X screen -t "Mission_Mon"
sleep 0.3
screen -S $SESSION -X send-keys "cd $WS && source install/setup.bash && export ROS_DOMAIN_ID=5" C-m
sleep 0.2
screen -S $SESSION -X send-keys "clear && echo -e '\\e[1;32m>>> MISSION MONITORING (Domain 5) <<<\\e[0m'" C-m
sleep 0.2
screen -S $SESSION -X send-keys "ros2 run mission_control mission_monitoring_node" C-m

ok "Nodes launched"

# Info
echo -e "\n${C}Session: $SESSION${N}"
echo -e "  Window 0: mission_command_node"
echo -e "  Window 1: mission_monitoring_node"
echo -e "\n${Y}Controls:${N}"
echo -e "  Ctrl+a n/p  - Next/prev window"
echo -e "  Ctrl+a 0-1  - Jump to window"
echo -e "  Ctrl+a d    - Detach"
echo -e "  Ctrl+a [    - Scroll (ESC=exit)"
echo -e "\n${Y}Reconnect:${N} screen -r $SESSION"
echo -e "${Y}Kill:${N} screen -S $SESSION -X quit"
echo -e "\n${B}Domain:${N} ROS_DOMAIN_ID=5 (Unified Architecture)"
echo -e "${B}Communication:${N} Direct DDS with ws_rpi on Domain 5\n"

info "Attaching to session..."
sleep 1
screen -r $SESSION
