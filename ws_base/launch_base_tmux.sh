#!/bin/bash
# Base Station Mission Control Launch with Tmux
# Domain 6: Base station mission control (command generation & monitoring)
# This script launches 2 mission control nodes in separate tmux panes

set -e

SESSION_NAME="base_station"
WS_PATH="$HOME/almondmatcha/ws_base"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
NC='\033[0m' # No Color

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

print_header() {
    echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║${NC} $1"
    echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"
}

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# ============================================================================
# PRE-FLIGHT CHECKS
# ============================================================================

check_prerequisites() {
    print_header "Pre-flight Checks"
    
    # Check if tmux is installed
    if ! command -v tmux &> /dev/null; then
        print_error "tmux is not installed. Install with: sudo apt install tmux"
        exit 1
    fi
    print_success "tmux found"
    
    # Check if ROS 2 is available
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS 2 is not installed or not in PATH"
        exit 1
    fi
    print_success "ROS 2 found"
    
    # Check if workspace exists
    if [ ! -d "$WS_PATH" ]; then
        print_error "Workspace not found at: $WS_PATH"
        exit 1
    fi
    print_success "Workspace found at $WS_PATH"
    
    # Check if install directory exists
    if [ ! -d "$WS_PATH/install" ]; then
        print_error "Install directory not found. Run 'colcon build' first."
        exit 1
    fi
    print_success "Install directory found"
    
    echo ""
}

# ============================================================================
# SESSION SETUP
# ============================================================================

setup_tmux_session() {
    print_header "Setting up Tmux Session"
    
    # Kill existing session if it exists
    if tmux has-session -t $SESSION_NAME 2>/dev/null; then
        print_warning "Found existing session '$SESSION_NAME', killing it..."
        tmux kill-session -t $SESSION_NAME
        sleep 0.5
    fi
    
    # Create new tmux session with detached mode
    tmux new-session -d -s $SESSION_NAME -x 220 -y 50
    print_success "Created tmux session: $SESSION_NAME"
    
    # Configure tmux options
    tmux set-option -t $SESSION_NAME -g mouse on
    tmux set-option -t $SESSION_NAME -g pane-border-status top
    tmux set-option -t $SESSION_NAME -g pane-border-format " [#{pane_index}] #{pane_title} "
    tmux set-option -t $SESSION_NAME -g pane-border-style fg=colour240
    tmux set-option -t $SESSION_NAME -g pane-active-border-style fg=colour51
    tmux set-option -t $SESSION_NAME -g base-index 0
    
    print_success "Configured tmux options"
    echo ""
}

# ============================================================================
# PANE CREATION AND CONFIGURATION
# ============================================================================

create_panes() {
    print_header "Creating Panes (2-pane layout)"
    
    # Create vertical split (creates pane 1)
    tmux split-window -t $SESSION_NAME -h
    sleep 0.3
    
    print_success "Created 2-pane layout"
    echo ""
}

send_command_to_pane() {
    local pane=$1
    local title=$2
    local command=$3
    
    tmux select-pane -t $SESSION_NAME:$pane -T "$title"
    tmux send-keys -t $SESSION_NAME:$pane "cd $WS_PATH && source install/setup.bash" C-m
    tmux send-keys -t $SESSION_NAME:$pane "export ROS_DOMAIN_ID=2" C-m
    sleep 0.2
    tmux send-keys -t $SESSION_NAME:$pane "clear" C-m
    sleep 0.1
    tmux send-keys -t $SESSION_NAME:$pane "echo -e '\\e[1;36m>>> $title <<<\\e[0m'" C-m
    sleep 0.2
    tmux send-keys -t $SESSION_NAME:$pane "$command" C-m
}

# ============================================================================
# LAUNCH NODES
# ============================================================================

launch_nodes() {
    print_header "Launching Mission Control Nodes"
    
    # Pane 0 (left): Mission Command Generator
    print_info "Launching mission_command_node in pane 0..."
    send_command_to_pane 0 "MISSION COMMAND GENERATOR (Domain 6)" \
        "ros2 run mission_control mission_command_node"
    
    # Pane 1 (right): Mission Monitoring
    print_info "Launching mission_monitoring_node in pane 1..."
    send_command_to_pane 1 "MISSION MONITORING (Domain 6)" \
        "ros2 run mission_control mission_monitoring_node"
    
    print_success "All nodes launched"
    echo ""
}

# ============================================================================
# DISPLAY INFORMATION
# ============================================================================

display_info() {
    print_header "Launch Complete!"
    
    echo -e "${GREEN}Tmux session '${SESSION_NAME}' is running with 2 panes:${NC}"
    echo ""
    echo -e "  ${CYAN}[0] Left Pane:${NC}  mission_command_node (generates mission goals)"
    echo -e "  ${CYAN}[1] Right Pane:${NC} mission_monitoring_node (displays telemetry)"
    echo ""
    
    echo -e "${YELLOW}Tmux Controls:${NC}"
    echo -e "  ${BLUE}Ctrl+b${NC} →  Arrow keys   : Navigate between panes"
    echo -e "  ${BLUE}Ctrl+b${NC} →  z            : Zoom current pane (fullscreen toggle)"
    echo -e "  ${BLUE}Ctrl+b${NC} →  [            : Scroll mode (press q to exit)"
    echo -e "  ${BLUE}Ctrl+b${NC} →  d            : Detach from session"
    echo -e "  ${BLUE}Ctrl+d${NC}    : Close current pane or exit"
    echo ""
    
    echo -e "${YELLOW}Domain Configuration:${NC}"
    echo -e "  ROS_DOMAIN_ID = 2 (Base Station)"
    echo -e "  Requires: pkg_base_bridge on ws_rpi (Domain 5 ↔ 2 relay)"
    echo ""
    
    echo -e "${YELLOW}To reconnect to this session later:${NC}"
    echo -e "  ${BLUE}tmux attach-session -t ${SESSION_NAME}${NC}"
    echo ""
    
    echo -e "${YELLOW}To kill this session:${NC}"
    echo -e "  ${BLUE}tmux kill-session -t ${SESSION_NAME}${NC}"
    echo ""
}

# ============================================================================
# MAIN EXECUTION
# ============================================================================

main() {
    check_prerequisites
    setup_tmux_session
    create_panes
    launch_nodes
    display_info
    
    # Focus on pane 0 and attach
    tmux select-pane -t $SESSION_NAME:0
    print_info "Attaching to tmux session..."
    echo ""
    
    tmux attach-session -t $SESSION_NAME
}

# Run main function
main "$@"
