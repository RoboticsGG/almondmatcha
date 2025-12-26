#!/bin/bash
# Launch Rover Monitoring with Domain Bridge
# - Domain 5: node_rover_monitoring + node_gnss_ublox (subscribe to rover topics, publish status)
# - Domain 4: domain_relay.py (relay status to base station)

SESSION_NAME="rover_monitoring"

# Kill existing session if exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Create new tmux session
tmux new-session -d -s $SESSION_NAME

# Split into 3 panes (left column: 2 panes, right column: 1 pane)
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v

# Wait for panes to be created
sleep 0.5

# Enable pane titles
tmux set-option -g pane-border-status top
tmux set-option -g pane-border-format " [#{pane_index}] #{pane_title} "
tmux set-option -g pane-border-style fg=colour240
tmux set-option -g pane-active-border-style fg=colour51

# Pane 0 (top-left): GNSS Ublox RTK
tmux select-pane -t 0 -T "GNSS_Ublox_RTK"
tmux send-keys -t $SESSION_NAME:0.0 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.0 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.0 "clear && echo -e '\\e[1;36m>>> UBLOX RTK GNSS (Domain 5) <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.0 "ros2 run pkg_gnss_navigation node_gnss_ublox" C-m

# Pane 1 (bottom-left): Rover Monitoring
tmux select-pane -t 1 -T "Rover_Monitoring"
tmux send-keys -t $SESSION_NAME:0.1 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.1 "clear && echo -e '\\e[1;32m>>> ROVER MONITORING (Domain 5) <<<\\e[0m' && sleep 2" C-m
tmux send-keys -t $SESSION_NAME:0.1 "ros2 run pkg_rover_monitoring node_rover_monitoring" C-m

# Pane 2 (right): Domain Relay (5→4)
tmux select-pane -t 2 -T "Domain_Relay_5to4"
tmux send-keys -t $SESSION_NAME:0.2 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.2 "clear && echo -e '\\e[1;33m>>> DOMAIN RELAY (5→4) <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.2 "echo 'Waiting for node_rover_monitoring to start...'" C-m
tmux send-keys -t $SESSION_NAME:0.2 "sleep 5" C-m
tmux send-keys -t $SESSION_NAME:0.2 "echo 'Starting domain bridge...'" C-m
tmux send-keys -t $SESSION_NAME:0.2 "python3 install/pkg_rover_monitoring/lib/pkg_rover_monitoring/domain_relay.py" C-m

# Focus on top-left pane and attach
tmux select-pane -t 0
tmux attach-session -t $SESSION_NAME

# CONTROLS:
# Ctrl+b then arrow keys: Navigate between panes
# Ctrl+b z: Zoom current pane (toggle fullscreen)
# Ctrl+b d: Detach from session (session keeps running)
# Ctrl+d or type 'exit': Close current pane
#
# To reattach: tmux attach -t rover_monitoring
