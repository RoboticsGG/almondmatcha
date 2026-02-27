#!/bin/bash
# Launch Rover Monitoring with Telemetry Relay
# - Domain 5: mission_monitoring_node_rpi + node_gnss_ublox (subscribe to rover topics, publish telemetry relay)
# - Base station subscribes to /tpc_telemetry_relay on Domain 5

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
tmux send-keys -t $SESSION_NAME:0.0 "ros2 run gnss_navigation gnss_ublox_node" C-m

# Pane 1 (bottom-left): Mission Monitoring (Telemetry Relay Publisher)
tmux select-pane -t 1 -T "Mission_Monitor_RPi"
tmux send-keys -t $SESSION_NAME:0.1 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.1 "clear && echo -e '\\e[1;32m>>> MISSION MONITORING (Domain 5 - Telemetry Relay) <<<\\e[0m' && sleep 2" C-m
tmux send-keys -t $SESSION_NAME:0.1 "ros2 run rover_monitoring mission_monitoring_node_rpi" C-m

# Pane 2 (right): CSV Data Logger
tmux select-pane -t 2 -T "CSV_Logger"
tmux send-keys -t $SESSION_NAME:0.2 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.2 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.2 "clear && echo -e '\\e[1;33m>>> CSV DATA LOGGER (Domain 5) <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.2 "echo 'Waiting for other nodes to start...'" C-m
tmux send-keys -t $SESSION_NAME:0.2 "sleep 3" C-m
tmux send-keys -t $SESSION_NAME:0.2 "ros2 run rover_monitoring rover_monitoring_node" C-m

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
