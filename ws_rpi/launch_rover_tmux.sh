#!/bin/bash
# Rover Launch with Tmux - 3x2 Grid Layout
# Launch all 6 nodes in separate terminal panes

SESSION_NAME="rover"

# Kill existing session if exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Create new tmux session
tmux new-session -d -s $SESSION_NAME

# Create 3x2 grid layout using tiled layout
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v
tmux select-pane -t 4
tmux split-window -v

# Wait for panes to be created
sleep 0.5

# Enable pane titles and colorize borders
tmux set-option -g pane-border-status top
tmux set-option -g pane-border-format " [#{pane_index}] #{pane_title} "
tmux set-option -g pane-border-style fg=colour240
tmux set-option -g pane-active-border-style fg=colour51

# Pane 0 (top-left): GNSS Spresense
tmux select-pane -t 0 -T "GNSS_Spresense"
tmux send-keys -t $SESSION_NAME:0.0 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.0 "clear && echo -e '\\e[1;36m>>> [1/6] GNSS SPRESENSE <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.0 "ros2 run pkg_gnss_navigation node_gnss_spresense" C-m

# Pane 1 (middle-left): GNSS Mission Monitor
tmux select-pane -t 1 -T "GNSS_Mission_Monitor"
tmux send-keys -t $SESSION_NAME:0.1 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "clear && echo -e '\\e[1;32m>>> [2/6] GNSS MISSION MONITOR <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.1 "ros2 run pkg_gnss_navigation node_gnss_mission_monitor" C-m

# Pane 2 (bottom-left): Chassis Controller
tmux select-pane -t 2 -T "Chassis_Controller"
tmux send-keys -t $SESSION_NAME:0.2 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.2 "clear && echo -e '\\e[1;33m>>> [3/6] CHASSIS CONTROLLER <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.2 "ros2 run pkg_chassis_control node_chassis_controller" C-m

# Pane 3 (top-right): Chassis IMU (Domain 5)
tmux select-pane -t 3 -T "Chassis_IMU_D5"
tmux send-keys -t $SESSION_NAME:0.3 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.3 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.3 "clear && echo -e '\\e[1;35m>>> [4/6] CHASSIS IMU (Domain 5) <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.3 "ros2 run pkg_chassis_sensors node_chassis_imu" C-m

# Pane 4 (middle-right): Domain Bridge (Domain 5)
tmux select-pane -t 4 -T "Domain_Bridge_D5"
tmux send-keys -t $SESSION_NAME:0.4 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.4 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.4 "clear && echo -e '\\e[1;34m>>> [5/6] DOMAIN BRIDGE (D5→D2) <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.4 "ros2 run pkg_chassis_control node_domain_bridge" C-m

# Pane 5 (bottom-right): Chassis Sensors (Domain 6)
tmux select-pane -t 5 -T "Chassis_Sensors_D6"
tmux send-keys -t $SESSION_NAME:0.5 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.5 "export ROS_DOMAIN_ID=6" C-m
tmux send-keys -t $SESSION_NAME:0.5 "clear && echo -e '\\e[1;31m>>> [6/6] CHASSIS SENSORS (Domain 6) <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.5 "ros2 run pkg_chassis_sensors node_chassis_sensors" C-m

# Focus on top-left pane and attach
tmux select-pane -t 0
tmux attach-session -t $SESSION_NAME

# CONTROLS:
# Ctrl+b then arrow keys: Navigate between panes
# Ctrl+b z: Zoom current pane (toggle fullscreen)
# Ctrl+b [: Scroll mode (press q to exit)
# Ctrl+d or type 'exit': Close current pane
