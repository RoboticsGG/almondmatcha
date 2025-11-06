#!/bin/bash
# Rover Launch with Tmux - 3x2 Grid Layout (5 nodes after consolidating to Domain 5)
# Launch all rover nodes in separate terminal panes

SESSION_NAME="rover"

# Kill existing session if exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Create new tmux session
tmux new-session -d -s $SESSION_NAME

# Create 3x2 grid layout
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
tmux send-keys -t $SESSION_NAME:0.0 "clear && echo -e '\\e[1;36m>>> [1/5] GNSS SPRESENSE <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.0 "ros2 run pkg_gnss_navigation node_gnss_spresense" C-m

# Pane 1 (middle-left): GNSS Mission Monitor
tmux select-pane -t 1 -T "GNSS_Mission_Monitor"
tmux send-keys -t $SESSION_NAME:0.1 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "clear && echo -e '\\e[1;32m>>> [2/5] GNSS MISSION MONITOR <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.1 "ros2 run pkg_gnss_navigation node_gnss_mission_monitor" C-m

# Pane 2 (bottom-left): Chassis Controller
tmux select-pane -t 2 -T "Chassis_Controller"
tmux send-keys -t $SESSION_NAME:0.2 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.2 "clear && echo -e '\\e[1;33m>>> [3/5] CHASSIS CONTROLLER <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.2 "ros2 run pkg_chassis_control node_chassis_controller" C-m

# Pane 3 (top-right): Chassis IMU (Domain 5)
tmux select-pane -t 3 -T "Chassis_IMU_D5"
tmux send-keys -t $SESSION_NAME:0.3 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.3 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.3 "clear && echo -e '\\e[1;35m>>> [4/5] CHASSIS IMU (Domain 5) <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.3 "ros2 run pkg_chassis_sensors node_chassis_imu" C-m

# Pane 4 (middle-right): Chassis Sensors (Domain 5)
tmux select-pane -t 4 -T "Chassis_Sensors_D5"
tmux send-keys -t $SESSION_NAME:0.4 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.4 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.4 "clear && echo -e '\\e[1;34m>>> [5/5] CHASSIS SENSORS (Domain 5) <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.4 "ros2 run pkg_chassis_sensors node_chassis_sensors" C-m

# Pane 5 (bottom-right): Reserved for future use or monitoring
tmux select-pane -t 5 -T "Monitor"
tmux send-keys -t $SESSION_NAME:0.5 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.5 "clear && echo -e '\\e[1;37m>>> [MONITOR] Reserved pane <<<\\e[0m'" C-m

# Focus on top-left pane and attach
tmux select-pane -t 0
tmux attach-session -t $SESSION_NAME

# CONTROLS:
# Ctrl+b then arrow keys: Navigate between panes
# Ctrl+b z: Zoom current pane (toggle fullscreen)
# Ctrl+b [: Scroll mode (press q to exit)
# Ctrl+d or type 'exit': Close current pane
