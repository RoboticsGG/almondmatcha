#!/bin/bash
# ws_rpi Tmux Launch Script - Domain 5 Unified Architecture
# Domain 5: All rover nodes (GNSS, chassis, sensors)
# Launch all rover nodes in separate terminal panes

SESSION_NAME="rover"

# Kill existing session if exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Create new tmux session
tmux new-session -d -s $SESSION_NAME

# Create 2x3 grid (5 panes: 2 left column, 3 right column)
tmux split-window -h  # Split into left and right columns
tmux select-pane -t 0
tmux split-window -v  # Split left column into 2
tmux select-pane -t 2
tmux split-window -v  # Split right column into 2
tmux split-window -v  # Split right column into 3

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
tmux send-keys -t $SESSION_NAME:0.0 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.0 "clear && echo -e '\\e[1;36m>>> [1/5] GNSS SPRESENSE <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.0 "ros2 run pkg_gnss_navigation node_gnss_spresense" C-m

# Pane 1 (bottom-left): GNSS Mission Monitor
tmux select-pane -t 1 -T "GNSS_Mission"
tmux send-keys -t $SESSION_NAME:0.1 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.1 "clear && echo -e '\\e[1;32m>>> [2/5] GNSS MISSION MONITOR <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.1 "ros2 run pkg_gnss_navigation node_gnss_mission_monitor" C-m

# Pane 2 (top-right): Chassis Controller
tmux select-pane -t 2 -T "Chassis_Controller"
tmux send-keys -t $SESSION_NAME:0.2 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.2 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.2 "clear && echo -e '\\e[1;33m>>> [3/5] CHASSIS CONTROLLER <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.2 "ros2 run pkg_chassis_control node_chassis_controller" C-m

# Pane 3 (middle-right): Chassis IMU
tmux select-pane -t 3 -T "Chassis_IMU"
tmux send-keys -t $SESSION_NAME:0.3 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.3 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.3 "clear && echo -e '\\e[1;35m>>> [4/5] CHASSIS IMU <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.3 "ros2 run pkg_chassis_sensors node_chassis_imu" C-m

# Pane 4 (bottom-right): Chassis Sensors
tmux select-pane -t 4 -T "Chassis_Sensors"
tmux send-keys -t $SESSION_NAME:0.4 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.4 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.4 "clear && echo -e '\\e[1;34m>>> [5/5] CHASSIS SENSORS <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.4 "ros2 run pkg_chassis_sensors node_chassis_sensors" C-m

# Focus on top-left pane and attach
tmux select-pane -t 0
tmux attach-session -t $SESSION_NAME

# CONTROLS:
# Ctrl+b then arrow keys: Navigate between panes
# Ctrl+b z: Zoom current pane (toggle fullscreen)
# Ctrl+b [: Scroll mode (press q to exit)
# Ctrl+d or type 'exit': Close current pane
