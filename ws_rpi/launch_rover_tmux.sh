#!/bin/bash
# ws_rpi Tmux Launch Script - Centralized Monitoring Architecture
# Domain 5: All rover nodes (GNSS, chassis, sensors, monitoring)
# Launch all rover nodes in separate terminal panes

SESSION_NAME="rover"

# Kill existing session if exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Create new tmux session
tmux new-session -d -s $SESSION_NAME

# Create 3x3 grid (8 panes total)
tmux split-window -h  # Split into left and right columns
tmux select-pane -t 0
tmux split-window -v  # Split left into 2
tmux split-window -v  # Split left into 3
tmux select-pane -t 3
tmux split-window -v  # Split right into 2
tmux split-window -v  # Split right into 3
tmux split-window -v  # Split right into 4

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
tmux send-keys -t $SESSION_NAME:0.0 "clear && echo -e '\\e[1;36m>>> [1/8] GNSS SPRESENSE <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.0 "ros2 run pkg_gnss_navigation node_gnss_spresense" C-m

# Pane 1 (middle-left): GNSS Ublox RTK
tmux select-pane -t 1 -T "GNSS_Ublox_RTK"
tmux send-keys -t $SESSION_NAME:0.1 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.1 "clear && echo -e '\\e[1;32m>>> [2/8] GNSS UBLOX RTK <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.1 "ros2 run pkg_gnss_navigation node_gnss_ublox" C-m

# Pane 2 (bottom-left): GNSS Mission Monitor
tmux select-pane -t 2 -T "GNSS_Mission"
tmux send-keys -t $SESSION_NAME:0.2 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.2 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.2 "clear && echo -e '\\e[1;33m>>> [3/8] GNSS MISSION MONITOR <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.2 "ros2 run pkg_gnss_navigation node_gnss_mission_monitor" C-m

# Pane 3 (top-right): Chassis Controller
tmux select-pane -t 3 -T "Chassis_Controller"
tmux send-keys -t $SESSION_NAME:0.3 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.3 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.3 "clear && echo -e '\\e[1;35m>>> [4/8] CHASSIS CONTROLLER <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.3 "ros2 run pkg_chassis_control node_chassis_controller" C-m

# Pane 4 (upper-middle-right): Chassis IMU
tmux select-pane -t 4 -T "Chassis_IMU"
tmux send-keys -t $SESSION_NAME:0.4 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.4 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.4 "clear && echo -e '\\e[1;34m>>> [5/8] CHASSIS IMU <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.4 "ros2 run pkg_chassis_sensors node_chassis_imu" C-m

# Pane 5 (lower-middle-right): Chassis Sensors
tmux select-pane -t 5 -T "Chassis_Sensors"
tmux send-keys -t $SESSION_NAME:0.5 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.5 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.5 "clear && echo -e '\\e[1;31m>>> [6/8] CHASSIS SENSORS <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.5 "ros2 run pkg_chassis_sensors node_chassis_sensors" C-m

# Pane 6 (second-bottom-right): Rover Monitoring (CSV Logger)
tmux select-pane -t 6 -T "Rover_Monitoring"
tmux send-keys -t $SESSION_NAME:0.6 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.6 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.6 "clear && echo -e '\\e[1;93m>>> [7/8] ROVER MONITORING (CSV LOGGER) <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.6 "ros2 run pkg_rover_monitoring node_rover_monitoring" C-m

# Pane 7 (bottom-right): Domain Relay (5→4)
tmux select-pane -t 7 -T "Domain_Relay"
tmux send-keys -t $SESSION_NAME:0.7 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.7 "clear && echo -e '\\e[1;96m>>> [8/8] DOMAIN RELAY (5→4) <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.7 "python3 install/pkg_rover_monitoring/lib/pkg_rover_monitoring/domain_relay.py" C-m

# Focus on monitoring pane and attach
tmux select-pane -t 6
tmux attach-session -t $SESSION_NAME

# CONTROLS:
# Ctrl+b then arrow keys: Navigate between panes
# Ctrl+b z: Zoom current pane (toggle fullscreen)
# Ctrl+b [: Scroll mode (press q to exit)
# Ctrl+d or type 'exit': Close current pane
