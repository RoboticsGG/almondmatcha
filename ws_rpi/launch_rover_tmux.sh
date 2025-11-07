#!/bin/bash
# Rover Launch with Tmux - 4x2 Grid Layout (7 nodes)
# Domain 5: All rover internal nodes (GNSS, chassis, sensors, bridge D5 side)
# Domain 2: Bridge node D2 side for base station communication
# Launch all rover nodes in separate terminal panes

SESSION_NAME="rover"

# Kill existing session if exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Create new tmux session
tmux new-session -d -s $SESSION_NAME

# Create 4x2 grid (7 panes used, 1 reserved)
tmux split-window -h  # Split into left and right columns
tmux select-pane -t 0
tmux split-window -v  # Split left column into 2
tmux split-window -v  # Split left column into 3
tmux split-window -v  # Split left column into 4 (final)
tmux select-pane -t 4
tmux split-window -v  # Split right column into 2
tmux split-window -v  # Split right column into 3 (final)

# Wait for panes to be created
sleep 0.5

# Enable pane titles and colorize borders
tmux set-option -g pane-border-status top
tmux set-option -g pane-border-format " [#{pane_index}] #{pane_title} "
tmux set-option -g pane-border-style fg=colour240
tmux set-option -g pane-active-border-style fg=colour51

# Pane 0 (top-left): GNSS Spresense (Domain 5)
tmux select-pane -t 0 -T "GNSS_Spresense_D5"
tmux send-keys -t $SESSION_NAME:0.0 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.0 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.0 "clear && echo -e '\\e[1;36m>>> [1/7] GNSS SPRESENSE (Domain 5) <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.0 "ros2 run pkg_gnss_navigation node_gnss_spresense" C-m

# Pane 1 (second from top-left): GNSS Mission Monitor (Domain 5)
tmux select-pane -t 1 -T "GNSS_Mission_Monitor_D5"
tmux send-keys -t $SESSION_NAME:0.1 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.1 "clear && echo -e '\\e[1;32m>>> [2/7] GNSS MISSION MONITOR (Domain 5) <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.1 "ros2 run pkg_gnss_navigation node_gnss_mission_monitor" C-m

# Pane 2 (third from top-left): Chassis Controller (Domain 5)
tmux select-pane -t 2 -T "Chassis_Controller_D5"
tmux send-keys -t $SESSION_NAME:0.2 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.2 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.2 "clear && echo -e '\\e[1;33m>>> [3/7] CHASSIS CONTROLLER (Domain 5) <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.2 "ros2 run pkg_chassis_control node_chassis_controller" C-m

# Pane 3 (bottom-left): Chassis IMU (Domain 5)
tmux select-pane -t 3 -T "Chassis_IMU_D5"
tmux send-keys -t $SESSION_NAME:0.3 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.3 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.3 "clear && echo -e '\\e[1;35m>>> [4/7] CHASSIS IMU (Domain 5) <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.3 "ros2 run pkg_chassis_sensors node_chassis_imu" C-m

# Pane 4 (top-right): Chassis Sensors (Domain 5)
tmux select-pane -t 4 -T "Chassis_Sensors_D5"
tmux send-keys -t $SESSION_NAME:0.4 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.4 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.4 "clear && echo -e '\\e[1;34m>>> [5/7] CHASSIS SENSORS (Domain 5) <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.4 "ros2 run pkg_chassis_sensors node_chassis_sensors" C-m

# Pane 5 (second from top-right): Base Bridge Domain 5 Side
tmux select-pane -t 5 -T "Base_Bridge_D5"
tmux send-keys -t $SESSION_NAME:0.5 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.5 "export ROS_DOMAIN_ID=5" C-m
tmux send-keys -t $SESSION_NAME:0.5 "clear && echo -e '\\e[1;96m>>> [6/7] BASE BRIDGE (Domain 5 Subscriber) <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.5 "ros2 run pkg_base_bridge node_base_bridge" C-m

# Pane 6 (third from top-right): Base Bridge Domain 2 Side
tmux select-pane -t 6 -T "Base_Bridge_D2"
tmux send-keys -t $SESSION_NAME:0.6 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.6 "export ROS_DOMAIN_ID=2" C-m
tmux send-keys -t $SESSION_NAME:0.6 "clear && echo -e '\\e[1;93m>>> [7/7] BASE BRIDGE (Domain 2 Publisher) <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.6 "ros2 run pkg_base_bridge node_base_bridge" C-m

# Attach to the tmux session
tmux attach-session -t $SESSION_NAME

# Pane 7 (bottom-right): Reserved for monitoring or future EKF
tmux select-pane -t 7 -T "Reserved_Monitor"
tmux send-keys -t $SESSION_NAME:0.7 "cd ~/almondmatcha/ws_rpi && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.7 "clear && echo -e '\\e[1;37m>>> [RESERVED] Future: EKF or Monitoring <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.7 "echo -e '\\nUseful commands:'" C-m
tmux send-keys -t $SESSION_NAME:0.7 "echo -e '  Domain 5: export ROS_DOMAIN_ID=5 && ros2 topic list'" C-m
tmux send-keys -t $SESSION_NAME:0.7 "echo -e '  Domain 2: export ROS_DOMAIN_ID=2 && ros2 topic list'" C-m
tmux send-keys -t $SESSION_NAME:0.7 "echo -e '  Monitor:  ros2 topic hz tpc_chassis_imu'" C-m

# Focus on top-left pane and attach
tmux select-pane -t 0
tmux attach-session -t $SESSION_NAME

# CONTROLS:
# Ctrl+b then arrow keys: Navigate between panes
# Ctrl+b z: Zoom current pane (toggle fullscreen)
# Ctrl+b [: Scroll mode (press q to exit)
# Ctrl+d or type 'exit': Close current pane
