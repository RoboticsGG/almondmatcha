#!/bin/bash
# ws_jetson Tmux Launch Script - Multi-Domain Architecture
# Domain 6: Vision processing (camera_stream, lane_detection)
# Domain 5: Control output — no separate bridge process
#
# This script launches all ws_jetson nodes in a tmux session with 4 panes:
#   [0] Camera Stream (Domain 6)
#   [1] Lane Detection (Domain 6)
#   [2] Rover Kinematic Control — dual-context (D6 sub tpc_rover_nav_lane | D5 pub tpc_rover_ctrl_cmd)
#   [3] Rover Local Monitoring — Domain 4 CSV logger (subscribes tpc_telemetry_relay)
#
# Architecture:
#   Domain 6: camera_stream → lane_detection → tpc_rover_nav_lane
#   rover_kinematic_control (single process, two contexts):
#     ctx_d6: subscribes tpc_rover_nav_lane, runs PID
#     ctx_d5: publishes tpc_rover_ctrl_cmd directly to chassis_controller (RPi)
#   No separate bridge node — dual-context handled inside rover_kinematic_control.

SESSION_NAME="jetson_vision"

# Kill existing session if exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Create new tmux session
tmux new-session -d -s $SESSION_NAME

# Create layout: 1 left pane (camera), 3 right panes stacked (detection, control, monitoring)
tmux split-window -h  # Split into left and right
tmux select-pane -t 1
tmux split-window -v  # Split right into top and bottom
tmux select-pane -t 2
tmux split-window -v  # Split right-bottom into middle and bottom

# Wait for panes to be created
sleep 0.5

# Enable pane titles and colorize borders
tmux set-option -g pane-border-status top
tmux set-option -g pane-border-format " [#{pane_index}] #{pane_title} "
tmux set-option -g pane-border-style fg=colour240
tmux set-option -g pane-active-border-style fg=colour51

# Pane 0 (left): Camera Stream - Domain 6
tmux select-pane -t 0 -T "Camera_D6"
tmux send-keys -t $SESSION_NAME:0.0 "source /opt/ros/humble/setup.bash && cd ~/almondmatcha/ws_jetson && source ~/almondmatcha/common_ifaces/install/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.0 "export ROS_DOMAIN_ID=6" C-m
tmux send-keys -t $SESSION_NAME:0.0 "clear && echo -e '\\e[1;36m>>> [Domain 6] CAMERA STREAM <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.0 "ros2 run vision_navigation camera_stream_node --ros-args --params-file src/vision_navigation/config/vision_nav_headless.yaml" C-m

# Pane 1 (top-right): Lane Detection - Domain 6
tmux select-pane -t 1 -T "Lane_Detect_D6"
tmux send-keys -t $SESSION_NAME:0.1 "source /opt/ros/humble/setup.bash && cd ~/almondmatcha/ws_jetson && source ~/almondmatcha/common_ifaces/install/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "export ROS_DOMAIN_ID=6" C-m
tmux send-keys -t $SESSION_NAME:0.1 "clear && echo -e '\\e[1;32m>>> [Domain 6] LANE DETECTION <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.1 "echo 'Waiting for camera initialization (3s)...' && sleep 3" C-m
tmux send-keys -t $SESSION_NAME:0.1 "ros2 run vision_navigation lane_detection_node --ros-args --params-file src/vision_navigation/config/vision_nav_headless.yaml" C-m

# Pane 2 (bottom-right): Rover Kinematic Control — dual-context (D6 sub | D5 pub)
tmux select-pane -t 2 -T "Kinematic_Ctrl_D6+D5"
tmux send-keys -t $SESSION_NAME:0.2 "source /opt/ros/humble/setup.bash && cd ~/almondmatcha/ws_jetson && source ~/almondmatcha/common_ifaces/install/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.2 "clear && echo -e '\\e[1;33m>>> [D6 sub + D5 pub] ROVER KINEMATIC CONTROL <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.2 "echo 'Waiting for lane detection (4s)...' && sleep 4" C-m
tmux send-keys -t $SESSION_NAME:0.2 "ros2 run vision_navigation rover_kinematic_control --ros-args --params-file src/vision_navigation/config/rover_kinematic_control_params.yaml" C-m

# Pane 3 (bottom-right): Rover Local Monitoring — Domain 4 CSV logger
tmux select-pane -t 3 -T "Local_Monitor_D4"
tmux send-keys -t $SESSION_NAME:0.3 "source /opt/ros/humble/setup.bash && cd ~/almondmatcha/ws_jetson && source ~/almondmatcha/common_ifaces/install/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.3 "export ROS_DOMAIN_ID=4" C-m
tmux send-keys -t $SESSION_NAME:0.3 "clear && echo -e '\\e[1;35m>>> [Domain 4] ROVER LOCAL MONITORING (CSV) <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.3 "echo 'Waiting for telemetry relay (5s)...' && sleep 5" C-m
tmux send-keys -t $SESSION_NAME:0.3 "ros2 run rover_monitoring rover_local_monitoring_node" C-m

# Focus on camera pane and attach
tmux select-pane -t 0
tmux attach-session -t $SESSION_NAME

# CONTROLS:
# Ctrl+b then arrow keys: Navigate between panes
# Ctrl+b z: Zoom current pane (toggle fullscreen)
# Ctrl+b [: Scroll mode (press q to exit, arrow keys to navigate)
# Ctrl+d or type 'exit': Close current pane
#
# SHUTDOWN:
# tmux kill-session -t jetson_vision
# OR: Ctrl+b, then type ':kill-session' and press Enter
# OR: Close all panes individually with Ctrl+d

