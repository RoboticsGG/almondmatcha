#!/bin/bash
# ws_jetson Tmux Launch Script - Multi-Domain Architecture
# Domain 6: Vision processing (camera_stream, lane_detection, steering_control)
# Domain 5: Control relay via bridge
#
# This script launches all ws_jetson nodes in a tmux session with 4 panes:
#   [0] Camera Stream (Domain 6)
#   [1] Lane Detection (Domain 6)
#   [2] Steering Control (Domain 6)
#   [3] Domain Bridge (D6→D5 relay for tpc_rover_fmctl only)
#
# Architecture:
#   Domain 6: camera_stream → lane_detection → tpc_rover_nav_lane
#             steering_control subscribes to tpc_rover_nav_lane → publishes tpc_rover_fmctl
#   Bridge: tpc_rover_fmctl (D6) → tpc_rover_fmctl (D5)
#   Domain 5: Only receives final control command (less network traffic)

SESSION_NAME="jetson_vision"

# Kill existing session if exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Create new tmux session
tmux new-session -d -s $SESSION_NAME

# Create layout: 1 left pane (camera), 3 right panes stacked (detection, bridge, control)
tmux split-window -h  # Split into left and right
tmux select-pane -t 1
tmux split-window -v  # Split right into 2
tmux select-pane -t 2
tmux split-window -v  # Split into 3

# Wait for panes to be created
sleep 0.5

# Enable pane titles and colorize borders
tmux set-option -g pane-border-status top
tmux set-option -g pane-border-format " [#{pane_index}] #{pane_title} "
tmux set-option -g pane-border-style fg=colour240
tmux set-option -g pane-active-border-style fg=colour51

# Pane 0 (left): Camera Stream - Domain 6
tmux select-pane -t 0 -T "Camera_D6"
tmux send-keys -t $SESSION_NAME:0.0 "cd ~/almondmatcha/ws_jetson && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.0 "export ROS_DOMAIN_ID=6" C-m
tmux send-keys -t $SESSION_NAME:0.0 "clear && echo -e '\\e[1;36m>>> [Domain 6] CAMERA STREAM <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.0 "ros2 run vision_navigation camera_stream --ros-args --params-file vision_navigation/config/vision_nav_headless.yaml" C-m

# Pane 1 (top-right): Lane Detection - Domain 6
tmux select-pane -t 1 -T "Lane_Detect_D6"
tmux send-keys -t $SESSION_NAME:0.1 "cd ~/almondmatcha/ws_jetson && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "export ROS_DOMAIN_ID=6" C-m
tmux send-keys -t $SESSION_NAME:0.1 "clear && echo -e '\\e[1;32m>>> [Domain 6] LANE DETECTION <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.1 "echo 'Waiting for camera initialization (3s)...' && sleep 3" C-m
tmux send-keys -t $SESSION_NAME:0.1 "ros2 run vision_navigation lane_detection --ros-args --params-file vision_navigation/config/vision_nav_headless.yaml" C-m

# Pane 2 (middle-right): Steering Control - Domain 6
tmux select-pane -t 2 -T "Steering_D6"
tmux send-keys -t $SESSION_NAME:0.2 "cd ~/almondmatcha/ws_jetson && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.2 "export ROS_DOMAIN_ID=6" C-m
tmux send-keys -t $SESSION_NAME:0.2 "clear && echo -e '\\e[1;33m>>> [Domain 6] STEERING CONTROL <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.2 "echo 'Waiting for lane detection (4s)...' && sleep 4" C-m
tmux send-keys -t $SESSION_NAME:0.2 "ros2 run vision_navigation steering_control_domain5 --ros-args --params-file vision_navigation/config/steering_control_params.yaml" C-m

# Pane 3 (bottom-right): Domain Bridge (D6→D5) - tpc_rover_fmctl only
tmux select-pane -t 3 -T "Bridge_D6→D5"
tmux send-keys -t $SESSION_NAME:0.3 "cd ~/almondmatcha/ws_jetson && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.3 "clear && echo -e '\\e[1;96m>>> [Bridge] D6→D5 CONTROL CMD RELAY <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.3 "echo 'Waiting for steering control (5s)...' && sleep 5" C-m
tmux send-keys -t $SESSION_NAME:0.3 "python3 install/vision_navigation/lib/vision_navigation/domain_bridge_jetson.py" C-m

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

