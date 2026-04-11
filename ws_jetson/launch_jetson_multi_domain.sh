#!/bin/bash
# ws_jetson — Multi-Domain POC Launch Script
# Multi-domain topology (same as main branch):
#   Pane 0  camera_stream_node         D6  (vision localhost)
#   Pane 1  lane_detection_node        D6  (vision localhost)
#   Pane 2  rover_kinematic_control    D6 sub → D5 pub  (dual-context, hardcoded in node)
#   Pane 3  rover_local_monitoring     D4  (subscribes tpc_telemetry_relay from RPi)
#
# D6 nodes (camera, lane_detection) do NOT use FASTRTPS_DEFAULT_PROFILES_FILE.
# They use default DDS with shared-memory transport for high-throughput localhost
# vision processing. The kinematic_control node internally creates two rclpy
# contexts (D6 subscriber, D5 publisher) — it also uses default DDS since its
# D5 publisher doesn't need STM32-specific multicast config.
#
# D4 local_monitoring uses default DDS (no STM32 on D4).

SESSION_NAME="jetson_poc"

# D6 source — vision pipeline nodes (no FastDDS XML, uses shared memory)
ROS_SRC_D6="source /opt/ros/humble/setup.bash && cd ~/almondmatcha/ws_jetson && source ~/almondmatcha/common_ifaces/install/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=6"

# D5/D6 dual-context source — kinematic_control handles domains internally
# No FASTRTPS_DEFAULT_PROFILES_FILE: the D5 publisher reaches RPi via standard
# DDS multicast, and the D6 subscriber uses shared memory with camera/lane nodes.
ROS_SRC_DUAL="source /opt/ros/humble/setup.bash && cd ~/almondmatcha/ws_jetson && source ~/almondmatcha/common_ifaces/install/setup.bash && source install/setup.bash"

# D4 source — local monitoring (subscribes to RPi telemetry relay)
ROS_SRC_D4="source /opt/ros/humble/setup.bash && cd ~/almondmatcha/ws_jetson && source ~/almondmatcha/common_ifaces/install/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=4"

tmux kill-session -t $SESSION_NAME 2>/dev/null

# Kill any stale ros2 run processes from a previous session.
# Also kill stale collector processes which consume STM32 participant table slots.
# Do NOT restart the ros2 daemon — see ws_rpi/launch_rover_single_domain.sh comment.
pkill -f "ros2 run" 2>/dev/null || true
pkill -f "collect_latency" 2>/dev/null || true
pkill -f "collect_net_stats" 2>/dev/null || true
sleep 2

tmux new-session -d -s $SESSION_NAME

# Same 4-pane layout as baseline
tmux split-window -h
tmux select-pane -t 1
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v

sleep 0.5

tmux set-option -g pane-border-status top
tmux set-option -g pane-border-format " [#{pane_index}] #{pane_title} "
tmux set-option -g pane-border-style fg=colour220        # yellow = POC mode
tmux set-option -g pane-active-border-style fg=colour196 # red = POC mode

# ── Staggered launch ─────────────────────────────────────────────────────────
# Script-level sleep between panes prevents a simultaneous SPDP burst on the
# network that would overflow the STM32 discovery queues.
# Vision pipeline ordering is preserved: camera → lane_detect → kinematic_ctrl.
# local_monitoring depends on tpc_telemetry_relay from the RPi, so it starts last.
# ─────────────────────────────────────────────────────────────────────────────

# Pane 0 (left): Camera Stream — D6 (vision localhost, shared memory)
tmux select-pane -t 0 -T "Camera [D6]"
tmux send-keys -t $SESSION_NAME:0.0 "$ROS_SRC_D6" C-m
tmux send-keys -t $SESSION_NAME:0.0 "clear && echo -e '\\e[1;36m>>> [D6] CAMERA STREAM (vision localhost) <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.0 "ros2 run vision_navigation camera_stream_node --ros-args --params-file src/vision_navigation/config/vision_nav_headless.yaml 2>&1 | tee ~/ros2_traces/poc_camera_stream.log" C-m
echo "[$(date +%H:%M:%S)] [1/4] camera_stream_node sent to pane 0 (D6)"
sleep 3

# Pane 1 (top-right): Lane Detection — D6 (vision localhost, shared memory)
# Starts after camera is up (3 s script delay above).
tmux select-pane -t 1 -T "Lane_Detect [D6]"
tmux send-keys -t $SESSION_NAME:0.1 "$ROS_SRC_D6" C-m
tmux send-keys -t $SESSION_NAME:0.1 "clear && echo -e '\\e[1;32m>>> [D6] LANE DETECTION (vision localhost) <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.1 "ros2 run vision_navigation lane_detection_node --ros-args --params-file src/vision_navigation/config/vision_nav_headless.yaml 2>&1 | tee ~/ros2_traces/poc_lane_detection.log" C-m
echo "[$(date +%H:%M:%S)] [2/4] lane_detection_node sent to pane 1 (D6)"
sleep 2

# Pane 2 (mid-right): Rover Kinematic Control — D6 sub + D5 pub (dual-context)
# The node internally creates two rclpy contexts: D6 subscriber for lane data,
# D5 publisher for control commands. No ROS_DOMAIN_ID env var needed — the node
# sets os.environ['ROS_DOMAIN_ID'] for each context in its own code.
tmux select-pane -t 2 -T "Kinematic_Ctrl [D6→D5]"
tmux send-keys -t $SESSION_NAME:0.2 "$ROS_SRC_DUAL" C-m
tmux send-keys -t $SESSION_NAME:0.2 "clear && echo -e '\\e[1;33m>>> [D6→D5] ROVER KINEMATIC CONTROL (dual-context) <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.2 "ros2 run vision_navigation rover_kinematic_control --ros-args --params-file src/vision_navigation/config/rover_kinematic_control_params.yaml 2>&1 | tee ~/ros2_traces/poc_kinematic_ctrl.log" C-m
echo "[$(date +%H:%M:%S)] [3/4] rover_kinematic_control sent to pane 2 (D6→D5)"
sleep 2

# Pane 3 (bot-right): Rover Local Monitoring — D4
# Subscribes to tpc_telemetry_relay published by mission_monitoring_node_rpi on D4.
tmux select-pane -t 3 -T "Local_Monitor [D4]"
tmux send-keys -t $SESSION_NAME:0.3 "$ROS_SRC_D4" C-m
tmux send-keys -t $SESSION_NAME:0.3 "clear && echo -e '\\e[1;35m>>> [D4] ROVER LOCAL MONITORING (telemetry relay) <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.3 "ros2 run rover_monitoring rover_local_monitoring_node 2>&1 | tee ~/ros2_traces/poc_local_monitor.log" C-m
echo "[$(date +%H:%M:%S)] [4/4] rover_local_monitoring_node sent to pane 3 (D4)"

tmux select-pane -t 0
[ -z "${SKIP_ATTACH:-}" ] && tmux attach-session -t $SESSION_NAME
