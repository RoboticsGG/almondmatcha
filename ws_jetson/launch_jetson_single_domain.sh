#!/bin/bash
# ws_jetson — Single-Domain POC Launch Script
# POC: ALL nodes on ROS_DOMAIN_ID=5 (was D6+D5+D4 in baseline)
#
# Changes vs launch_jetson_tmux.sh:
#   Pane 0  camera_stream_node         D6 → D5  (Image msgs now on network!)
#   Pane 1  lane_detection_node        D6 → D5
#   Pane 2  rover_kinematic_control    D6/D5 dual-ctx → single D5 env
#   Pane 3  rover_local_monitoring     D4 → D5  (subscribes tpc_telemetry_relay on D5)
#
# WARNING: camera_stream publishes sensor_msgs/Image at 30 FPS on D5.
# This WILL be visible to STM32 boards (which cannot subscribe to it but
# will receive RTPS announcements). That is exactly the stress being measured.

SESSION_NAME="jetson_poc"
ROS_SRC="source /opt/ros/humble/setup.bash && cd ~/almondmatcha/ws_jetson && source ~/almondmatcha/common_ifaces/install/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=5 && export FASTRTPS_DEFAULT_PROFILES_FILE=~/almondmatcha/ws_jetson/fastdds_jetson.xml"

tmux kill-session -t $SESSION_NAME 2>/dev/null

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

# Pane 0 (left): Camera Stream — D5 (was D6)
tmux select-pane -t 0 -T "Camera [D5 POC]"
tmux send-keys -t $SESSION_NAME:0.0 "$ROS_SRC" C-m
tmux send-keys -t $SESSION_NAME:0.0 "clear && echo -e '\\e[1;36m>>> [Domain 5 POC] CAMERA STREAM (was D6) <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.0 "ros2 run vision_navigation camera_stream_node --ros-args --params-file src/vision_navigation/config/vision_nav_headless.yaml 2>&1 | tee ~/ros2_traces/poc_camera_stream.log" C-m
echo "[$(date +%H:%M:%S)] [1/4] camera_stream_node sent to pane 0"
sleep 3

# Pane 1 (top-right): Lane Detection — D5 (was D6)
# Starts after camera is up (3 s script delay above).
tmux select-pane -t 1 -T "Lane_Detect [D5 POC]"
tmux send-keys -t $SESSION_NAME:0.1 "$ROS_SRC" C-m
tmux send-keys -t $SESSION_NAME:0.1 "clear && echo -e '\\e[1;32m>>> [Domain 5 POC] LANE DETECTION (was D6) <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.1 "ros2 run vision_navigation lane_detection_node --ros-args --params-file src/vision_navigation/config/vision_nav_headless.yaml 2>&1 | tee ~/ros2_traces/poc_lane_detection.log" C-m
echo "[$(date +%H:%M:%S)] [2/4] lane_detection_node sent to pane 1"
sleep 2

# Pane 2 (mid-right): Rover Kinematic Control — D5 only (was D6 sub + D5 pub)
# Starts after lane_detection is up (2 s script delay above).
tmux select-pane -t 2 -T "Kinematic_Ctrl [D5 POC]"
tmux send-keys -t $SESSION_NAME:0.2 "$ROS_SRC" C-m
tmux send-keys -t $SESSION_NAME:0.2 "clear && echo -e '\\e[1;33m>>> [Domain 5 POC] ROVER KINEMATIC CONTROL (all D5) <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.2 "ros2 run vision_navigation rover_kinematic_control --ros-args --params-file src/vision_navigation/config/rover_kinematic_control_params.yaml 2>&1 | tee ~/ros2_traces/poc_kinematic_ctrl.log" C-m
echo "[$(date +%H:%M:%S)] [3/4] rover_kinematic_control sent to pane 2"
sleep 2

# Pane 3 (bot-right): Rover Local Monitoring — D5 (was D4)
# tpc_telemetry_relay is now on D5; starts last as it depends on RPi telemetry.
tmux select-pane -t 3 -T "Local_Monitor [D5 POC]"
tmux send-keys -t $SESSION_NAME:0.3 "$ROS_SRC" C-m
tmux send-keys -t $SESSION_NAME:0.3 "clear && echo -e '\\e[1;35m>>> [Domain 5 POC] ROVER LOCAL MONITORING (was D4) <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.3 "ros2 run rover_monitoring rover_local_monitoring_node 2>&1 | tee ~/ros2_traces/poc_local_monitor.log" C-m
echo "[$(date +%H:%M:%S)] [4/4] rover_local_monitoring_node sent to pane 3"

tmux select-pane -t 0
[ -z "${SKIP_ATTACH:-}" ] && tmux attach-session -t $SESSION_NAME
