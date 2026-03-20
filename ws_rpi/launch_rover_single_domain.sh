#!/bin/bash
# ws_rpi — Single-Domain POC Launch Script
# POC: all nodes on ROS_DOMAIN_ID=5 (unchanged from baseline; RPi was already D5)
# This script is identical in node set to launch_rover_tmux.sh but uses a distinct
# tmux session name ("rover_poc") so both can coexist during cross-comparisons.
#
# NOTE: Run this script AFTER deploying the single-domain scripts on Jetson and Base,
#       and AFTER flashing the updated STM32 firmware (MAX_NUM_PARTICIPANTS=20,
#       discovery delay raised to 12 s for the 15-participant single-domain topology).
#
#  col→  LEFT          MIDDLE          RIGHT
#  row0  0:GNSS_Spres  1:Chassis_Ctrl  2:Mission_Monitor
#  row1  3:GNSS_Ublox  5:Chassis_IMU   7:CSV_Logger
#  row2  4:GNSS_Miss   6:Chassis_Sens  8:(spare/trace)

SESSION_NAME="rover_poc"
# D5 for every pane — same as baseline; kept explicit for clarity
SRC="cd ~/almondmatcha/ws_rpi && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=5"

tmux kill-session -t $SESSION_NAME 2>/dev/null

# ── Build 3-column layout ─────────────────────────────────────────────────────
tmux new-session -d -s $SESSION_NAME -n "rover_poc"

tmux split-window  -h -p 67 -t $SESSION_NAME:0
tmux select-pane   -t $SESSION_NAME:0.1
tmux split-window  -h -p 50 -t $SESSION_NAME:0.1

tmux select-pane   -t $SESSION_NAME:0.0
tmux split-window  -v -p 67 -t $SESSION_NAME:0.0
tmux select-pane   -t $SESSION_NAME:0.3
tmux split-window  -v -p 50 -t $SESSION_NAME:0.3

tmux select-pane   -t $SESSION_NAME:0.1
tmux split-window  -v -p 67 -t $SESSION_NAME:0.1
tmux select-pane   -t $SESSION_NAME:0.5
tmux split-window  -v -p 50 -t $SESSION_NAME:0.5

tmux select-pane   -t $SESSION_NAME:0.2
tmux split-window  -v -p 67 -t $SESSION_NAME:0.2
tmux select-pane   -t $SESSION_NAME:0.7
tmux split-window  -v -p 50 -t $SESSION_NAME:0.7

sleep 0.5

tmux set-option -g pane-border-status top
tmux set-option -g pane-border-format " [#{pane_index}] #{pane_title} "
tmux set-option -g pane-border-style fg=colour220       # yellow = POC mode
tmux set-option -g pane-active-border-style fg=colour196 # red = POC mode

# ── LEFT column ───────────────────────────────────────────────────────────────
tmux select-pane -t $SESSION_NAME:0.0 -T "GNSS_Spresense [D5]"
tmux send-keys   -t $SESSION_NAME:0.0 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.0 "clear && echo -e '\\e[1;36m>>> [1/8] GNSS SPRESENSE  [POC D5] <<<\\e[0m' && sleep 1" C-m
tmux send-keys   -t $SESSION_NAME:0.0 "ros2 run gnss_navigation gnss_spresense_node" C-m

tmux select-pane -t $SESSION_NAME:0.3 -T "GNSS_Ublox_RTK [D5]"
tmux send-keys   -t $SESSION_NAME:0.3 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.3 "clear && echo -e '\\e[1;32m>>> [2/8] GNSS UBLOX RTK  [POC D5] <<<\\e[0m' && sleep 1" C-m
tmux send-keys   -t $SESSION_NAME:0.3 "ros2 run gnss_navigation gnss_ublox_node" C-m

tmux select-pane -t $SESSION_NAME:0.4 -T "GNSS_Mission [D5]"
tmux send-keys   -t $SESSION_NAME:0.4 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.4 "clear && echo -e '\\e[1;33m>>> [3/8] GNSS MISSION MONITOR  [POC D5] <<<\\e[0m' && sleep 1" C-m
tmux send-keys   -t $SESSION_NAME:0.4 "ros2 run gnss_navigation gnss_mission_monitor_node" C-m

# ── MIDDLE column ─────────────────────────────────────────────────────────────
tmux select-pane -t $SESSION_NAME:0.1 -T "Chassis_Controller [D5]"
tmux send-keys   -t $SESSION_NAME:0.1 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.1 "clear && echo -e '\\e[1;35m>>> [4/8] CHASSIS CONTROLLER  [POC D5] <<<\\e[0m' && sleep 1" C-m
tmux send-keys   -t $SESSION_NAME:0.1 "ros2 run chassis_control chassis_controller_node" C-m

tmux select-pane -t $SESSION_NAME:0.5 -T "Chassis_IMU [D5]"
tmux send-keys   -t $SESSION_NAME:0.5 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.5 "clear && echo -e '\\e[1;34m>>> [5/8] CHASSIS IMU  [POC D5] <<<\\e[0m' && sleep 1" C-m
tmux send-keys   -t $SESSION_NAME:0.5 "ros2 run chassis_sensors chassis_imu_node" C-m

tmux select-pane -t $SESSION_NAME:0.6 -T "Chassis_Sensors [D5]"
tmux send-keys   -t $SESSION_NAME:0.6 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.6 "clear && echo -e '\\e[1;31m>>> [6/8] CHASSIS SENSORS  [POC D5] <<<\\e[0m' && sleep 1" C-m
tmux send-keys   -t $SESSION_NAME:0.6 "ros2 run chassis_sensors chassis_sensors_node" C-m

# ── RIGHT column ──────────────────────────────────────────────────────────────
# mission_monitoring_node_rpi now stays on D5 (was already D5).
# In single-domain: tpc_telemetry_relay is visible to ALL nodes including STM32.
tmux select-pane -t $SESSION_NAME:0.2 -T "Mission_Monitor_RPi [D5]"
tmux send-keys   -t $SESSION_NAME:0.2 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.2 "clear && echo -e '\\e[1;93m>>> [7/8] MISSION MONITORING  [POC D5] <<<\\e[0m' && sleep 1" C-m
tmux send-keys   -t $SESSION_NAME:0.2 "ros2 run rover_monitoring mission_monitoring_node_rpi" C-m

tmux select-pane -t $SESSION_NAME:0.7 -T "CSV_Logger [D5]"
tmux send-keys   -t $SESSION_NAME:0.7 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.7 "clear && echo -e '\\e[1;96m>>> [8/8] CSV DATA LOGGER  [POC D5] <<<\\e[0m' && sleep 1" C-m
tmux send-keys   -t $SESSION_NAME:0.7 "ros2 run rover_monitoring rover_monitoring_node" C-m

# Pane 8 — spare / use for trace status or ad-hoc ros2 topic hz checks
tmux select-pane -t $SESSION_NAME:0.8 -T "Trace_Monitor"
tmux send-keys   -t $SESSION_NAME:0.8 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.8 "clear && echo -e '\\e[1;90m>>> [TRACE / SPARE]  ROS2 topic hz monitoring available <<<\\e[0m'" C-m
tmux send-keys   -t $SESSION_NAME:0.8 "echo 'Tip: ros2 topic hz /tpc_chassis_imu --window 50'" C-m

tmux select-pane -t $SESSION_NAME:0.2
tmux attach-session -t $SESSION_NAME
