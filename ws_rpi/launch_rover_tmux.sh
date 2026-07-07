#!/bin/bash
# ws_rpi Tmux Launch Script - Centralized Monitoring Architecture
# Domain 5: All rover nodes (GNSS, chassis, sensors, monitoring)
# Layout: single window, 3x3 grid (9 panes, pane 8 = spare shell)
#
#  col→  LEFT          MIDDLE          RIGHT
#  row0  0:GNSS_Spres  1:Chassis_Ctrl  2:Mission_Monitor
#  row1  3:GNSS_Ublox  5:Chassis_IMU   7:CSV_Logger
#  row2  4:GNSS_Miss   6:Chassis_Sens  8:(spare)

SESSION_NAME="rover"
SRC="cd ~/almondmatcha/ws_rpi && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=5"

# Kill existing session if exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

# ── Build 3-column layout ─────────────────────────────────────────────────────
# Start: pane 0 (full window)
tmux new-session -d -s $SESSION_NAME -n "rover"

# Create 3 equal columns using percentage splits
tmux split-window  -h -p 67 -t $SESSION_NAME:0   # pane 0 (33%) | pane 1 (67%)
tmux select-pane   -t $SESSION_NAME:0.1
tmux split-window  -h -p 50 -t $SESSION_NAME:0.1 # pane 1 (33%) | pane 2 (33%)

# Split left column (pane 0) into 3 rows
tmux select-pane   -t $SESSION_NAME:0.0
tmux split-window  -v -p 67 -t $SESSION_NAME:0.0 # pane 3 (bot 67% of left)
tmux select-pane   -t $SESSION_NAME:0.3
tmux split-window  -v -p 50 -t $SESSION_NAME:0.3 # pane 4 (bot 50% of that)

# Split middle column (pane 1) into 3 rows
tmux select-pane   -t $SESSION_NAME:0.1
tmux split-window  -v -p 67 -t $SESSION_NAME:0.1 # pane 5
tmux select-pane   -t $SESSION_NAME:0.5
tmux split-window  -v -p 50 -t $SESSION_NAME:0.5 # pane 6

# Split right column (pane 2) into 3 rows
tmux select-pane   -t $SESSION_NAME:0.2
tmux split-window  -v -p 67 -t $SESSION_NAME:0.2 # pane 7
tmux select-pane   -t $SESSION_NAME:0.7
tmux split-window  -v -p 50 -t $SESSION_NAME:0.7 # pane 8 (spare)

sleep 0.5

# Enable pane titles and colorize borders
tmux set-option -g pane-border-status top
tmux set-option -g pane-border-format " [#{pane_index}] #{pane_title} "
tmux set-option -g pane-border-style fg=colour240
tmux set-option -g pane-active-border-style fg=colour51

# ── LEFT column ───────────────────────────────────────────────────────────────

# Pane 0 (top-left): GNSS Spresense
tmux select-pane -t $SESSION_NAME:0.0 -T "GNSS_Spresense"
tmux send-keys   -t $SESSION_NAME:0.0 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.0 "clear && echo -e '\\e[1;36m>>> [1/8] GNSS SPRESENSE <<<\\e[0m' && sleep 1" C-m
tmux send-keys   -t $SESSION_NAME:0.0 "ros2 run gnss_navigation gnss_spresense_node" C-m

# Pane 3 (mid-left): GNSS Ublox RTK
tmux select-pane -t $SESSION_NAME:0.3 -T "GNSS_Ublox_RTK"
tmux send-keys   -t $SESSION_NAME:0.3 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.3 "clear && echo -e '\\e[1;32m>>> [2/8] GNSS UBLOX RTK <<<\\e[0m' && sleep 1" C-m
tmux send-keys   -t $SESSION_NAME:0.3 "ros2 run gnss_navigation gnss_ublox_node" C-m

# Pane 4 (bot-left): GNSS Mission Monitor
tmux select-pane -t $SESSION_NAME:0.4 -T "GNSS_Mission"
tmux send-keys   -t $SESSION_NAME:0.4 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.4 "clear && echo -e '\\e[1;33m>>> [3/8] GNSS MISSION MONITOR <<<\\e[0m' && sleep 1" C-m
tmux send-keys   -t $SESSION_NAME:0.4 "ros2 run gnss_navigation gnss_mission_monitor_node" C-m

# ── MIDDLE column ─────────────────────────────────────────────────────────────

# Pane 1 (top-mid): Chassis Controller
tmux select-pane -t $SESSION_NAME:0.1 -T "Chassis_Controller"
tmux send-keys   -t $SESSION_NAME:0.1 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.1 "clear && echo -e '\\e[1;35m>>> [4/8] CHASSIS CONTROLLER <<<\\e[0m' && sleep 1" C-m
tmux send-keys   -t $SESSION_NAME:0.1 "ros2 run chassis_control chassis_controller_node" C-m

# Pane 5 (mid-mid): Chassis IMU
tmux select-pane -t $SESSION_NAME:0.5 -T "Chassis_IMU"
tmux send-keys   -t $SESSION_NAME:0.5 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.5 "clear && echo -e '\\e[1;34m>>> [5/8] CHASSIS IMU <<<\\e[0m' && sleep 1" C-m
tmux send-keys   -t $SESSION_NAME:0.5 "ros2 run chassis_sensors chassis_imu_node" C-m

# Pane 6 (bot-mid): Chassis Sensors
tmux select-pane -t $SESSION_NAME:0.6 -T "Chassis_Sensors"
tmux send-keys   -t $SESSION_NAME:0.6 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.6 "clear && echo -e '\\e[1;31m>>> [6/8] CHASSIS SENSORS <<<\\e[0m' && sleep 1" C-m
tmux send-keys   -t $SESSION_NAME:0.6 "ros2 run chassis_sensors chassis_sensors_node" C-m

# ── RIGHT column ──────────────────────────────────────────────────────────────

# Pane 2 (top-right): Mission Monitoring RPi (Telemetry Relay)
tmux select-pane -t $SESSION_NAME:0.2 -T "Mission_Monitor_RPi"
tmux send-keys   -t $SESSION_NAME:0.2 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.2 "clear && echo -e '\\e[1;93m>>> [7/8] MISSION MONITORING (TELEMETRY RELAY) <<<\\e[0m' && sleep 1" C-m
tmux send-keys   -t $SESSION_NAME:0.2 "ros2 run rover_monitoring mission_monitoring_node_rpi" C-m

# Pane 7 (mid-right): CSV Data Logger
tmux select-pane -t $SESSION_NAME:0.7 -T "CSV_Logger"
tmux send-keys   -t $SESSION_NAME:0.7 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.7 "clear && echo -e '\\e[1;96m>>> [8/8] CSV DATA LOGGER <<<\\e[0m' && sleep 1" C-m
tmux send-keys   -t $SESSION_NAME:0.7 "ros2 run rover_monitoring rover_monitoring_node" C-m

# Pane 8 (bot-right): spare shell
tmux select-pane -t $SESSION_NAME:0.8 -T "Spare_Shell"
tmux send-keys   -t $SESSION_NAME:0.8 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.8 "clear && echo -e '\\e[1;90m>>> [SPARE] shell ready <<<\\e[0m'" C-m

# Focus on Mission Monitor pane and attach
tmux select-pane -t $SESSION_NAME:0.2
[[ "${SKIP_ATTACH:-0}" != "1" ]] && tmux attach-session -t $SESSION_NAME

# CONTROLS:
# Ctrl+b arrow keys : Navigate between panes
# Ctrl+b z          : Zoom current pane (toggle fullscreen)
# Ctrl+b [          : Scroll mode (press q to exit)
# Ctrl+d or 'exit'  : Close current pane
