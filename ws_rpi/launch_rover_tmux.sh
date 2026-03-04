#!/bin/bash
# ws_rpi Tmux Launch Script - Centralized Monitoring Architecture
# Domain 5: All rover nodes (GNSS, chassis, sensors, monitoring)
# Layout: 2 windows x 2x2 panes (4 panes each, always fits any terminal size)
#   Window 0 [GNSS+Chassis]:   0.0 GNSS_Spresense  | 0.1 GNSS_Ublox
#                               0.2 GNSS_Mission    | 0.3 Chassis_Controller
#   Window 1 [Sensors+Monitor]: 1.0 Chassis_IMU     | 1.1 Chassis_Sensors
#                               1.2 Mission_Monitor | 1.3 CSV_Logger

SESSION_NAME="rover"
SRC="cd ~/almondmatcha/ws_rpi && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=5"

# Kill existing session if exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

# ── Window 0: GNSS + Chassis Controller ──────────────────────────────────────
tmux new-session  -d -s $SESSION_NAME -n "GNSS_Chassis"
tmux split-window -h  -t $SESSION_NAME:0        # 0.0=left-top  | 0.1=right-top
tmux split-window -v  -t $SESSION_NAME:0.0      # 0.0=left-top  | 0.2=left-bot
tmux split-window -v  -t $SESSION_NAME:0.1      # 0.1=right-top | 0.3=right-bot

# ── Window 1: Chassis Sensors + Monitoring ────────────────────────────────────
tmux new-window    -t $SESSION_NAME:1 -n "Sensors_Monitor"
tmux split-window -h  -t $SESSION_NAME:1        # 1.0=left-top  | 1.1=right-top
tmux split-window -v  -t $SESSION_NAME:1.0      # 1.0=left-top  | 1.2=left-bot
tmux split-window -v  -t $SESSION_NAME:1.1      # 1.1=right-top | 1.3=right-bot

sleep 0.5

# Enable pane titles and colorize borders
tmux set-option -g pane-border-status top
tmux set-option -g pane-border-format " [#{window_index}.#{pane_index}] #{pane_title} "
tmux set-option -g pane-border-style fg=colour240
tmux set-option -g pane-active-border-style fg=colour51

# ── Window 0 panes ────────────────────────────────────────────────────────────

# Pane 0.0 (top-left): GNSS Spresense
tmux select-pane -t $SESSION_NAME:0.0 -T "GNSS_Spresense"
tmux send-keys -t $SESSION_NAME:0.0 "$SRC" C-m
tmux send-keys -t $SESSION_NAME:0.0 "clear && echo -e '\\e[1;36m>>> [1/8] GNSS SPRESENSE <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.0 "ros2 run gnss_navigation gnss_spresense_node" C-m

# Pane 0.1 (top-right): GNSS Ublox RTK
tmux select-pane -t $SESSION_NAME:0.1 -T "GNSS_Ublox_RTK"
tmux send-keys -t $SESSION_NAME:0.1 "$SRC" C-m
tmux send-keys -t $SESSION_NAME:0.1 "clear && echo -e '\\e[1;32m>>> [2/8] GNSS UBLOX RTK <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.1 "ros2 run gnss_navigation gnss_ublox_node" C-m

# Pane 0.2 (bottom-left): GNSS Mission Monitor
tmux select-pane -t $SESSION_NAME:0.2 -T "GNSS_Mission"
tmux send-keys -t $SESSION_NAME:0.2 "$SRC" C-m
tmux send-keys -t $SESSION_NAME:0.2 "clear && echo -e '\\e[1;33m>>> [3/8] GNSS MISSION MONITOR <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.2 "ros2 run gnss_navigation gnss_mission_monitor_node" C-m

# Pane 0.3 (bottom-right): Chassis Controller
tmux select-pane -t $SESSION_NAME:0.3 -T "Chassis_Controller"
tmux send-keys -t $SESSION_NAME:0.3 "$SRC" C-m
tmux send-keys -t $SESSION_NAME:0.3 "clear && echo -e '\\e[1;35m>>> [4/8] CHASSIS CONTROLLER <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:0.3 "ros2 run chassis_control chassis_controller_node" C-m

# ── Window 1 panes ────────────────────────────────────────────────────────────

# Pane 1.0 (top-left): Chassis IMU
tmux select-pane -t $SESSION_NAME:1.0 -T "Chassis_IMU"
tmux send-keys -t $SESSION_NAME:1.0 "$SRC" C-m
tmux send-keys -t $SESSION_NAME:1.0 "clear && echo -e '\\e[1;34m>>> [5/8] CHASSIS IMU <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:1.0 "ros2 run chassis_sensors chassis_imu_node" C-m

# Pane 1.1 (top-right): Chassis Sensors
tmux select-pane -t $SESSION_NAME:1.1 -T "Chassis_Sensors"
tmux send-keys -t $SESSION_NAME:1.1 "$SRC" C-m
tmux send-keys -t $SESSION_NAME:1.1 "clear && echo -e '\\e[1;31m>>> [6/8] CHASSIS SENSORS <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:1.1 "ros2 run chassis_sensors chassis_sensors_node" C-m

# Pane 1.2 (bottom-left): Mission Monitoring RPi (Telemetry Relay)
tmux select-pane -t $SESSION_NAME:1.2 -T "Mission_Monitor_RPi"
tmux send-keys -t $SESSION_NAME:1.2 "$SRC" C-m
tmux send-keys -t $SESSION_NAME:1.2 "clear && echo -e '\\e[1;93m>>> [7/8] MISSION MONITORING (TELEMETRY RELAY) <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:1.2 "ros2 run rover_monitoring mission_monitoring_node_rpi" C-m

# Pane 1.3 (bottom-right): CSV Data Logger
tmux select-pane -t $SESSION_NAME:1.3 -T "CSV_Logger"
tmux send-keys -t $SESSION_NAME:1.3 "$SRC" C-m
tmux send-keys -t $SESSION_NAME:1.3 "clear && echo -e '\\e[1;96m>>> [8/8] CSV DATA LOGGER <<<\\e[0m' && sleep 1" C-m
tmux send-keys -t $SESSION_NAME:1.3 "ros2 run rover_monitoring rover_monitoring_node" C-m

# Start on window 0 and attach
tmux select-window -t $SESSION_NAME:0
tmux select-pane -t $SESSION_NAME:0.0
tmux attach-session -t $SESSION_NAME

# CONTROLS:
# Ctrl+b n / Ctrl+b p : Next / previous window
# Ctrl+b arrow keys   : Navigate between panes
# Ctrl+b z            : Zoom current pane (toggle fullscreen)
# Ctrl+b [            : Scroll mode (press q to exit)
# Ctrl+d or 'exit'    : Close current pane
