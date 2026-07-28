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


# One run directory per launch, shared by every node on this machine -- mirrors
# what ws_jetson already does, so both machines lay a run out the same way.
RUNS_DIR="$HOME/almondmatcha/ws_rpi/runs"
mkdir -p "$RUNS_DIR"
_next=$(( $(find "$RUNS_DIR" -maxdepth 1 -name 'run_*' -printf '%f\n' 2>/dev/null \
           | sed -n 's/^run_\([0-9]\{3\}\)_.*/\1/p' | sort -n | tail -1 | sed 's/^0*//') + 1 ))
export ROVER_RUN_DIR="$RUNS_DIR/$(printf 'run_%03d_%s' "$_next" "$(date +%Y%m%d_%H%M%S)")"
echo "[run] output directory for this launch: $ROVER_RUN_DIR"

# ============================================================================
# Per-node console logs
# ============================================================================
# Every node's stdout+stderr is teed to $ROVER_RUN_DIR/<node>.log alongside the
# CSVs, so a run is one self-contained folder and a node that dies at startup
# leaves its reason on disk. Without this the only copy of a startup error --
# e.g. camera_stream's "Failed to start RealSense pipeline ... Connected
# devices: [...]" -- lives in a tmux scrollback that is gone once the session
# is killed, which is exactly when it is needed.
#
# stdbuf -oL -eL and PYTHONUNBUFFERED=1 are required, not cosmetic: stdout
# becomes block-buffered the moment it is a pipe rather than a tty, so without
# them the log lags by kilobytes and a crash discards the buffered tail --
# losing precisely the lines that explain the crash.
export PYTHONUNBUFFERED=1
mkdir -p "$ROVER_RUN_DIR"

# run_logged <pane> <logname> <command>
# The command is passed single-quoted by every caller so that any $(...) inside
# it reaches the pane's shell intact and is evaluated there, not here.
run_logged() {
    local pane="$1" name="$2" cmd="$3"
    tmux send-keys -t $SESSION_NAME:$pane \
        "stdbuf -oL -eL $cmd 2>&1 | tee -a '$ROVER_RUN_DIR/$name.log'" C-m
}

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
run_logged 0.0 gnss_spresense 'ros2 run gnss_navigation gnss_spresense_node'

# Pane 3 (mid-left): GNSS Ublox RTK
tmux select-pane -t $SESSION_NAME:0.3 -T "GNSS_Ublox_RTK"
tmux send-keys   -t $SESSION_NAME:0.3 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.3 "clear && echo -e '\\e[1;32m>>> [2/8] GNSS UBLOX RTK <<<\\e[0m' && sleep 1" C-m
run_logged 0.3 gnss_ublox 'ros2 run gnss_navigation gnss_ublox_node'

# Pane 4 (bot-left): GNSS Mission Monitor
tmux select-pane -t $SESSION_NAME:0.4 -T "GNSS_Mission"
tmux send-keys   -t $SESSION_NAME:0.4 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.4 "clear && echo -e '\\e[1;33m>>> [3/8] GNSS MISSION MONITOR <<<\\e[0m' && sleep 1" C-m
run_logged 0.4 gnss_mission_monitor 'ros2 run gnss_navigation gnss_mission_monitor_node'

# ── MIDDLE column ─────────────────────────────────────────────────────────────

# Pane 1 (top-mid): Chassis Controller
tmux select-pane -t $SESSION_NAME:0.1 -T "Chassis_Controller"
tmux send-keys   -t $SESSION_NAME:0.1 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.1 "clear && echo -e '\\e[1;35m>>> [4/8] CHASSIS CONTROLLER <<<\\e[0m' && sleep 1" C-m
run_logged 0.1 chassis_controller 'ros2 run chassis_control chassis_controller_node --ros-args --params-file $(ros2 pkg prefix chassis_control)/share/chassis_control/config/chassis_speed_control_params.yaml'

# Pane 5 (mid-mid): Chassis IMU
tmux select-pane -t $SESSION_NAME:0.5 -T "Chassis_IMU"
tmux send-keys   -t $SESSION_NAME:0.5 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.5 "clear && echo -e '\\e[1;34m>>> [5/8] CHASSIS IMU <<<\\e[0m' && sleep 1" C-m
run_logged 0.5 chassis_imu 'ros2 run chassis_sensors chassis_imu_node'

# Pane 6 (bot-mid): Chassis Sensors
tmux select-pane -t $SESSION_NAME:0.6 -T "Chassis_Sensors"
tmux send-keys   -t $SESSION_NAME:0.6 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.6 "clear && echo -e '\\e[1;31m>>> [6/8] CHASSIS SENSORS <<<\\e[0m' && sleep 1" C-m
run_logged 0.6 chassis_sensors 'ros2 run chassis_sensors chassis_sensors_node'

# ── RIGHT column ──────────────────────────────────────────────────────────────

# Pane 2 (top-right): Mission Monitoring RPi (Telemetry Relay)
tmux select-pane -t $SESSION_NAME:0.2 -T "Mission_Monitor_RPi"
tmux send-keys   -t $SESSION_NAME:0.2 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.2 "clear && echo -e '\\e[1;93m>>> [7/8] MISSION MONITORING (TELEMETRY RELAY) <<<\\e[0m' && sleep 1" C-m
run_logged 0.2 mission_monitoring_rpi 'ros2 run rover_monitoring mission_monitoring_node_rpi'

# Pane 7 (mid-right): CSV Data Logger
tmux select-pane -t $SESSION_NAME:0.7 -T "CSV_Logger"
tmux send-keys   -t $SESSION_NAME:0.7 "$SRC" C-m
tmux send-keys   -t $SESSION_NAME:0.7 "clear && echo -e '\\e[1;96m>>> [8/8] CSV DATA LOGGER <<<\\e[0m' && sleep 1" C-m
run_logged 0.7 rover_monitoring 'ros2 run rover_monitoring rover_monitoring_node'

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
