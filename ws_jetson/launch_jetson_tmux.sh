#!/bin/bash
# ws_jetson Tmux Launch Script - Multi-Domain Architecture
# Domain 6: Vision processing (camera_stream, lane_detection)
# Domain 5: Control output — no separate bridge process
#
# This script launches all ws_jetson nodes in a tmux session with 5 panes:
#   [0] Camera Stream (Domain 6)
#   [1] Lane Detection (Domain 6)
#   [2] Rover Kinematic Control — dual-context (D6 sub tpc_rover_nav_lane | D5 pub tpc_rover_ctrl_cmd)
#   [3] Rover Local Monitoring — Domain 4 CSV logger (subscribes tpc_telemetry_relay)
#   [4] Camera Recorder (Domain 6) — raw video of tpc_rover_d415_rgb for offline debugging
#
# Architecture:
#   Domain 6: camera_stream → lane_detection → tpc_rover_nav_lane
#   rover_kinematic_control (single process, two contexts):
#     ctx_d6: subscribes tpc_rover_nav_lane, runs PID
#     ctx_d5: publishes tpc_rover_ctrl_cmd directly to chassis_controller (RPi)
#   No separate bridge node — dual-context handled inside rover_kinematic_control.

SESSION_NAME="jetson_vision"

# ── One run directory, shared by every logging node ───────────────────────────
# Each logging node is a separate process, so if they each allocated their own
# run number and timestamp a single launch would scatter output across several
# run_NNN_* directories (and a stray runs/logs/). Allocate it once here and
# export it; every node prefers $ROVER_RUN_DIR over computing its own.
#
# Deliberately NOT created here — the nodes create it on their first actual
# write, so a launch that logs nothing leaves no empty directory behind.
RUNS_DIR="$HOME/almondmatcha/ws_jetson/runs"
_last=$(ls -d "$RUNS_DIR"/run_[0-9][0-9][0-9]_* 2>/dev/null \
        | sed -n 's#.*/run_\([0-9][0-9][0-9]\)_.*#\1#p' | sort -n | tail -1)
_next=$(( 10#${_last:-000} + 1 ))
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


# Kill existing session if exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Create new tmux session
tmux new-session -d -s $SESSION_NAME

# Create layout: 1 left pane (camera), 4 right panes stacked (detection, control, monitoring, recorder)
tmux split-window -h  # Split into left and right
tmux select-pane -t 1
tmux split-window -v  # Split right into top and bottom
tmux select-pane -t 2
tmux split-window -v  # Split right-bottom into middle and bottom
tmux select-pane -t 3
tmux split-window -v  # Split bottom-right again for the recorder

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
tmux send-keys -t $SESSION_NAME:0.0 "export ROVER_RUN_DIR='$ROVER_RUN_DIR'" C-m
tmux send-keys -t $SESSION_NAME:0.0 "export ROS_DOMAIN_ID=6" C-m
tmux send-keys -t $SESSION_NAME:0.0 "clear && echo -e '\\e[1;36m>>> [Domain 6] CAMERA STREAM <<<\\e[0m' && sleep 1" C-m
run_logged 0.0 camera_stream 'ros2 run vision_navigation camera_stream_node --ros-args --params-file src/vision_navigation/config/vision_nav_headless.yaml'

# Pane 1 (top-right): Lane Detection - Domain 6
tmux select-pane -t 1 -T "Lane_Detect_D6"
tmux send-keys -t $SESSION_NAME:0.1 "source /opt/ros/humble/setup.bash && cd ~/almondmatcha/ws_jetson && source ~/almondmatcha/common_ifaces/install/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.1 "export ROVER_RUN_DIR='$ROVER_RUN_DIR'" C-m
tmux send-keys -t $SESSION_NAME:0.1 "export ROS_DOMAIN_ID=6" C-m
tmux send-keys -t $SESSION_NAME:0.1 "clear && echo -e '\\e[1;32m>>> [Domain 6] LANE DETECTION <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.1 "echo 'Waiting for camera initialization (3s)...' && sleep 3" C-m
run_logged 0.1 lane_detection 'ros2 run vision_navigation lane_detection_node --ros-args --params-file src/vision_navigation/config/vision_nav_headless.yaml'

# Pane 2 (bottom-right): Rover Kinematic Control — dual-context (D6 sub | D5 pub)
tmux select-pane -t 2 -T "Kinematic_Ctrl_D6+D5"
tmux send-keys -t $SESSION_NAME:0.2 "source /opt/ros/humble/setup.bash && cd ~/almondmatcha/ws_jetson && source ~/almondmatcha/common_ifaces/install/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.2 "export ROVER_RUN_DIR='$ROVER_RUN_DIR'" C-m
tmux send-keys -t $SESSION_NAME:0.2 "clear && echo -e '\\e[1;33m>>> [D6 sub + D5 pub] ROVER KINEMATIC CONTROL <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.2 "echo 'Waiting for lane detection (4s)...' && sleep 4" C-m
run_logged 0.2 rover_kinematic_control 'ros2 run vision_navigation rover_kinematic_control --ros-args --params-file src/vision_navigation/config/rover_kinematic_control_params.yaml'

# Pane 3 (bottom-right): Rover Local Monitoring — Domain 4 CSV logger
tmux select-pane -t 3 -T "Local_Monitor_D4"
tmux send-keys -t $SESSION_NAME:0.3 "source /opt/ros/humble/setup.bash && cd ~/almondmatcha/ws_jetson && source ~/almondmatcha/common_ifaces/install/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.3 "export ROVER_RUN_DIR='$ROVER_RUN_DIR'" C-m
tmux send-keys -t $SESSION_NAME:0.3 "export ROS_DOMAIN_ID=4" C-m
tmux send-keys -t $SESSION_NAME:0.3 "clear && echo -e '\\e[1;35m>>> [Domain 4] ROVER LOCAL MONITORING (CSV) <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.3 "echo 'Waiting for telemetry relay (5s)...' && sleep 5" C-m
run_logged 0.3 rover_local_monitoring 'ros2 run rover_monitoring rover_local_monitoring_node'

# Pane 4 (bottom-right): Camera Recorder — Domain 6, raw video for offline debugging
tmux select-pane -t 4 -T "Camera_Recorder_D6"
tmux send-keys -t $SESSION_NAME:0.4 "source /opt/ros/humble/setup.bash && cd ~/almondmatcha/ws_jetson && source ~/almondmatcha/common_ifaces/install/setup.bash && source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.4 "export ROVER_RUN_DIR='$ROVER_RUN_DIR'" C-m
tmux send-keys -t $SESSION_NAME:0.4 "export ROS_DOMAIN_ID=6" C-m
tmux send-keys -t $SESSION_NAME:0.4 "clear && echo -e '\\e[1;96m>>> [Domain 6] CAMERA RECORDER (raw video, offline debug) <<<\\e[0m'" C-m
tmux send-keys -t $SESSION_NAME:0.4 "echo 'Waiting for camera initialization (3s)...' && sleep 3" C-m
run_logged 0.4 camera_recorder 'ros2 run vision_navigation camera_recorder_node'

# Focus on camera pane and attach
tmux select-pane -t 0
[[ "${SKIP_ATTACH:-0}" != "1" ]] && tmux attach-session -t $SESSION_NAME

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

