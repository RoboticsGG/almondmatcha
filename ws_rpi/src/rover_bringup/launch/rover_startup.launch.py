from launch import LaunchDescription
from launch.actions import GroupAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import datetime
import os

# Rover Startup Launch File
# ==========================
# Launches all ws_rpi nodes on the tri-domain architecture (D5 + D4).
#
# Usage:
#   cd ~/almondmatcha/ws_rpi/
#   source install/setup.bash
#   ros2 launch rover_bringup rover_startup.launch.py
#
# Domain 5 nodes (ws_rpi — rover control):
#   - gnss_spresense_node:        Standard GPS positioning
#   - gnss_ublox_node:            RTK GNSS centimeter-level positioning
#   - gnss_mission_monitor_node:  Waypoint navigation state machine
#   - chassis_controller_node:    Motor command coordination
#   - chassis_imu_node:           IMU data relay
#   - chassis_sensors_node:       Encoder/power relay
#   - rover_monitoring_node:      Lightweight local RPi CSV logger (D5 only)
#   - mission_monitoring_node_rpi: D5→D4 relay + primary high-res CSV logging
#
# Other domains (separate workspaces):
#   ws_base (D5 + D4):  mission_command_node, mission_monitoring_node_pc
#   ws_jetson (D6+D5+D4): camera_stream_node, lane_detection_node,
#                          rover_kinematic_control, rover_local_monitoring_node
#   STM32 (D5 mROS2):   chassis_controller (chassis), sensors_node (sensors)

set_custom_log_dir = SetEnvironmentVariable(
    name='ROS_LOG_DIR',
    value=f'/home/curry/almondmatcha/runs/ros_logs/{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}'
)

def generate_launch_description():

    chassis_speed_config = os.path.join(
        get_package_share_directory('chassis_control'),
        'config',
        'chassis_speed_control_params.yaml'
    )

    domain_5_group = GroupAction(
        actions=[
            SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='5'),

            # GNSS Navigation Nodes
            Node(
                package='gnss_navigation',
                executable='gnss_spresense_node',
                name='gnss_spresense_node',
                output='log',
                emulate_tty=True
            ),
            Node(
                package='gnss_navigation',
                executable='gnss_ublox_node',
                name='gnss_ublox_node',
                output='log',
                emulate_tty=True
            ),
            Node(
                package='gnss_navigation',
                executable='gnss_mission_monitor_node',
                name='gnss_mission_monitor_node',
                output='log',
                emulate_tty=True
            ),

            # Chassis Control Node
            Node(
                package='chassis_control',
                executable='chassis_controller_node',
                name='chassis_controller_node',
                output='log',
                emulate_tty=True,
                parameters=[chassis_speed_config]
            ),

            # Chassis Sensor Nodes
            Node(
                package='chassis_sensors',
                executable='chassis_imu_node',
                name='chassis_imu_node',
                output='log',
                emulate_tty=True
            ),
            Node(
                package='chassis_sensors',
                executable='chassis_sensors_node',
                name='chassis_sensors_node',
                output='log',
                emulate_tty=True
            ),

            # Monitoring / Logging Nodes
            Node(
                package='rover_monitoring',
                executable='rover_monitoring_node',
                name='rover_monitoring_node',
                output='log',
                emulate_tty=True
            ),
            Node(
                package='rover_monitoring',
                executable='mission_monitoring_node_rpi',
                name='mission_monitoring_node_rpi',
                output='log',
                emulate_tty=True
            ),
        ]
    )

    return LaunchDescription([
        set_custom_log_dir,
        domain_5_group,
    ])