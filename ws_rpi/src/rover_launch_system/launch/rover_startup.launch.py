from launch import LaunchDescription
from launch.actions import GroupAction, SetEnvironmentVariable
from launch_ros.actions import Node
import datetime

# Rover Startup Launch File
# ==========================
# Launches all ws_rpi nodes on unified Domain 5 architecture.
# All systems (rover, base station, vision, STM32) communicate directly on Domain 5.
#
# Usage:
#   cd ~/almondmatcha/ws_rpi/
#   source install/setup.bash
#   ros2 launch rover_launch_system rover_startup.launch.py
#
# Domain 5 Unified Architecture:
#   ws_rpi (Raspberry Pi):
#     - node_gnss_spresense: GNSS positioning
#     - node_gnss_mission_monitor: Mission waypoint tracking
#     - node_chassis_controller: Motor command coordination
#     - node_chassis_imu: IMU data logging
#     - node_chassis_sensors: Encoder/power logging
#
#   ws_base (Ground Station - Domain 5):
#     - mission_command_node: Send navigation goals and speed limits
#     - mission_monitoring_node: Display rover telemetry
#
#   ws_jetson (Vision System - Domain 5):
#     - camera_stream_node, lane_detection_node, steering_control_node
#
#   STM32 Boards (mROS2 - Domain 5):
#     - chassis_dynamics: Motor + IMU (192.168.1.2)
#     - sensors_gnss: Encoders + GNSS (192.168.1.6)

set_custom_log_dir = SetEnvironmentVariable(
    name='ROS_LOG_DIR', 
    value=f'/home/curry/almondmatcha/runs/ros_logs/{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}'
)

def generate_launch_description():

    # Domain 5: All rover nodes (sensors, control, navigation)
    # All systems communicate directly on Domain 5 - no bridge needed
    domain_5_group = GroupAction(
        actions=[
            # Manual Command Equivalent: export ROS_DOMAIN_ID=5
            SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='5'),

            # GNSS Navigation Nodes
            # Command: ros2 run pkg_gnss_navigation node_gnss_spresense
            Node(
                package='pkg_gnss_navigation', 
                executable='node_gnss_spresense',
                name='spresense_gnss_node',
                output='log',
                emulate_tty=True
            ),

            # Command: ros2 run pkg_gnss_navigation node_gnss_mission_monitor
            Node(
                package='pkg_gnss_navigation',
                executable='node_gnss_mission_monitor',
                name='gnss_mission_monitor_node',
                output='log',
                emulate_tty=True
            ),

            # Chassis Control Node
            # Command: ros2 run pkg_chassis_control node_chassis_controller
            Node(
                package='pkg_chassis_control',
                executable='node_chassis_controller',
                name='chassis_controller_node',
                output='log',
                emulate_tty=True
            ),

            # Chassis Sensor Nodes (logging and monitoring)
            # Command: ros2 run pkg_chassis_sensors node_chassis_imu
            Node(
                package='pkg_chassis_sensors',
                executable='node_chassis_imu',
                name='chassis_imu_node',
                output='log',
                emulate_tty=True
            ),

            # Command: ros2 run pkg_chassis_sensors node_chassis_sensors
            Node(
                package='pkg_chassis_sensors',
                executable='node_chassis_sensors',
                name='chassis_sensors_node',
                output='log',
                emulate_tty=True
            ),
        ]
    )

    # Return the launch description with Domain 5 unified architecture
    # Note: Bridge node removed - ws_base now operates on Domain 5 for direct communication
    return LaunchDescription([
        set_custom_log_dir,
        domain_5_group,
    ])