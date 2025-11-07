from launch import LaunchDescription
from launch.actions import GroupAction, SetEnvironmentVariable
from launch_ros.actions import Node
import datetime

# Rover Startup Launch File
# ==========================
# Launches all ws_rpi nodes with proper domain separation:
#   - Domain 5: All rover internal nodes (sensors, control, GNSS)
#   - Domain 2: Bridge node for base station communication (ws_base)
#
# Usage:
#   cd ~/almondmatcha/ws_rpi/
#   source install/setup.bash
#   ros2 launch rover_launch_system rover_startup.launch.py
#
# Architecture:
#   Domain 5 (Rover Internal):
#     - node_gnss_spresense: GNSS positioning
#     - node_gnss_mission_monitor: Mission waypoint tracking
#     - node_chassis_controller: Motor command coordination
#     - node_chassis_imu: IMU data logging
#     - node_chassis_sensors: Encoder/power logging
#     - node_base_bridge (D5 side): Subscribes rover topics for relay
#
#   Domain 2 (Base Communication):
#     - node_base_bridge (D2 side): Publishes to base station (ws_base)
#
# External Systems:
#   - ws_jetson (Domain 5): Vision nodes (camera, lane detection, steering)
#   - ws_base (Domain 2): Base station monitoring and command nodes
#   - STM32 boards (Domain 5): mROS2 chassis controller and sensor nodes

set_custom_log_dir = SetEnvironmentVariable(
    name='ROS_LOG_DIR', 
    value=f'/home/curry/almondmatcha/runs/ros_logs/{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}'
)

def generate_launch_description():

    # Domain 5: All rover internal nodes (sensors, control, navigation)
    # This enables direct communication for sensor fusion without cross-domain overhead
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

            # Bridge Node (Domain 5 side) - subscribes to rover topics
            # Command: ros2 run pkg_base_bridge node_base_bridge
            Node(
                package='pkg_base_bridge',
                executable='node_base_bridge',
                name='base_bridge_d5_node',
                output='screen',
                emulate_tty=True
            ),
        ]
    )

    # Domain 2: Bridge node for base station communication
    # This node relays topics between Domain 5 (rover) and Domain 2 (base station)
    domain_2_group = GroupAction(
        actions=[
            # Manual Command Equivalent: export ROS_DOMAIN_ID=2
            SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='2'),

            # Bridge Node (Domain 2 side) - publishes to base station
            # Command: ros2 run pkg_base_bridge node_base_bridge
            Node(
                package='pkg_base_bridge',
                executable='node_base_bridge',
                name='base_bridge_d2_node',
                output='screen',
                emulate_tty=True
            ),
        ]
    )

    # Return the full launch description with both domain groups
    return LaunchDescription([
        set_custom_log_dir,
        domain_5_group,
        domain_2_group,
    ])