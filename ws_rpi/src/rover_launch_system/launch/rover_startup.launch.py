from launch import LaunchDescription
from launch.actions import GroupAction, SetEnvironmentVariable
from launch_ros.actions import Node
import datetime

# cd ~/almondmatcha/ws_rpi/
# source install/setup.bash
# ros2 launch rover_launch_system rover_startup.launch.py

set_custom_log_dir = SetEnvironmentVariable(
    name='ROS_LOG_DIR', 
    value=f'/home/curry/almondmatcha/runs/ros_logs/{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}'
)

def generate_launch_description():

    # 1. Group for nodes running on ROS_DOMAIN_ID=2 (GNSS & Navigation)
    domain_2_group = GroupAction(
        actions=[
            # Manual Command Equivalent: export ROS_DOMAIN_ID=2
            SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='2'),

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

            # Command: ros2 run pkg_chassis_control node_chassis_controller
            Node(
                package='pkg_chassis_control',
                executable='node_chassis_controller',
                name='chassis_controller_node',
                output='log',
                emulate_tty=True
            ),
        ]
    )

    # 2. Group for nodes running on ROS_DOMAIN_ID=5 (Chassis IMU & Sensors)
    domain_5_group = GroupAction(
        actions=[
            # Manual Command Equivalent: export ROS_DOMAIN_ID=5
            SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='5'),

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

    # Return the full launch description, executing all groups and nodes simultaneously
    return LaunchDescription([
        set_custom_log_dir,
        domain_2_group,
        domain_5_group,
    ])