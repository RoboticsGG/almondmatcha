from launch import LaunchDescription
from launch.actions import GroupAction, SetEnvironmentVariable
from launch_ros.actions import Node
import datetime

# cd Almond/ros2-rover-ws/
# source install/setup.bash
# ros2 launch rover_launch_system rover_startup.launch.py

set_custom_log_dir = SetEnvironmentVariable(
    name='ROS_LOG_DIR', 
    value=f'/home/curry/Almond/ros2-rover-ws/run_logs/{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}'
)

def generate_launch_description():

    # 1. Group for nodes running on ROS_DOMAIN_ID=2 (Pose Processing & Rocon2)
    domain_2_group = GroupAction(
        actions=[
            # Manual Command Equivalent: export ROS_DOMAIN_ID=2
            SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='2'),

            # Command: ros2 run pkg_poseproc node_gnss
            Node(
                package='pkg_poseproc', 
                executable='node_gnss',
                name='gnss_node',
                output='log',
                emulate_tty=True
            ),

            # Command: ros2 run pkg_poseproc node_poseproc
            Node(
                package='pkg_poseproc',
                executable='node_poseproc',
                name='poseproc_node',
                output='log',
                emulate_tty=True
            ),

            # Command: ros2 run pkg_rocon node_rocon2
            Node(
                package='pkg_rocon',
                executable='node_rocon2',
                name='rocon2_node',
                output='log',
                emulate_tty=True
            ),
        ]
    )

    # 2. Group for nodes running on ROS_DOMAIN_ID=5 (IMU Data & DBridge)
    domain_5_group = GroupAction(
        actions=[
            # Manual Command Equivalent: export ROS_DOMAIN_ID=5
            SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='5'),

            # Command: ros2 run pkg_sensdata node_imudata
            Node(
                package='pkg_sensdata',
                executable='node_imudata',
                name='imudata_node',
                output='log',
                emulate_tty=True
            ),

            # Command: ros2 run pkg_rocon node_dbridge
            Node(
                package='pkg_rocon',
                executable='node_dbridge',
                name='dbridge_node',
                output='log',
                emulate_tty=True
            ),
        ]
    )

    # 3. Group for node running on ROS_DOMAIN_ID=6 (Sensor Data)
    domain_6_group = GroupAction(
        actions=[
            # Manual Command Equivalent: export ROS_DOMAIN_ID=6
            SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='6'),

            # Command: ros2 run pkg_sensdata node_sensdata
            Node(
                package='pkg_sensdata',
                executable='node_sensdata',
                name='sensdata_node',
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
        domain_6_group,
    ])