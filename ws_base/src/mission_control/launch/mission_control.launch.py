import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get package directory
    mission_control_dir = get_package_share_directory('mission_control')
    
    # Path to config file
    config_file = os.path.join(mission_control_dir, 'config', 'params.yaml')
    
    # Declare launch arguments that can override config file values
    rover_spd_arg = DeclareLaunchArgument(
        'rover_spd',
        default_value='',
        description='Speed of the rover (0-100%). Overrides config file if provided.'
    )
    des_lat_arg = DeclareLaunchArgument(
        'des_lat',
        default_value='',
        description='Destination Latitude. Overrides config file if provided.'
    )
    des_long_arg = DeclareLaunchArgument(
        'des_long',
        default_value='',
        description='Destination Longitude. Overrides config file if provided.'
    )

    # Build parameters list - load from YAML file
    # Command line args will override if provided
    def build_parameters():
        params = [config_file]
        
        # Add override parameters if provided (non-empty)
        overrides = {}
        rover_spd_val = LaunchConfiguration('rover_spd')
        des_lat_val = LaunchConfiguration('des_lat')
        des_long_val = LaunchConfiguration('des_long')
        
        return params

    return LaunchDescription([
        rover_spd_arg,
        des_lat_arg,
        des_long_arg,

        Node(
            package='mission_control',
            executable='mission_command_node',
            name='mission_command_node',
            parameters=[config_file],  # Load default parameters from YAML
            output='screen'
        ),
        
        Node(
            package='mission_control',
            executable='mission_monitoring_node_pc',
            name='mission_monitoring_node_pc',
            output='screen'
        )
    ])

#################################################################