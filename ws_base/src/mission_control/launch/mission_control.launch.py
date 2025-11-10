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
            executable='mission_monitoring_node',
            name='mission_monitoring_node',
            output='screen'
        )
    ])

#################################################################
#  import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, Shutdown
# from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration

# def generate_launch_description():
#     # Declare launch arguments with string values (ROS 2 arguments are passed as strings)
#     rover_spd_arg = DeclareLaunchArgument('rover_spd', default_value='0', description='Speed of the rover')
#     des_lat_arg = DeclareLaunchArgument('des_lat', default_value='0.000000', description='Destination Latitude')
#     des_long_arg = DeclareLaunchArgument('des_long', default_value='0.00000', description='Destination Longitude')

#     return LaunchDescription([
#         rover_spd_arg,
#         des_lat_arg,
#         des_long_arg,

#         Node(
#             package='package_base_command',
#             executable='node_command',
#             name='node_command',
#             parameters=[{
#                 'rover_spd': LaunchConfiguration('rover_spd'),
#                 'des_lat': LaunchConfiguration('des_lat'),
#                 'des_long': LaunchConfiguration('des_long')
#             }],
#             on_exit=Shutdown(),         # Ensure proper shutdown when stopping
#             emulate_tty=True,           # Fix issues with logs not appearing properly
#             output='screen',            # Print logs to terminal
#             shutdown_timeout=20.0   
#         )
#     ])


################### Read config from yaml file ###################
# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     config = os.path.join(
#         get_package_share_directory('package_base_command'),  # Get package directory
#         'config',
#         'params.yaml'  # Path to params.yaml inside the config folder
#     )

#     return LaunchDescription([
#         Node(
#             package='package_base_command',
#             executable='node_command',
#             name='node_command',
#             parameters=[config]  # Load parameters from the file
#         )
#     ])
#################################################################