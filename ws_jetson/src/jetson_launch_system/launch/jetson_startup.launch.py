from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable # Used for setting environment variables
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import datetime

def generate_launch_description():
    # --- 1. DECLARE LAUNCH ARGUMENTS ---
    output_arg = DeclareLaunchArgument(
        'node_output',
        default_value=TextSubstitution(text='log'), 
        description='Console output destination for all nodes (screen or log)'
    )
    
    # --- 2. ENVIRONMENT VARIABLE SETUP ---
    
    # 2a. Set ROS_DOMAIN_ID = 2
    set_domain_id = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value='2'
    )
    
    # 2b. LOG DIRECTORY SETUP
    set_custom_log_dir = SetEnvironmentVariable(
        name='ROS_LOG_DIR', 
        value=f'/home/yupi/Almond/ros2-jetson-ws/run_logs/{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}'
    )

    # Use the declared argument as a LaunchConfiguration
    node_output = LaunchConfiguration('node_output')

    # --- 3. NODE DEFINITIONS (Unchanged) ---
    package_name = 'pkg_imagproc'

    cam_stream_node = Node(
        package=package_name,
        executable='node_cam_stream',
        name='cam_stream_node',
        output=node_output, 
        emulate_tty=True
    )

    nav_process_node = Node(
        package=package_name,
        executable='node_nav_process',
        name='nav_process_node',
        output=node_output, 
        emulate_tty=True
    )

    roctl_node = Node(
        package=package_name,
        executable='node_roctl',
        name='roctl_node',
        output=node_output, 
        emulate_tty=True
    )

    # --- 4. RETURN LAUNCH DESCRIPTION ---
    return LaunchDescription([
        # Add the ROS_DOMAIN_ID setup here
        set_domain_id,
        output_arg,
        set_custom_log_dir,
        cam_stream_node,
        nav_process_node,
        roctl_node,
    ])