"""
Vision Navigation System - Domain 5 (Control Output)

Launches control interface node on ROS Domain 5 (rover control loop).
This single lightweight node subscribes to vision results from Domain 6
and publishes steering commands to Domain 5 for the rover control system.

Domain Architecture:
    Domain 6: Vision processing (separate launch file)
        - Camera streaming (high bandwidth RGB/depth)
        - Lane detection (computationally intensive)
        - Internal communication only

    Domain 5: Control interface (THIS LAUNCH FILE)
        - Single node that reads Domain 6 results via shared memory
        - Publishes /tpc_rover_fmctl to Domain 5 for rover control
        - Minimal discovery burden on STM32 boards

Multi-Domain Communication:
    Since both Domain 6 and Domain 5 nodes run on the SAME MACHINE (Jetson),
    they can communicate via:
    1. Shared memory topics (ROS2 intra-process communication)
    2. Localhost-only DDS discovery

Benefits:
    - Only 1 ws_jetson node visible to STM32 boards on Domain 5
    - Vision processing isolated from control loop
    - Scalable: add more vision/AI nodes to Domain 6 without affecting Domain 5
    - Control loop remains time-critical and stable

Usage:
    # Terminal 1: Start vision processing on Domain 6 (FIRST)
    ros2 launch vision_navigation vision_domain6.launch.py

    # Terminal 2: Start control output on Domain 5 (SECOND)
    ros2 launch vision_navigation control_domain5.launch.py

Author: Vision Navigation System
Date: November 11, 2025
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare

from vision_navigation_pkg.config import ControlConfig


def generate_launch_description():
    # ==================== ROS2 Domain Configuration ====================
    # Domain 5: Control loop (STM32, ws_rpi, ws_base, THIS node)
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '5')
    
    # ==================== Config File Paths ====================
    steering_config = PathJoinSubstitution([
        FindPackageShare('vision_navigation'),
        'config',
        'steering_control_params.yaml'
    ])
    
    # ==================== Launch Arguments ====================
    
    k_e1 = DeclareLaunchArgument('k_e1', default_value=str(ControlConfig.K_E1),
                                  description='Heading error weight')
    k_e2 = DeclareLaunchArgument('k_e2', default_value=str(ControlConfig.K_E2),
                                  description='Lateral offset weight')
    k_p = DeclareLaunchArgument('k_p', default_value=str(ControlConfig.K_P),
                                 description='Proportional gain')
    k_i = DeclareLaunchArgument('k_i', default_value=str(ControlConfig.K_I),
                                 description='Integral gain')
    k_d = DeclareLaunchArgument('k_d', default_value=str(ControlConfig.K_D),
                                 description='Derivative gain')
    ema_alpha = DeclareLaunchArgument('ema_alpha', default_value=str(ControlConfig.EMA_ALPHA),
                                       description='EMA smoothing factor')
    steer_max_deg = DeclareLaunchArgument('steer_max_deg', default_value=str(ControlConfig.STEER_MAX_DEGREES),
                                           description='Maximum steering angle')
    steer_when_lost = DeclareLaunchArgument('steer_when_lost', default_value=str(ControlConfig.STEER_WHEN_LOST),
                                             description='Steering when lane lost')
    
    # ==================== Control Interface Node (Domain 5) ====================
    
    # NOTE: This node internally creates TWO contexts:
    # - Context for Domain 6: subscribes to tpc_rover_nav_lane
    # - Context for Domain 5: publishes tpc_rover_fmctl
    # Both contexts run in the same process (simple two-process approach)
    
    steering_control_node = Node(
        package='vision_navigation',
        executable='steering_control_domain5',
        name='steering_control_domain5',
        output='screen',
        emulate_tty=True,
        parameters=[steering_config],
    )
    
    # ==================== Launch Sequence ====================
    
    return LaunchDescription([
        set_domain_id,
        
        # Declare arguments
        k_e1, k_e2, k_p, k_i, k_d, ema_alpha, steer_max_deg, steer_when_lost,
        
        # Startup messages
        LogInfo(msg='========================================'),
        LogInfo(msg='Vision Navigation - Domain 5 (Control Output)'),
        LogInfo(msg='========================================'),
        LogInfo(msg='Domain: 5 (Rover control loop)'),
        LogInfo(msg='Node: steering_control_domain5 (dual-context)'),
        LogInfo(msg='Input: tpc_rover_nav_lane from Domain 6 (localhost)'),
        LogInfo(msg='Output: tpc_rover_fmctl to Domain 5 (rover control)'),
        LogInfo(msg='========================================'),
        LogInfo(msg='[PREREQUISITE] Ensure vision_domain6.launch.py is running!'),
        LogInfo(msg='========================================'),
        
        # Start dual-context control node
        steering_control_node,
        
        LogInfo(msg='[Domain 5] Control interface node started'),
        LogInfo(msg='[Domain 5] Subscribing to Domain 6 vision data (localhost)'),
        LogInfo(msg='[Domain 5] Publishing control commands to Domain 5 (rover network)'),
        LogInfo(msg='[Domain 5] Control interface fully initialized!'),
    ])
