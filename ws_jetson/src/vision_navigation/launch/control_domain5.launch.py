"""
Vision Navigation System — Kinematic Control (dual-context)

Launches rover_kinematic_control on the Jetson.
This single process holds TWO rclpy contexts (one per domain), replacing
the former domain_bridge_jetson separate node.

Domain Architecture:
    Domain 6 context: subscribes /tpc_rover_nav_lane (vision data)
    Domain 5 context: publishes /tpc_rover_ctrl_cmd (chassis command)

    Vision pipeline (separate launch file — vision_domain6.launch.py):
        - camera_stream  →  lane_detection  →  tpc_rover_nav_lane (D6)

Benefits:
    - Only 1 ws_jetson node visible on Domain 5 (no discovery overhead)
    - No IPC relay hop — contexts share the same Python process memory
    - Encoder feedback from D5 is trivially added (D5 context already present)
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

from vision_navigation.config import ControlConfig


def generate_launch_description():
    # ==================== ROS2 Domain Configuration ====================
    # Domain 5: Control loop (STM32, ws_rpi, ws_base, THIS node)
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '5')
    
    # ==================== Config File Paths ====================
    steering_config = PathJoinSubstitution([
        FindPackageShare('vision_navigation'),
        'config',
        'rover_kinematic_control_params.yaml'
    ])
    
    # ==================== Launch Arguments ====================
    
    k_lat = DeclareLaunchArgument('k_lat', default_value=str(ControlConfig.K_LAT),
                                   description='Gain on lateral offset b (deg/metre)')
    k_head = DeclareLaunchArgument('k_head', default_value=str(ControlConfig.K_HEAD),
                                    description='Gain on heading error theta (deg/deg)')
    wheelbase_m = DeclareLaunchArgument('wheelbase_m', default_value=str(ControlConfig.WHEELBASE_M),
                                         description='Front-to-rear axle distance, metres')
    ema_alpha = DeclareLaunchArgument('ema_alpha', default_value=str(ControlConfig.EMA_ALPHA),
                                       description='EMA smoothing factor')
    steer_max_deg = DeclareLaunchArgument('steer_max_deg', default_value=str(ControlConfig.STEER_MAX_DEGREES),
                                           description='Maximum steering angle')
    steer_when_lost = DeclareLaunchArgument('steer_when_lost', default_value=str(ControlConfig.STEER_WHEN_LOST),
                                             description='Steering when lane lost')
    
    # ==================== Control Interface Node (Domain 5) ====================
    
    # NOTE: This node internally creates TWO rclpy.Context objects:
    # - ctx_d6 (Domain 6): subscribes to tpc_rover_nav_lane  [D5InterfaceNode]
    # - ctx_d5 (Domain 5): publishes tpc_rover_ctrl_cmd       [RoverKinematicControlNode]
    # Both contexts run in the same OS process — no IPC relay, no bridge node.
    
    rover_kinematic_control_node = Node(
        package='vision_navigation',
        executable='rover_kinematic_control',
        name='rover_kinematic_control',
        output='screen',
        emulate_tty=True,
        parameters=[steering_config],
    )
    
    # ==================== Launch Sequence ====================
    
    return LaunchDescription([
        set_domain_id,
        
        # Declare arguments
        k_lat, k_head, wheelbase_m, ema_alpha, steer_max_deg, steer_when_lost,
        
        # Startup messages
        LogInfo(msg='========================================'),
        LogInfo(msg='Vision Navigation - Domain 5 (Control Output)'),
        LogInfo(msg='========================================'),
        LogInfo(msg='Domain: 5 (Rover control loop)'),
        LogInfo(msg='Node: rover_kinematic_control (dual-context)'),
        LogInfo(msg='Input: tpc_rover_nav_lane from Domain 6 (localhost)'),
        LogInfo(msg='Output: tpc_rover_ctrl_cmd to Domain 5 (rover control)'),
        LogInfo(msg='========================================'),
        LogInfo(msg='[PREREQUISITE] Ensure vision_domain6.launch.py is running!'),
        LogInfo(msg='========================================'),
        
        # Start dual-context control node
        rover_kinematic_control_node,
        
        LogInfo(msg='[Domain 5] Control interface node started'),
        LogInfo(msg='[Domain 5] Subscribing to Domain 6 vision data (localhost)'),
        LogInfo(msg='[Domain 5] Publishing control commands to Domain 5 (rover network)'),
        LogInfo(msg='[Domain 5] Control interface fully initialized!'),
    ])
