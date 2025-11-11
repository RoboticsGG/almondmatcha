"""
Vision Navigation System - Domain 6 (Vision Processing)

Launches vision processing nodes on ROS Domain 6 (isolated from control loop).
These nodes handle all heavy computation: camera streaming, image processing,
lane detection, and publish results internally on Domain 6.

Domain Architecture:
    Domain 6: Vision processing (THIS LAUNCH FILE)
        - Camera streaming (high bandwidth RGB/depth)
        - Lane detection (computationally intensive)
        - Internal communication only

    Domain 5: Control interface (separate launch file)
        - Single lightweight node that reads Domain 6 results
        - Publishes control commands to Domain 5 for rover control

Benefits:
    - Reduces STM32 discovery burden (fewer participants on Domain 5)
    - Isolates high-bandwidth camera data from control loop
    - Better scalability for adding more vision/AI nodes
    - Control loop remains time-critical and stable

Usage:
    # Terminal 1: Start vision processing on Domain 6
    ros2 launch vision_navigation vision_domain6.launch.py

    # Terminal 2: Start control output on Domain 5 (separate launch)
    ros2 launch vision_navigation control_domain5.launch.py

Author: Vision Navigation System
Date: November 11, 2025
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare

from vision_navigation_pkg.config import CameraConfig


def generate_launch_description():
    # ==================== ROS2 Domain Configuration ====================
    # Domain 6: Vision processing nodes (isolated from control loop)
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '6')
    
    # ==================== Config File Paths ====================
    system_config = PathJoinSubstitution([
        FindPackageShare('vision_navigation'),
        'config',
        'vision_nav_headless.yaml'
    ])
    
    # ==================== Launch Arguments ====================
    
    camera_width = DeclareLaunchArgument(
        'camera_width',
        default_value=str(CameraConfig.WIDTH),
        description='Camera frame width in pixels'
    )
    
    camera_height = DeclareLaunchArgument(
        'camera_height',
        default_value=str(CameraConfig.HEIGHT),
        description='Camera frame height in pixels'
    )
    
    camera_fps = DeclareLaunchArgument(
        'camera_fps',
        default_value=str(CameraConfig.FPS),
        description='Camera frames per second'
    )
    
    enable_depth = DeclareLaunchArgument(
        'enable_depth',
        default_value=str(CameraConfig.ENABLE_DEPTH_STREAM).lower(),
        description='Enable depth frame streaming'
    )
    
    video_path = DeclareLaunchArgument(
        'video_path',
        default_value=CameraConfig.VIDEO_PATH,
        description='Path to video file (empty = use D415 camera)'
    )
    
    loop_video = DeclareLaunchArgument(
        'loop_video',
        default_value=str(CameraConfig.LOOP_VIDEO).lower(),
        description='Loop video playback when finished'
    )
    
    json_config = DeclareLaunchArgument(
        'json_config',
        default_value=CameraConfig.JSON_CONFIG_PATH,
        description='Path to RealSense advanced mode JSON configuration'
    )
    
    # ==================== Vision Processing Nodes (Domain 6) ====================
    
    camera_stream_node = Node(
        package='vision_navigation',
        executable='camera_stream',
        name='camera_stream',
        output='screen',
        emulate_tty=True,
        parameters=[system_config],
    )
    
    lane_detection_node = Node(
        package='vision_navigation',
        executable='lane_detection',
        name='lane_detection',
        output='screen',
        emulate_tty=True,
        parameters=[system_config],
    )
    
    # ==================== Launch Sequence ====================
    
    return LaunchDescription([
        set_domain_id,
        
        # Declare arguments
        camera_width, camera_height, camera_fps,
        enable_depth, video_path, loop_video, json_config,
        
        # Startup messages
        LogInfo(msg='========================================'),
        LogInfo(msg='Vision Navigation - Domain 6 (Vision Processing)'),
        LogInfo(msg='========================================'),
        LogInfo(msg='Domain: 6 (Isolated vision processing)'),
        LogInfo(msg='Nodes: camera_stream, lane_detection'),
        LogInfo(msg='Output: Internal lane parameters on Domain 6'),
        LogInfo(msg='========================================'),
        
        # Start camera immediately
        camera_stream_node,
        LogInfo(msg='[Domain 6] Camera node started, initializing hardware (2s)...'),
        
        # Start lane detection after 2s
        TimerAction(period=2.0, actions=[
            LogInfo(msg='[Domain 6] Camera ready, starting lane detection...'),
            lane_detection_node,
            LogInfo(msg='[Domain 6] Vision processing nodes fully initialized!'),
            LogInfo(msg='[Domain 6] Publishing lane data on tpc_rover_nav_lane (Domain 6)'),
            LogInfo(msg='[NEXT STEP] Launch control_domain5.launch.py in another terminal'),
        ]),
    ])
