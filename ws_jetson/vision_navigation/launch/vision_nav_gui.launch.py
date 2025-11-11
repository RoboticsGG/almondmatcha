"""
Vision Navigation System - GUI Mode Launch File

Launches complete vision navigation system with GUI visualization enabled.
Designed for debugging and testing with a monitor connected to Jetson.

Features:
- Camera preview window (RGB + Depth if enabled)
- Lane detection visualization window
- All three nodes launched in sequence with proper timing

Usage:
    ros2 launch vision_navigation vision_nav_gui.launch.py

    # With custom parameters
    ros2 launch vision_navigation vision_nav_gui.launch.py \
        enable_depth:=true k_p:=5.0

Author: Vision Navigation System
Date: November 4, 2025
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
    # GUI mode shows camera preview and lane detection visualization
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '6')
    
    # ==================== Config File Paths ====================
    system_config = PathJoinSubstitution([
        FindPackageShare('vision_navigation'),
        'config',
        'vision_nav_gui.yaml'
    ])
    
    # ==================== Launch Arguments ====================
    
    # Camera parameters
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
    
    # ==================== Nodes with GUI ENABLED ====================
    
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
    
    # NOTE: Steering control should be launched separately on Domain 5
    # Use control_domain5.launch.py for the control interface
    # This GUI file only launches Domain 6 vision nodes with visualization
    
    # ==================== Launch Sequence ====================
    
    return LaunchDescription([
        set_domain_id,
        
        # Declare arguments
        camera_width, camera_height, camera_fps,
        enable_depth, video_path, loop_video, json_config,
        
        # Startup messages
        LogInfo(msg='========================================'),
        LogInfo(msg='Vision Navigation - Domain 6 (GUI Mode)'),
        LogInfo(msg='========================================'),
        LogInfo(msg='Domain: 6 (Isolated vision processing)'),
        LogInfo(msg='GUI: Camera preview and lane visualization ENABLED'),
        LogInfo(msg='========================================'),
        
        # Start camera immediately
        camera_stream_node,
        LogInfo(msg='[Domain 6] Camera node started with GUI preview, initializing hardware (2s)...'),
        
        # Start lane detection after 2s
        TimerAction(period=2.0, actions=[
            LogInfo(msg='[Domain 6] Camera ready, starting lane detection with visualization...'),
            lane_detection_node,
            LogInfo(msg='[Domain 6] Vision processing nodes (GUI) fully initialized!'),
            LogInfo(msg='[Domain 6] Publishing lane data on tpc_rover_nav_lane'),
            LogInfo(msg='[NEXT] Control interface will be launched on Domain 5 automatically'),
        ]),
    ])
