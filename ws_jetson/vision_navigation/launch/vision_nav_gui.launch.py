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

from vision_navigation_pkg.config import (
    CameraConfig, LaneDetectionConfig, ControlConfig
)


def generate_launch_description():
    # ==================== ROS2 Domain Configuration ====================
    set_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '2')
    
    # ==================== Config File Path ====================
    config_file = PathJoinSubstitution([
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
    
    # Control parameters
    k_e1 = DeclareLaunchArgument('k_e1', default_value=str(ControlConfig.K_E1))
    k_e2 = DeclareLaunchArgument('k_e2', default_value=str(ControlConfig.K_E2))
    k_p = DeclareLaunchArgument('k_p', default_value=str(ControlConfig.K_P))
    k_i = DeclareLaunchArgument('k_i', default_value=str(ControlConfig.K_I))
    k_d = DeclareLaunchArgument('k_d', default_value=str(ControlConfig.K_D))
    ema_alpha = DeclareLaunchArgument('ema_alpha', default_value=str(ControlConfig.EMA_ALPHA))
    steer_max_deg = DeclareLaunchArgument('steer_max_deg', default_value=str(ControlConfig.STEER_MAX_DEGREES))
    steer_when_lost = DeclareLaunchArgument('steer_when_lost', default_value=str(ControlConfig.STEER_WHEN_LOST))
    
    # ==================== Nodes with GUI ENABLED ====================
    
    camera_stream_node = Node(
        package='vision_navigation',
        executable='camera_stream',
        name='camera_stream',
        output='screen',
        emulate_tty=True,
        parameters=[config_file],
    )
    
    lane_detection_node = Node(
        package='vision_navigation',
        executable='lane_detection',
        name='lane_detection',
        output='screen',
        emulate_tty=True,
        parameters=[config_file],
    )
    
    steering_control_node = Node(
        package='vision_navigation',
        executable='steering_control',
        name='steering_control',
        output='screen',
        emulate_tty=True,
        parameters=[config_file],
    )
    
    # ==================== Launch Sequence ====================
    
    return LaunchDescription([
        set_domain_id,
        
        # Declare arguments
        camera_width, camera_height, camera_fps,
        enable_depth, video_path, loop_video, json_config,
        k_e1, k_e2, k_p, k_i, k_d, ema_alpha, steer_max_deg, steer_when_lost,
        
        # Startup messages
        LogInfo(msg='Vision Navigation System starting in GUI MODE...'),
        LogInfo(msg='[GUI] Camera preview and lane visualization ENABLED'),
        
        # Start camera immediately
        camera_stream_node,
        LogInfo(msg='[INFO] Camera node started with GUI preview, initializing hardware (2s)...'),
        
        # Start lane detection after 2s
        TimerAction(period=2.0, actions=[
            LogInfo(msg='[INFO] Camera ready, starting lane detection with visualization...'),
            lane_detection_node,
        ]),
        
        # Start steering control after 3s
        TimerAction(period=3.0, actions=[
            LogInfo(msg='[INFO] Lane detection ready, starting control node...'),
            steering_control_node,
            LogInfo(msg='[INFO] Vision Navigation System (GUI MODE) fully initialized!'),
        ]),
    ])
