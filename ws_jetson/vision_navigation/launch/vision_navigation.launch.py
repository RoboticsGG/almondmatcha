"""
Vision Navigation System - ROS2 Launch File

Launches complete vision navigation system with three coordinated nodes in sequence:
1. Camera Stream Node - Acquires RGB frames from D415 camera or video file (starts first)
2. Lane Detection Node - Detects lane markers and computes steering parameters (waits for camera)
3. Steering Control Node - Computes steering command via PID controller (waits for lane detection)

Camera Initialization: 2-second delay to allow D415 camera hardware to fully initialize
before streaming frames to downstream processing nodes.

Configuration: Automatically reads defaults from config.py. Override with command-line arguments.

Usage:
    # Launch with default parameters from config.py
    ros2 launch vision_navigation vision_navigation.launch.py

    # Override specific parameters
    ros2 launch vision_navigation vision_navigation.launch.py \
        camera_width:=640 k_p:=5.0

Author: Vision Navigation System
Date: November 4, 2025
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from vision_navigation_pkg.config import (
    CameraConfig, LaneDetectionConfig, ControlConfig, SystemConfig
)


def generate_launch_description():
    # ==================== Launch Arguments (Auto-synced from config.py) ====================
    
    # Camera stream node parameters
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
    
    camera_preview = DeclareLaunchArgument(
        'camera_preview',
        default_value=str(CameraConfig.OPEN_CAMERA_DISPLAY).lower(),
        description='Display camera preview window (D415 mode only)'
    )
    
    enable_depth = DeclareLaunchArgument(
        'enable_depth',
        default_value=str(CameraConfig.ENABLE_DEPTH_STREAM).lower(),
        description='Enable depth frame streaming (D415 mode only)'
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
    
    # Lane detection node parameters
    lane_visualization = DeclareLaunchArgument(
        'lane_visualization',
        default_value=str(LaneDetectionConfig.SHOW_WINDOW).lower(),
        description='Display lane detection visualization window'
    )
    
    # Steering control node parameters
    k_e1 = DeclareLaunchArgument(
        'k_e1',
        default_value=str(ControlConfig.K_E1),
        description='Weight on heading error (theta) in combined error'
    )
    
    k_e2 = DeclareLaunchArgument(
        'k_e2',
        default_value=str(ControlConfig.K_E2),
        description='Weight on lateral offset (b) in combined error'
    )
    
    k_p = DeclareLaunchArgument(
        'k_p',
        default_value=str(ControlConfig.K_P),
        description='PID proportional gain'
    )
    
    k_i = DeclareLaunchArgument(
        'k_i',
        default_value=str(ControlConfig.K_I),
        description='PID integral gain'
    )
    
    k_d = DeclareLaunchArgument(
        'k_d',
        default_value=str(ControlConfig.K_D),
        description='PID derivative gain'
    )
    
    ema_alpha = DeclareLaunchArgument(
        'ema_alpha',
        default_value=str(ControlConfig.EMA_ALPHA),
        description='Exponential moving average smoothing factor (0-1)'
    )
    
    steer_max_deg = DeclareLaunchArgument(
        'steer_max_deg',
        default_value=str(ControlConfig.STEER_MAX_DEGREES),
        description='Maximum steering angle saturation in degrees'
    )
    
    steer_when_lost = DeclareLaunchArgument(
        'steer_when_lost',
        default_value=str(ControlConfig.STEER_WHEN_LOST),
        description='Default steering command when lane not detected'
    )
    
    # ==================== Nodes ====================
    
    # Camera Stream Node
    camera_stream_node = Node(
        package='vision_navigation',
        executable='camera_stream',
        name='camera_stream',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'width': LaunchConfiguration('camera_width'),
            'height': LaunchConfiguration('camera_height'),
            'fps': LaunchConfiguration('camera_fps'),
            'open_cam': LaunchConfiguration('camera_preview'),
            'enable_depth': LaunchConfiguration('enable_depth'),
            'video_path': LaunchConfiguration('video_path'),
            'loop_video': LaunchConfiguration('loop_video'),
            'json_config': LaunchConfiguration('json_config'),
        }],
    )
    
    # Lane Detection Node
    lane_detection_node = Node(
        package='vision_navigation',
        executable='lane_detection',
        name='lane_detection',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'show_window': LaunchConfiguration('lane_visualization'),
        }],
    )
    
    # Steering Control Node
    steering_control_node = Node(
        package='vision_navigation',
        executable='steering_control',
        name='steering_control',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'k_e1': LaunchConfiguration('k_e1'),
            'k_e2': LaunchConfiguration('k_e2'),
            'k_p': LaunchConfiguration('k_p'),
            'k_i': LaunchConfiguration('k_i'),
            'k_d': LaunchConfiguration('k_d'),
            'ema_alpha': LaunchConfiguration('ema_alpha'),
            'steer_max_deg': LaunchConfiguration('steer_max_deg'),
            'steer_when_lost': LaunchConfiguration('steer_when_lost'),
        }],
    )
    
    # ==================== Launch Info ====================
    
    log_info = LogInfo(msg='Vision Navigation System starting...')
    
    log_camera_init = LogInfo(msg='[INFO] Camera node started, initializing hardware (2s)...')
    log_lane_start = LogInfo(msg='[INFO] Camera ready, starting lane detection node...')
    log_control_start = LogInfo(msg='[INFO] Lane detection ready, starting control node...')
    log_system_ready = LogInfo(msg='[INFO] Vision Navigation System fully initialized and ready!')
    
    # ==================== Return Launch Description ====================
    
    return LaunchDescription([
        # Declare arguments
        camera_width,
        camera_height,
        camera_fps,
        camera_preview,
        enable_depth,
        video_path,
        loop_video,
        json_config,
        lane_visualization,
        k_e1,
        k_e2,
        k_p,
        k_i,
        k_d,
        ema_alpha,
        steer_max_deg,
        steer_when_lost,
        
        # System startup
        log_info,
        
        # Stage 1: Start camera (priority 100, starts immediately)
        camera_stream_node,
        log_camera_init,
        
        # Stage 2: Start lane detection after 2.0 seconds (camera initialization delay)
        TimerAction(period=2.0, actions=[
            log_lane_start,
            lane_detection_node,
        ]),
        
        # Stage 3: Start steering control after 3.0 seconds (camera + lane detection ready)
        TimerAction(period=3.0, actions=[
            log_control_start,
            steering_control_node,
            log_system_ready,
        ]),
    ])
