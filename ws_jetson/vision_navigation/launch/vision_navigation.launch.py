"""
Vision Navigation System - ROS2 Launch File

Launches complete vision navigation system with three coordinated nodes:
1. Camera Stream Node - Acquires RGB frames from D415 camera or video file
2. Lane Detection Node - Detects lane markers and computes steering parameters
3. Steering Control Node - Computes steering command via PID controller

Usage:
    # Launch with default parameters (D415 camera mode)
    ros2 launch vision_navigation vision_navigation.launch.py

    # Launch with video file input
    ros2 launch vision_navigation vision_navigation.launch.py \
        video_path:=/path/to/video.mp4

    # Launch with visualization enabled
    ros2 launch vision_navigation vision_navigation.launch.py \
        camera_preview:=true lane_visualization:=true

Author: Vision Navigation System
Date: November 4, 2025
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ==================== Launch Arguments ====================
    
    # Camera stream node parameters
    camera_width = DeclareLaunchArgument(
        'camera_width',
        default_value='1280',
        description='Camera frame width in pixels'
    )
    
    camera_height = DeclareLaunchArgument(
        'camera_height',
        default_value='720',
        description='Camera frame height in pixels'
    )
    
    camera_fps = DeclareLaunchArgument(
        'camera_fps',
        default_value='30',
        description='Camera frames per second'
    )
    
    camera_preview = DeclareLaunchArgument(
        'camera_preview',
        default_value='false',
        description='Display camera preview window (D415 mode only)'
    )
    
    enable_depth = DeclareLaunchArgument(
        'enable_depth',
        default_value='false',
        description='Enable depth frame streaming (D415 mode only)'
    )
    
    video_path = DeclareLaunchArgument(
        'video_path',
        default_value='',
        description='Path to video file (empty = use D415 camera)'
    )
    
    loop_video = DeclareLaunchArgument(
        'loop_video',
        default_value='true',
        description='Loop video playback when finished'
    )
    
    json_config = DeclareLaunchArgument(
        'json_config',
        default_value='',
        description='Path to RealSense advanced mode JSON configuration'
    )
    
    # Lane detection node parameters
    lane_visualization = DeclareLaunchArgument(
        'lane_visualization',
        default_value='false',
        description='Display lane detection visualization window'
    )
    
    # Steering control node parameters
    k_e1 = DeclareLaunchArgument(
        'k_e1',
        default_value='1.0',
        description='Weight on heading error (theta) in combined error'
    )
    
    k_e2 = DeclareLaunchArgument(
        'k_e2',
        default_value='0.1',
        description='Weight on lateral offset (b) in combined error'
    )
    
    k_p = DeclareLaunchArgument(
        'k_p',
        default_value='4.0',
        description='PID proportional gain'
    )
    
    k_i = DeclareLaunchArgument(
        'k_i',
        default_value='0.0',
        description='PID integral gain'
    )
    
    k_d = DeclareLaunchArgument(
        'k_d',
        default_value='0.0',
        description='PID derivative gain'
    )
    
    ema_alpha = DeclareLaunchArgument(
        'ema_alpha',
        default_value='0.05',
        description='Exponential moving average smoothing factor (0-1)'
    )
    
    steer_max_deg = DeclareLaunchArgument(
        'steer_max_deg',
        default_value='60.0',
        description='Maximum steering angle saturation in degrees'
    )
    
    steer_when_lost = DeclareLaunchArgument(
        'steer_when_lost',
        default_value='0.0',
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
    
    log_info = LogInfo(msg='Vision Navigation System starting with 3 nodes...')
    
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
        
        # Log info
        log_info,
        
        # Start nodes
        camera_stream_node,
        lane_detection_node,
        steering_control_node,
    ])
