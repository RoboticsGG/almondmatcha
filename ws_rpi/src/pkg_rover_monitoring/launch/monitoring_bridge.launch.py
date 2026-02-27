#!/usr/bin/env python3
"""
Dual Monitor Node Launch - Telemetry Relay Architecture
- Mission Monitoring Node (RPi): Aggregates all Domain 5 telemetry and publishes unified relay
- Published topic: /tpc_telemetry_relay (Domain 5)
- Consumed by: mission_monitoring_node_pc on base station
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Mission monitoring node (RPi) - runs in Domain 5
    # Subscribes to all Domain 5 rover topics and publishes aggregated telemetry relay
    mission_monitoring_node_rpi = Node(
        package='pkg_rover_monitoring',
        executable='mission_monitoring_node_rpi',
        name='mission_monitoring_node_rpi',
        output='screen',
        parameters=[],
        # This node runs in Domain 5 to subscribe to all rover topics
        # and publishes TelemetryRelay to /tpc_telemetry_relay
        environment={'ROS_DOMAIN_ID': '5'}
    )
    
    # CSV data logger (optional) - for local data recording
    csv_logger_node = Node(
        package='pkg_rover_monitoring',
        executable='rover_monitoring_node',
        name='rover_monitoring_node',
        output='screen',
        parameters=[],
        environment={'ROS_DOMAIN_ID': '5'}
    )
    
    return LaunchDescription([
        mission_monitoring_node_rpi,
        csv_logger_node,
    ])
