#!/usr/bin/env python3
"""
Multi-Domain Rover Monitoring Launch
- Domain 5: Subscribes to rover control topics
- Domain 4: Publishes aggregated monitoring status
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Rover monitoring node - runs in Domain 5
    # Subscribes to all Domain 5 rover topics
    rover_monitoring_node = Node(
        package='pkg_rover_monitoring',
        executable='node_rover_monitoring',
        name='node_rover_monitoring',
        output='screen',
        parameters=[],
        # This node runs in Domain 5 to subscribe to rover topics
        # and publishes RoverStatus also in Domain 5
        environment={'ROS_DOMAIN_ID': '5'}
    )
    
    # Domain bridge: republish /tpc_rover_status from Domain 5 to Domain 4
    # This way base station can subscribe in Domain 4 without seeing Domain 5
    domain_bridge_node = Node(
        package='domain_bridge',
        executable='domain_bridge',
        name='domain_bridge_monitoring',
        output='screen',
        parameters=[{
            'from_domain': 5,
            'to_domain': 4,
            'topics': ['/tpc_rover_status']
        }]
    )
    
    return LaunchDescription([
        rover_monitoring_node,
        domain_bridge_node,
    ])
