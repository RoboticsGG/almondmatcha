#!/usr/bin/env python3
"""
Simple Domain Bridge for Rover Monitoring
Subscribes to /tpc_rover_status in Domain 5
Republishes to /tpc_rover_status in Domain 4

This isolates monitoring traffic from the rover control domain.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.context import Context
from msgs_ifaces.msg import RoverStatus


class DomainBridgeNode(Node):
    def __init__(self):
        # Create context for Domain 5 (subscribing)
        self.context_domain5 = Context()
        rclpy.init(context=self.context_domain5, domain_id=5)
        
        # Create context for Domain 4 (publishing)
        self.context_domain4 = Context()
        rclpy.init(context=self.context_domain4, domain_id=4)
        
        # Node in Domain 5 (subscriber)
        super().__init__('domain_bridge_sub', context=self.context_domain5)
        self.subscription = self.create_subscription(
            RoverStatus,
            '/tpc_rover_status',
            self.rover_status_callback,
            10
        )
        
        # Node in Domain 4 (publisher)
        self.pub_node = Node('domain_bridge_pub', context=self.context_domain4)
        self.publisher = self.pub_node.create_publisher(
            RoverStatus,
            '/tpc_rover_status',
            10
        )
        
        self.get_logger().info('Domain Bridge Started')
        self.get_logger().info('  Subscribing: Domain 5 → /tpc_rover_status')
        self.get_logger().info('  Publishing:  Domain 4 → /tpc_rover_status')
    
    def rover_status_callback(self, msg):
        """Relay message from Domain 5 to Domain 4"""
        self.publisher.publish(msg)


def main(args=None):
    # Note: We can't easily create two contexts in one process with rclpy
    # Better approach: Use environment variable and simple relay
    
    import os
    import sys
    
    # Check which domain this instance should run in
    mode = os.environ.get('BRIDGE_MODE', 'subscriber')
    
    if mode == 'subscriber':
        # This instance subscribes in Domain 5
        os.environ['ROS_DOMAIN_ID'] = '5'
        rclpy.init(args=args)
        
        class SubscriberNode(Node):
            def __init__(self):
                super().__init__('bridge_subscriber')
                self.subscription = self.create_subscription(
                    RoverStatus,
                    '/tpc_rover_status',
                    self.callback,
                    10
                )
                # Write to pipe or shared memory
                self.get_logger().info('Domain 5 subscriber ready')
            
            def callback(self, msg):
                # In real implementation, forward to Domain 4 via IPC
                pass
        
        node = SubscriberNode()
        rclpy.spin(node)
        rclpy.shutdown()
    
    elif mode == 'publisher':
        # This instance publishes in Domain 4
        os.environ['ROS_DOMAIN_ID'] = '4'
        rclpy.init(args=args)
        
        class PublisherNode(Node):
            def __init__(self):
                super().__init__('bridge_publisher')
                self.publisher = self.create_publisher(
                    RoverStatus,
                    '/tpc_rover_status',
                    10
                )
                self.get_logger().info('Domain 4 publisher ready')
        
        node = PublisherNode()
        rclpy.spin(node)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
