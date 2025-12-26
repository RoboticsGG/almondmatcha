#!/usr/bin/env python3
"""
Simple Domain Relay for ROS2
Relays /tpc_rover_status from Domain 5 to Domain 4

This is a lightweight, reusable domain bridge utility.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from msgs_ifaces.msg import RoverStatus
import threading
import queue


class DomainRelayNode:
    """
    Relays messages from Domain 5 to Domain 4 using two separate nodes.
    """
    
    def __init__(self):
        self.message_queue = queue.Queue(maxsize=10)
        self.shutdown_event = threading.Event()
        
    def run_subscriber(self):
        """Subscriber node in Domain 5"""
        import os
        os.environ['ROS_DOMAIN_ID'] = '5'
        
        rclpy.init(args=None)
        
        class SubscriberNode(Node):
            def __init__(self, msg_queue):
                super().__init__('domain_relay_subscriber')
                self.msg_queue = msg_queue
                self.subscription = self.create_subscription(
                    RoverStatus,
                    '/tpc_rover_status',
                    self.callback,
                    10
                )
                self.get_logger().info('Domain 5 subscriber ready: /tpc_rover_status')
                self.count = 0
            
            def callback(self, msg):
                try:
                    self.msg_queue.put_nowait(msg)
                    self.count += 1
                    if self.count % 10 == 0:
                        self.get_logger().info(f'Relayed {self.count} messages (Domain 5→4)')
                except queue.Full:
                    self.get_logger().warn('Message queue full, dropping message')
        
        node = SubscriberNode(self.message_queue)
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    
    def run_publisher(self):
        """Publisher node in Domain 4"""
        import os
        os.environ['ROS_DOMAIN_ID'] = '4'
        
        # Small delay to ensure subscriber starts first
        import time
        time.sleep(1)
        
        rclpy.init(args=None)
        
        class PublisherNode(Node):
            def __init__(self, msg_queue):
                super().__init__('domain_relay_publisher')
                self.msg_queue = msg_queue
                self.publisher = self.create_publisher(
                    RoverStatus,
                    '/tpc_rover_status',
                    10
                )
                # Timer to check queue and publish
                self.timer = self.create_timer(0.1, self.publish_from_queue)
                self.get_logger().info('Domain 4 publisher ready: /tpc_rover_status')
            
            def publish_from_queue(self):
                try:
                    while not self.msg_queue.empty():
                        msg = self.msg_queue.get_nowait()
                        self.publisher.publish(msg)
                except queue.Empty:
                    pass
        
        node = PublisherNode(self.message_queue)
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    
    def run(self):
        """Run both subscriber and publisher in separate threads"""
        sub_thread = threading.Thread(target=self.run_subscriber, daemon=True)
        pub_thread = threading.Thread(target=self.run_publisher, daemon=True)
        
        sub_thread.start()
        pub_thread.start()
        
        print("Domain Relay Started:")
        print("  Domain 5 → /tpc_rover_status (subscribing)")
        print("  Domain 4 → /tpc_rover_status (publishing)")
        print("Press Ctrl+C to stop")
        
        try:
            # Keep main thread alive
            sub_thread.join()
            pub_thread.join()
        except KeyboardInterrupt:
            print("\nShutting down domain relay...")


def main():
    relay = DomainRelayNode()
    relay.run()


if __name__ == '__main__':
    main()
