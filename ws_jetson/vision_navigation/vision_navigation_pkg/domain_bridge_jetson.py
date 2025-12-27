#!/usr/bin/env python3
"""
Jetson Domain Bridge: Domain 6 → Domain 5
Relays steering control commands from internal vision domain to rover network domain.

Subscribes: /tpc_rover_fmctl (Domain 6) - from steering_control
Publishes:  /tpc_rover_fmctl (Domain 5) - to rover chassis_controller

This bridge runs with TWO ROS2 contexts (one per domain) in separate threads.
Only relays the final control command to minimize Domain 5 traffic.
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.context import Context
from std_msgs.msg import Float32MultiArray
import threading


class DomainBridgeSubscriber(Node):
    """Subscriber on Domain 6 (vision processing)"""
    
    def __init__(self, callback, context):
        super().__init__('bridge_sub_d6', context=context)
        self.callback = callback
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.sub = self.create_subscription(
            Float32MultiArray,
            'tpc_rover_fmctl',
            self._relay_callback,
            qos
        )
        
        self.get_logger().info("[Bridge D6 Sub] Subscribed to tpc_rover_fmctl on Domain 6")
    
    def _relay_callback(self, msg):
        """Forward message to Domain 5 publisher"""
        self.callback(msg)


class DomainBridgePublisher(Node):
    """Publisher on Domain 5 (rover network)"""
    
    def __init__(self, context):
        super().__init__('bridge_pub_d5', context=context)
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.pub = self.create_publisher(
            Float32MultiArray,
            'tpc_rover_fmctl',
            qos
        )
        
        self.msg_count = 0
        self.get_logger().info("[Bridge D5 Pub] Publishing tpc_rover_fmctl to Domain 5")
        
        # Heartbeat timer
        self.timer = self.create_timer(5.0, self._heartbeat)
    
    def relay_message(self, msg):
        """Publish message to Domain 5"""
        self.pub.publish(msg)
        self.msg_count += 1
    
    def _heartbeat(self):
        """Status update"""
        self.get_logger().info(f"[Bridge D6→D5] Relayed {self.msg_count} control commands")


def main():
    # Initialize rclpy
    rclpy.init()
    
    # Create two separate contexts for different domains
    ctx_d6 = Context()
    ctx_d6.init()
    
    ctx_d5 = Context()
    ctx_d5.init()
    
    # Set domain IDs
    os.environ['ROS_DOMAIN_ID'] = '5'
    pub_node = DomainBridgePublisher(ctx_d5)
    
    os.environ['ROS_DOMAIN_ID'] = '6'
    sub_node = DomainBridgeSubscriber(pub_node.relay_message, ctx_d6)
    
    # Create executors for each context
    executor_d6 = MultiThreadedExecutor(context=ctx_d6)
    executor_d6.add_node(sub_node)
    
    executor_d5 = MultiThreadedExecutor(context=ctx_d5)
    executor_d5.add_node(pub_node)
    
    # Spin both executors in separate threads
    thread_d6 = threading.Thread(target=executor_d6.spin, daemon=True)
    thread_d5 = threading.Thread(target=executor_d5.spin, daemon=True)
    
    thread_d6.start()
    thread_d5.start()
    
    print("[Domain Bridge] Running D6→D5 relay for /tpc_rover_fmctl")
    print("Press Ctrl+C to stop")
    
    try:
        thread_d6.join()
        thread_d5.join()
    except KeyboardInterrupt:
        print("\n[Domain Bridge] Shutting down...")
    finally:
        executor_d6.shutdown()
        executor_d5.shutdown()
        sub_node.destroy_node()
        pub_node.destroy_node()
        ctx_d6.try_shutdown()
        ctx_d5.try_shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
