#!/usr/bin/env python3
"""
Dual-Domain Steering Control Node - Domain 5 Output

Purpose:
    Bridges vision processing (Domain 6) to rover control (Domain 5).
    This node runs on Domain 5 but subscribes to Domain 6 topics via
    localhost shared memory (same Jetson machine).

Architecture:
    Input:  Domain 6 → /tpc_rover_nav_lane (from lane detection)
    Output: Domain 5 → /tpc_rover_fmctl (to rover chassis controller)

Multi-Domain Strategy:
    Since ROS2 Python doesn't easily support multiple contexts in one node,
    we use a simple approach:
    - Main node context on Domain 5 (publisher)
    - Subscribes to Domain 6 topics via DDS localhost discovery
    - Both domains run on same machine (Jetson) so DDS finds them automatically

Benefits vs Bridge Node:
    - No message relay overhead
    - Simple architecture (one process, one node)
    - Native ROS2 communication (no custom bridge logic)
    - Automatic via DDS multicast on localhost

Control Parameters:
    k_e1: Weight on heading error theta
    k_e2: Weight on lateral offset b
    k_p: Proportional gain
    k_i: Integral gain (accumulated error)
    k_d: Derivative gain (error rate)
    ema_alpha: Exponential moving average smoothing factor
    steer_max_deg: Maximum steering angle saturation (±degrees)
    steer_when_lost: Steering command when lane not detected (safety)

CSV Logging:
    Records control loop data to '~/almondmatcha/runs/logs/ws_jetson_control_d5_TIMESTAMP.csv'

Author: Vision Navigation System
Date: November 11, 2025
"""

import time
import csv
import os
from typing import Tuple
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from vision_navigation_pkg.control_filters import (
    ExponentialMovingAverageLPF,
    clamp,
    pid_controller
)


class DualDomainControlNode(Node):
    """
    Dual-domain steering controller.
    
    Subscribes to vision data from Domain 6 (localhost).
    Publishes control commands to Domain 5 (rover network).
    """

    def __init__(self) -> None:
        super().__init__('steering_control_domain5')

        # ===================== Control Gains =====================
        self.declare_parameter('k_e1', 1.0)
        self.declare_parameter('k_e2', 0.1)
        self.declare_parameter('k_p', 4.0)
        self.declare_parameter('k_i', 0.0)
        self.declare_parameter('k_d', 0.0)

        self.k_e1: float = float(self.get_parameter('k_e1').value)
        self.k_e2: float = float(self.get_parameter('k_e2').value)
        self.k_p: float = float(self.get_parameter('k_p').value)
        self.k_i: float = float(self.get_parameter('k_i').value)
        self.k_d: float = float(self.get_parameter('k_d').value)

        # ===================== Filter Parameters =====================
        self.declare_parameter('ema_alpha', 0.05)
        ema_alpha: float = float(self.get_parameter('ema_alpha').value)

        self.ema_theta = ExponentialMovingAverageLPF(ema_alpha)
        self.ema_b = ExponentialMovingAverageLPF(ema_alpha)

        # ===================== Safety Parameters =====================
        self.declare_parameter('steer_max_deg', 60.0)
        self.declare_parameter('steer_when_lost', 0.0)

        self.steer_max_deg: float = float(self.get_parameter('steer_max_deg').value)
        self.steer_when_lost: float = float(self.get_parameter('steer_when_lost').value)

        # ===================== PID State =====================
        self.integral: float = 0.0
        self.last_time: float = time.time()
        self.last_error: float = 0.0

        # ===================== QoS for Cross-Domain Communication =====================
        # Use BEST_EFFORT for lower latency (same machine, reliable network)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ===================== Publishers (Domain 5) =====================
        # This publisher is on Domain 5 (set via ROS_DOMAIN_ID environment)
        self.pub_fmctl = self.create_publisher(
            Float32MultiArray,
            'tpc_rover_fmctl',
            qos_profile
        )

        # ===================== Subscribers (Domain 6 via localhost) =====================
        # This subscriber will discover Domain 6 topics via DDS localhost multicast
        # No special configuration needed - DDS handles cross-domain on same machine
        self.sub_lane = self.create_subscription(
            Float32MultiArray,
            'tpc_rover_nav_lane',
            self._on_lane_data,
            qos_profile
        )

        # ===================== Logging =====================
        self._init_logging()

        # ===================== Heartbeat Timer =====================
        # Show node is alive and waiting for data
        self.lane_msg_count = 0
        self.pub_msg_count = 0
        self.heartbeat_timer = self.create_timer(5.0, self._heartbeat_callback)

        self.get_logger().info(
            f"Dual-domain control node initialized on Domain 5"
        )
        self.get_logger().info(
            f"Subscribing to Domain 6 'tpc_rover_nav_lane' via localhost DDS"
        )
        self.get_logger().info(
            f"Publishing to Domain 5 'tpc_rover_fmctl' for rover control"
        )
        self.get_logger().info(
            f"PID gains: Kp={self.k_p}, Ki={self.k_i}, Kd={self.k_d}"
        )

    # ===================== Initialization Methods =====================

    def _init_logging(self) -> None:
        """Initialize CSV logging."""
        log_dir = os.path.expanduser("~/almondmatcha/runs/logs")
        os.makedirs(log_dir, exist_ok=True)
        
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"ws_jetson_control_d5_{timestamp}.csv"
        self.csv_path: str = os.path.join(log_dir, filename)
        self.csv_file = open(self.csv_path, mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time_sec", "theta_ema", "b_ema", "u", "e_sum"])
        self.get_logger().info(f"Logging to: {self.csv_path}")

    # ===================== Subscription Callbacks =====================

    def _on_lane_data(self, msg: Float32MultiArray) -> None:
        """
        Process incoming lane detection data from Domain 6.
        
        Args:
            msg: Lane parameters [theta_deg, b_offset, detected_flag]
        """
        self.lane_msg_count += 1
        
        # ===== Extract and validate input =====
        if len(msg.data) < 3:
            self.get_logger().warn("Lane message incomplete")
            return

        theta = float(msg.data[0])
        b = float(msg.data[1])
        detected = bool(msg.data[2])

        # ===== Input Saturation =====
        theta = clamp(theta, -35, 35)
        b = clamp(b, -100, 100)

        # ===== EMA Filtering =====
        theta_ema = self.ema_theta.update(theta)
        b_ema = self.ema_b.update(b)

        # ===== Warm-up Period =====
        buffer_full = (self.ema_theta.is_full() and self.ema_b.is_full())
        detected_valid = detected and buffer_full

        # ===== Error Computation =====
        error_sum = (self.k_e1 * theta_ema) + (self.k_e2 * b_ema)

        # ===== PID Control =====
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        u, self.integral, self.last_error = pid_controller(
            error_sum,
            self.k_p, self.k_i, self.k_d,
            self.integral,
            self.last_error,
            dt,
            integral_limit=200.0
        )

        # ===== Steering Command =====
        steer_angle = u if detected_valid else self.steer_when_lost
        steer_angle = float(np.clip(steer_angle, -self.steer_max_deg, self.steer_max_deg))

        # ===== Logging =====
        self._log_control_data(time.time(), theta_ema, b_ema, u, error_sum)

        # ===== Publish Control Command to Domain 5 =====
        cmd_msg = Float32MultiArray()
        cmd_msg.data = [steer_angle, float(detected_valid)]
        self.pub_fmctl.publish(cmd_msg)
        self.pub_msg_count += 1

        # ===== Terminal Output =====
        status = "Detected" if detected_valid else "Lost"
        self.get_logger().info(
            f"[D6→D5] θ={theta_ema:.2f}° b={b_ema:.2f}px err={error_sum:.2f} "
            f"u={u:.2f} steer={steer_angle:.2f}° {status}"
        )
Heartbeat Methods =====================

    def _heartbeat_callback(self) -> None:
        """Periodic status update to show node is alive."""
        if self.lane_msg_count == 0:
            self.get_logger().warn(
                f"[D5 Steering] Waiting for lane data from Domain 6... "
                f"(Received: 0 | Published: {self.pub_msg_count})"
            )
        else:
            self.get_logger().info(
                f"[D5 Steering] Alive - Received: {self.lane_msg_count} | "
                f"Published: {self.pub_msg_count} to tpc_rover_fmctl"
            )

    # ===================== 
    # ===================== Logging Methods =====================

    def _log_control_data(
        self,
        timestamp: float,
        theta_ema: float,
        b_ema: float,
        u: float,
        error_sum: float
    ) -> None:
        """Log control loop data to CSV."""
        try:
            row = [timestamp, theta_ema, b_ema, u, error_sum]
            self.csv_writer.writerow(row)
            self.csv_file.flush()
        except Exception as e:
            self.get_logger().warn(f"CSV logging failed: {e}")

    # ===================== Cleanup =====================

    def destroy_node(self) -> None:
        """Close logging file."""
        try:
            if hasattr(self, 'csv_file') and self.csv_file:
                self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    try:
        node = DualDomainControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down dual-domain control node...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
