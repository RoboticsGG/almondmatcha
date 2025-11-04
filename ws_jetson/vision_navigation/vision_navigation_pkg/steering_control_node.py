#!/usr/bin/env python3
"""
Rover Control Node - Lane Following Control

Purpose:
    Closed-loop steering control for lane following. Implements PID controller
    that processes lane detection parameters and commands steering angle.

Topics Subscribed:
    /tpc_rover_nav_lane (std_msgs/Float32MultiArray) - Lane parameters [theta, b, detected]
        theta: Heading error from lane center (degrees)
        b: Lateral offset from lane center (pixels)
        detected: Detection flag (1.0 = valid, 0.0 = lost)

Topics Published:
    /tpc_rover_fmctl (std_msgs/Float32MultiArray) - Front module control [steer_angle, detected]
        steer_angle: Steering command in degrees (+right, -left)
        detected: Whether lane is currently detected

Control Parameters:
    k_e1: Weight on heading error theta
    k_e2: Weight on lateral offset b
    k_p: Proportional gain
    k_i: Integral gain (accumulated error)
    k_d: Derivative gain (error rate)
    ema_alpha: Exponential moving average smoothing factor
    steer_max_deg: Maximum steering angle saturation (±degrees)
    steer_when_lost: Steering command when lane not detected (safety)

Sign Convention:
    +steer_angle => Turn RIGHT
    -steer_angle => Turn LEFT

CSV Logging:
    Records control loop data to 'logs/rover_ctl_log_ver_3.csv':
    time_sec, zeta_ema, b_ema, u, e_sum

Author: Vision Navigation System
Date: November 4, 2025
"""

import time
import csv
import os
from typing import Tuple
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from vision_navigation_pkg.control_filters import (
    ExponentialMovingAverageLPF,
    clamp,
    pid_controller
)


class RoverControlNode(Node):
    """
    Lane following rover steering controller.
    
    Implements closed-loop PID control of steering angle based on
    lane detection feedback from vision system.
    """

    def __init__(self) -> None:
        super().__init__('node_rover_ctl')

        # ===================== Control Gains =====================
        self.declare_parameter('k_e1', 1.0)     # Heading error weight
        self.declare_parameter('k_e2', 0.1)     # Lateral offset weight
        self.declare_parameter('k_p', 4.0)      # Proportional gain
        self.declare_parameter('k_i', 0.0)      # Integral gain
        self.declare_parameter('k_d', 0.0)      # Derivative gain

        self.k_e1: float = float(self.get_parameter('k_e1').value)
        self.k_e2: float = float(self.get_parameter('k_e2').value)
        self.k_p: float = float(self.get_parameter('k_p').value)
        self.k_i: float = float(self.get_parameter('k_i').value)
        self.k_d: float = float(self.get_parameter('k_d').value)

        # ===================== Filter Parameters =====================
        self.declare_parameter('ema_alpha', 0.05)  # EMA smoothing factor
        ema_alpha: float = float(self.get_parameter('ema_alpha').value)

        self.ema_theta = ExponentialMovingAverageLPF(ema_alpha)
        self.ema_b = ExponentialMovingAverageLPF(ema_alpha)

        # ===================== Safety/Behavior Parameters =====================
        self.declare_parameter('steer_max_deg', 60.0)      # Maximum steering angle
        self.declare_parameter('steer_when_lost', 0.0)     # Steering when lane lost

        self.steer_max_deg: float = float(self.get_parameter('steer_max_deg').value)
        self.steer_when_lost: float = float(self.get_parameter('steer_when_lost').value)

        # ===================== PID State =====================
        self.integral: float = 0.0
        self.last_time: float = time.time()
        self.last_error: float = 0.0

        # ===================== Publishers =====================
        self.pub_fmctl = self.create_publisher(
            Float32MultiArray, 
            'tpc_rover_fmctl', 
            10
        )

        # ===================== Subscribers =====================
        self.sub_lane = self.create_subscription(
            Float32MultiArray, 
            'tpc_rover_nav_lane', 
            self._on_lane_data,
            10
        )

        # ===================== Logging =====================
        self._init_logging()

        self.get_logger().info(
            f"Rover control node initialized. "
            f"Steer limits: +/- {self.steer_max_deg} deg, "
            f"PID gains: Kp={self.k_p}, Ki={self.k_i}, Kd={self.k_d}"
        )

    # ===================== Initialization Methods =====================

    def _init_logging(self) -> None:
        """Initialize CSV logging for control loop data."""
        log_dir = "logs"
        os.makedirs(log_dir, exist_ok=True)
        
        self.csv_path: str = os.path.join(log_dir, "rover_ctl_log_ver_3.csv")
        self.csv_file = open(self.csv_path, mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time_sec", "theta_ema", "b_ema", "u", "e_sum"])

    # ===================== Subscription Callbacks =====================

    def _on_lane_data(self, msg: Float32MultiArray) -> None:
        """
        Process incoming lane detection data and update control.
        
        Expected message format:
            msg.data = [theta_deg, b_offset, detected_flag]
            
        Args:
            msg: Lane parameters from vision system
        """
        # ===== Extract and validate input =====
        if len(msg.data) < 3:
            self.get_logger().warn("Lane message incomplete")
            return

        theta = float(msg.data[0])       # Heading error (+ = needs right turn)
        b = float(msg.data[1])           # Lateral offset
        detected = bool(msg.data[2])     # Lane detection flag

        # ===== Input Saturation =====
        # Prevent spikes from affecting filter
        theta = clamp(theta, -35, 35)
        b = clamp(b, -100, 100)

        # ===== Exponential Moving Average Filtering =====
        theta_ema = self.ema_theta.update(theta)
        b_ema = self.ema_b.update(b)

        # ===== Warm-up Period =====
        # Consider lane detected only after filter buffers are full
        buffer_full = (self.ema_theta.is_full() and self.ema_b.is_full())
        detected_valid = detected and buffer_full

        # ===== Error Computation =====
        # Combined error: weighted sum of heading and lateral errors
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
            dt
        )

        # ===== Steering Command =====
        steer_angle = u

        # Safety: use configured steering when lane not detected
        if not detected_valid:
            steer_angle = self.steer_when_lost

        # Saturation: limit maximum steering angle
        steer_angle = float(np.clip(steer_angle, -self.steer_max_deg, self.steer_max_deg))

        # ===== Logging =====
        self._log_control_data(time.time(), theta_ema, b_ema, u, error_sum)

        # ===== Publish Control Command =====
        cmd_msg = Float32MultiArray()
        cmd_msg.data = [steer_angle, float(detected_valid)]
        self.pub_fmctl.publish(cmd_msg)

        # ===== Terminal Output =====
        self._log_control_status(theta_ema, b_ema, error_sum, u, steer_angle, detected_valid)

    # ===================== Logging Methods =====================

    def _log_control_data(
        self, 
        timestamp: float, 
        theta_ema: float, 
        b_ema: float, 
        u: float, 
        error_sum: float
    ) -> None:
        """
        Log control loop data to CSV file.
        
        Args:
            timestamp: Unix timestamp
            theta_ema: Filtered heading error
            b_ema: Filtered lateral offset
            u: Control output
            error_sum: Combined error
        """
        try:
            row = [timestamp, theta_ema, b_ema, u, error_sum]
            self.csv_writer.writerow(row)
            self.csv_file.flush()
        except Exception as e:
            self.get_logger().warn(f"CSV logging failed: {e}")

    def _log_control_status(
        self,
        theta_ema: float,
        b_ema: float,
        error_sum: float,
        u: float,
        steer_angle: float,
        detected: bool
    ) -> None:
        """
        Log control status to ROS logger.
        
        Args:
            theta_ema: Filtered heading error
            b_ema: Filtered lateral offset
            error_sum: Combined weighted error
            u: Raw PID output
            steer_angle: Final steering command
            detected: Lane detection status
        """
        status = "Detected" if detected else "Lost"
        self.get_logger().info(
            f"Lane (filtered): theta={theta_ema:7.2f} deg, b={b_ema:7.2f} | "
            f"error={error_sum:7.2f} | u={u:7.2f} | "
            f"steer={steer_angle:7.2f} deg | status={status}"
        )

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
        node = RoverControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
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

