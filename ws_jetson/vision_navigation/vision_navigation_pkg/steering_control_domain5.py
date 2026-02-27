#!/usr/bin/env python3
"""
steering_control_domain5.py  (entry point: rover_kinematic_control)
Workspace:  ws_jetson  |  Package: vision_navigation_pkg  |  Domain: 6 (sub) → 6 pub, relayed to D5

Rover Kinematic Control Node - Domain 6 (Vision Processing Domain)

Purpose:
    Implements bicycle-model-inspired kinematic control. Receives lane detection
    data and computes two coupled control outputs: steering angle and chassis speed.
    The domain bridge relays the combined command to Domain 5 for the RPi chassis
    controller.

Architecture:
    Input:  Domain 6 → tpc_rover_nav_lane  (from lane_detection, same domain)
    Output: Domain 6 → tpc_rover_ctrl_cmd  (relayed by domain_bridge to D5)

Control Outputs (tpc_rover_ctrl_cmd):
    data[0]  steer_angle  Steering command in degrees (+right, -left)
    data[1]  speed_cmd    Chassis speed command (0–100% PWM duty cycle)
    data[2]  detected     Lane detection validity flag (0.0 or 1.0)

Steering Control Parameters:
    k_e1: Weight on heading error theta
    k_e2: Weight on lateral offset b
    k_p / k_i / k_d: PID gains
    ema_alpha: Exponential moving average smoothing factor
    steer_max_deg: Maximum steering angle saturation (±degrees)
    steer_when_lost: Steering command when lane not detected (safety)

Speed Control Parameters:
    speed_ref: Desired forward speed when lane is detected (0–100% PWM duty cycle)
    speed_lost_ratio: Speed ratio applied when lane is temporarily lost (default 0.5)
    detection_timeout_sec: Seconds without detection before full safety stop (default 10.0)

    NOTE: Encoder-based closed-loop speed feedback (bicycle kinematic model) is a
    planned future feature. The domain bridge will relay tpc_chassis_sensors from
    D5 → D6 to provide wheel encoder data to this node.

CSV Logging:
    Records to ~/almondmatcha/runs/logs/ws_jetson_kinematic_ctrl_TIMESTAMP.csv

Author: Vision Navigation System
Date: February 27, 2026
"""

import time
import csv
import os
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


class RoverKinematicControlNode(Node):
    """
    Bicycle-model kinematic controller on Domain 6 (vision processing domain).

    Subscribes to lane detection data on Domain 6 (local).
    Computes coupled steering angle + chassis speed commands.
    Publishes tpc_rover_ctrl_cmd on Domain 6; the domain bridge relays it to D5.
    """

    def __init__(self) -> None:
        super().__init__('rover_kinematic_control')

        # ===================== Steering PID Gains =====================
        self.declare_parameter('k_e1', 1.0)    # Heading error weight
        self.declare_parameter('k_e2', 0.1)    # Lateral offset weight
        self.declare_parameter('k_p', 4.0)     # Proportional gain
        self.declare_parameter('k_i', 0.0)     # Integral gain
        self.declare_parameter('k_d', 0.0)     # Derivative gain

        self.k_e1: float = float(self.get_parameter('k_e1').value)
        self.k_e2: float = float(self.get_parameter('k_e2').value)
        self.k_p: float  = float(self.get_parameter('k_p').value)
        self.k_i: float  = float(self.get_parameter('k_i').value)
        self.k_d: float  = float(self.get_parameter('k_d').value)

        # ===================== EMA Filter =====================
        self.declare_parameter('ema_alpha', 0.05)
        ema_alpha: float = float(self.get_parameter('ema_alpha').value)

        self.ema_theta = ExponentialMovingAverageLPF(ema_alpha)
        self.ema_b     = ExponentialMovingAverageLPF(ema_alpha)

        # ===================== Steering Safety Parameters =====================
        self.declare_parameter('steer_max_deg', 60.0)
        self.declare_parameter('steer_when_lost', 0.0)

        self.steer_max_deg: float  = float(self.get_parameter('steer_max_deg').value)
        self.steer_when_lost: float = float(self.get_parameter('steer_when_lost').value)

        # ===================== Speed Control Parameters =====================
        # Unit: 0–100 (% PWM duty cycle). The STM32 divides spd_msg by 100 for motor PWM.
        # Values above 100 are meaningless (exceed 100% duty) and will be clamped in chassis_controller.
        self.declare_parameter('speed_ref', 50)               # Target speed at startup (0–100%)
        self.declare_parameter('speed_lost_ratio', 0.5)       # Speed multiplier when lane is lost
        self.declare_parameter('detection_timeout_sec', 10.0) # Timeout before safety stop

        self.speed_ref: int              = int(self.get_parameter('speed_ref').value)
        self.speed_lost_ratio: float     = float(self.get_parameter('speed_lost_ratio').value)
        self.detection_timeout_sec: float = float(self.get_parameter('detection_timeout_sec').value)

        # ===================== PID State =====================
        self.integral: float   = 0.0
        self.last_time: float  = time.time()
        self.last_error: float = 0.0

        # ===================== Detection Timeout State =====================
        self.detect_zero_active: bool  = False
        self.detect_zero_start: float  = 0.0

        # ===================== QoS =====================
        # BEST_EFFORT matches the Jetson domain bridge and chassis_controller QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ===================== Publisher =====================
        # Published on Domain 6; relayed to Domain 5 by domain_bridge_jetson
        # Output format: [steer_angle, speed_cmd, detected]
        self.pub_ctrl_cmd = self.create_publisher(
            Float32MultiArray,
            'tpc_rover_ctrl_cmd',
            qos
        )

        # ===================== Subscriber =====================
        self.sub_lane = self.create_subscription(
            Float32MultiArray,
            'tpc_rover_nav_lane',
            self._on_lane_data,
            qos
        )

        # ===================== Logging =====================
        self._init_logging()

        # ===================== Heartbeat Timer =====================
        self.lane_msg_count = 0
        self.pub_msg_count  = 0
        self.heartbeat_timer = self.create_timer(5.0, self._heartbeat_callback)

        self.get_logger().info("Rover Kinematic Control node initialized on Domain 6")
        self.get_logger().info(f"  Subscribe: tpc_rover_nav_lane (D6 local)")
        self.get_logger().info(f"  Publish:   tpc_rover_ctrl_cmd (D6 → bridge → D5)")
        self.get_logger().info(f"  Steering PID  Kp={self.k_p} Ki={self.k_i} Kd={self.k_d}")
        self.get_logger().info(f"  Speed ref={self.speed_ref}%  lost_ratio={self.speed_lost_ratio}"
                               f"  timeout={self.detection_timeout_sec}s  (unit: 0–100% duty)")

    # ===================== Initialization =====================

    def _init_logging(self) -> None:
        """Initialize CSV logging for the control loop."""
        log_dir = os.path.expanduser("~/almondmatcha/runs/logs")
        os.makedirs(log_dir, exist_ok=True)

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"ws_jetson_kinematic_ctrl_{timestamp}.csv"
        self.csv_path: str = os.path.join(log_dir, filename)
        self.csv_file = open(self.csv_path, mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time_sec", "theta_ema", "b_ema", "pid_u", "e_sum",
                                  "steer_angle", "speed_cmd", "detected"])
        self.get_logger().info(f"Logging to: {self.csv_path}")

    # ===================== Subscribers =====================

    def _on_lane_data(self, msg: Float32MultiArray) -> None:
        """
        Process lane detection data and publish combined kinematic control command.

        Expected input  tpc_rover_nav_lane: [theta_deg, b_offset, detected_flag]
        Published output tpc_rover_ctrl_cmd: [steer_angle, speed_cmd, detected]

        Args:
            msg: Lane parameters from lane_detection node.
        """
        self.lane_msg_count += 1

        # ===== Validate input =====
        if len(msg.data) < 3:
            self.get_logger().warn("tpc_rover_nav_lane message has fewer than 3 fields — skipping")
            return

        theta    = float(msg.data[0])   # Heading error in degrees (+ = needs right turn)
        b        = float(msg.data[1])   # Lateral pixel offset from lane center
        detected = bool(msg.data[2])    # Raw detection flag from vision

        # ===== Input saturation (prevent filter spikes) =====
        theta = clamp(theta, -35.0, 35.0)
        b     = clamp(b, -100.0, 100.0)

        # ===== EMA low-pass filtering =====
        theta_ema = self.ema_theta.update(theta)
        b_ema     = self.ema_b.update(b)

        # ===== Warm-up guard: wait for filter buffers to fill =====
        buffer_full   = self.ema_theta.is_full() and self.ema_b.is_full()
        detected_valid = detected and buffer_full

        # ===== Steering: PID on combined heading + lateral error =====
        error_sum = (self.k_e1 * theta_ema) + (self.k_e2 * b_ema)

        now = time.time()
        dt  = now - self.last_time
        self.last_time = now

        u, self.integral, self.last_error = pid_controller(
            error_sum,
            self.k_p, self.k_i, self.k_d,
            self.integral,
            self.last_error,
            dt,
            integral_limit=200.0
        )

        steer_angle = u if detected_valid else self.steer_when_lost
        steer_angle = float(np.clip(steer_angle, -self.steer_max_deg, self.steer_max_deg))

        # ===== Speed: detection-timeout safety logic =====
        speed_cmd = self._compute_speed_cmd(detected_valid)

        # ===== Logging =====
        self._log_control_data(now, theta_ema, b_ema, u, error_sum, steer_angle, speed_cmd,
                                detected_valid)

        # ===== Publish combined control command =====
        cmd_msg = Float32MultiArray()
        cmd_msg.data = [steer_angle, float(speed_cmd), float(detected_valid)]
        self.pub_ctrl_cmd.publish(cmd_msg)
        self.pub_msg_count += 1

        # ===== Terminal output =====
        status = "DETECTED" if detected_valid else "LOST"
        self.get_logger().info(
            f"[KIN] θ={theta_ema:.1f}° b={b_ema:.1f}px err={error_sum:.2f} "
            f"u={u:.2f} | steer={steer_angle:.1f}° spd={speed_cmd} [{status}]"
        )

    # ===================== Speed Control =====================

    def _compute_speed_cmd(self, detected_valid: bool) -> int:
        """
        Compute chassis speed command (0–255) based on lane detection state.

        Speed policy:
            - Lane detected                  → full speed_ref
            - Lane lost, within timeout      → speed_ref × speed_lost_ratio  (caution)
            - Lane lost, timeout exceeded    → 0  (safety stop)

        NOTE: Encoder-based closed-loop feedback is a planned future enhancement.
        When ready, this method will be extended with a speed PID using wheel encoder
        data relayed from tpc_chassis_sensors (D5 → D6 via domain_bridge_jetson).

        Args:
            detected_valid: True if lane is currently visible and filter is warm.

        Returns:
            Integer speed command, unit 0–100 (% PWM duty cycle).
            The STM32 motor_control.cpp converts: motor_duty = speed_percent / 100.0
        """
        if detected_valid:
            self.detect_zero_active = False
            return self.speed_ref

        # Lane not detected — start or continue the timeout counter
        if not self.detect_zero_active:
            self.detect_zero_start  = time.time()
            self.detect_zero_active = True

        elapsed = time.time() - self.detect_zero_start

        if elapsed >= self.detection_timeout_sec:
            return 0   # Safety stop: lane has been missing too long

        # Caution mode: reduce speed while waiting for lane to reappear
        return int(self.speed_ref * self.speed_lost_ratio)

    # ===================== Heartbeat =====================

    def _heartbeat_callback(self) -> None:
        """Periodic status log to confirm the node is alive."""
        if self.lane_msg_count == 0:
            self.get_logger().warn(
                f"[KIN] Waiting for tpc_rover_nav_lane data... "
                f"(published: {self.pub_msg_count})"
            )
        else:
            self.get_logger().info(
                f"[KIN] Alive — lane_rx={self.lane_msg_count} "
                f"ctrl_tx={self.pub_msg_count} → tpc_rover_ctrl_cmd (D6→bridge→D5)"
            )

    # ===================== Logging =====================

    def _log_control_data(
        self,
        timestamp: float,
        theta_ema: float,
        b_ema: float,
        pid_u: float,
        error_sum: float,
        steer_angle: float,
        speed_cmd: int,
        detected: bool
    ) -> None:
        """Write one control-loop row to the CSV log."""
        try:
            self.csv_writer.writerow([
                f"{timestamp:.6f}", f"{theta_ema:.4f}", f"{b_ema:.4f}",
                f"{pid_u:.4f}", f"{error_sum:.4f}",
                f"{steer_angle:.4f}", speed_cmd, int(detected)
            ])
            self.csv_file.flush()
        except Exception as e:
            self.get_logger().warn(f"CSV logging failed: {e}")

    # ===================== Cleanup =====================

    def destroy_node(self) -> None:
        """Flush and close the CSV log file on shutdown."""
        try:
            if hasattr(self, 'csv_file') and self.csv_file:
                self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    try:
        node = RoverKinematicControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down rover kinematic control node...")
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
