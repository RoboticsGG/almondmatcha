#!/usr/bin/env python3
"""
steering_control_domain5.py  (entry point: rover_kinematic_control)
Workspace:  ws_jetson  |  Package: vision_navigation_pkg
Architecture: Dual-context single process (Domain 6 sub + Domain 5 pub/sub)

Rover Kinematic Control Node

Purpose:
    Merged domain bridge + kinematic control in one process.
    Runs TWO rclpy contexts so it can subscribe to the vision domain (D6) and
    publish directly to the rover network (D5) without a separate relay process.

        D6 context -- sub: tpc_rover_nav_lane  (from lane_detection)
        D5 context -- pub: tpc_rover_ctrl_cmd   (to node_chassis_controller, RPi)
        D5 context -- sub: tpc_chassis_sensors  (stub: future encoder speed loop)

    Eliminates the former domain_bridge_jetson process:
    - One fewer process, one fewer pub/sub hop
    - D5 encoder data reachable with a single extra D5 subscription when needed
    - Camera topics stay isolated on D6 (not visible on rover DDS network)

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
    speed_ref: Desired forward speed when lane is detected (0-100% PWM duty cycle)
    speed_lost_ratio: Speed ratio applied when lane is temporarily lost (default 0.5)
    detection_timeout_sec: Seconds without detection before full safety stop (default 10.0)

    NOTE: Encoder-based closed-loop speed feedback is a planned future feature.
    Activate by uncommenting the tpc_chassis_sensors subscription in D5InterfaceNode
    and adding a speed PID callback.

CSV Logging:
    Records to ~/almondmatcha/runs/logs/ws_jetson_kinematic_ctrl_TIMESTAMP.csv

Author: AlmondMatcha Rover Team
Date: February 27, 2026
"""

import os
import time
import csv
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray

from vision_navigation_pkg.control_filters import (
    ExponentialMovingAverageLPF,
    clamp,
    pid_controller
)


# =============================================================================
# Domain 5 Interface Node
# =============================================================================

class D5InterfaceNode(Node):
    """
    Runs on Domain 5 (rover network).

    Responsibilities:
        - Publish tpc_rover_ctrl_cmd to node_chassis_controller (RPi)
        - Subscribe to tpc_chassis_sensors [stub: future encoder feedback]

    publish_ctrl_cmd() is called from the D6 thread -- rclpy Publisher.publish()
    is thread-safe, so no lock is required.
    """

    def __init__(self, context: Context) -> None:
        super().__init__('rover_kinematic_control_d5', context=context)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publisher: [steer_angle, speed_cmd, detected] -> chassis_controller (RPi)
        self.pub_ctrl_cmd = self.create_publisher(
            Float32MultiArray, 'tpc_rover_ctrl_cmd', qos
        )

        # -- Encoder feedback stub (future closed-loop speed control) -----------
        # To activate:
        #   1. Build/source msgs_ifaces in ws_jetson
        #   2. Import:  from msgs_ifaces.msg import ChassisSensors
        #   3. Uncomment the subscription below
        #   4. Implement _on_chassis_sensors() with a speed PID using
        #      msg.mt_lf_encode_msg and msg.mt_rt_encode_msg
        #
        # self.sub_chassis_sensors = self.create_subscription(
        #     ChassisSensors, 'tpc_chassis_sensors',
        #     self._on_chassis_sensors, qos
        # )
        # def _on_chassis_sensors(self, msg) -> None:
        #     # TODO: speed PID with encoder deltas
        #     pass
        # -----------------------------------------------------------------------

        self.relay_count = 0
        self.create_timer(5.0, self._heartbeat)

        self.get_logger().info(
            "[D5] Interface node ready on Domain 5 -- publishing tpc_rover_ctrl_cmd"
        )

    def publish_ctrl_cmd(self, msg: Float32MultiArray) -> None:
        """Publish a control command to Domain 5. Thread-safe."""
        self.pub_ctrl_cmd.publish(msg)
        self.relay_count += 1

    def _heartbeat(self) -> None:
        self.get_logger().info(
            f"[D5] Published {self.relay_count} commands -> tpc_rover_ctrl_cmd (Domain 5)"
        )


# =============================================================================
# Domain 6 Kinematic Control Node
# =============================================================================

class RoverKinematicControlNode(Node):
    """
    Runs on Domain 6 (Jetson localhost / vision domain).

    Subscribes to lane detection data, runs the PID steering + speed control
    loop, and pushes the result to Domain 5 by calling
    d5_interface.publish_ctrl_cmd() directly (same process, cross-context).
    """

    def __init__(self, context: Context, d5_interface: D5InterfaceNode) -> None:
        super().__init__('rover_kinematic_control', context=context)

        self.d5_interface = d5_interface

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
        # Unit: 0-100 (% PWM duty cycle). STM32 converts: motor_duty = speed_percent / 100.0
        self.declare_parameter('speed_ref', 50)
        self.declare_parameter('speed_lost_ratio', 0.5)
        self.declare_parameter('detection_timeout_sec', 10.0)

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
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ===================== Subscriber =====================
        # No local publisher — output goes via d5_interface.publish_ctrl_cmd()
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
        self.get_logger().info(f"  Subscribe (D6): tpc_rover_nav_lane")
        self.get_logger().info(f"  Publish  (D5):  tpc_rover_ctrl_cmd  [direct via D5InterfaceNode]")
        self.get_logger().info(f"  Steering PID  Kp={self.k_p} Ki={self.k_i} Kd={self.k_d}")
        self.get_logger().info(
            f"  Speed ref={self.speed_ref}%  lost_ratio={self.speed_lost_ratio}"
            f"  timeout={self.detection_timeout_sec}s  (unit: 0-100% duty)"
        )

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

        # ---- Publish to Domain 5 via D5InterfaceNode (same process, thread-safe) ----
        cmd_msg = Float32MultiArray()
        cmd_msg.data = [steer_angle, float(speed_cmd), float(detected_valid)]
        self.d5_interface.publish_ctrl_cmd(cmd_msg)
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
        Compute chassis speed command based on lane detection state.

        Policy:
            detected              -> speed_ref (full speed)
            lost, within timeout  -> speed_ref * speed_lost_ratio (caution)
            lost, timeout expired -> 0 (safety stop)

        NOTE: When encoder feedback is activated (tpc_chassis_sensors D5 sub),
        this method will be replaced by a speed PID using wheel encoder deltas.

        Returns:
            Integer speed command (0-100% PWM duty cycle).
        """
        if detected_valid:
            self.detect_zero_active = False
            return self.speed_ref

        if not self.detect_zero_active:
            self.detect_zero_start  = time.time()
            self.detect_zero_active = True

        elapsed = time.time() - self.detect_zero_start

        if elapsed >= self.detection_timeout_sec:
            return 0

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
                f"[KIN] Alive -- lane_rx={self.lane_msg_count} ctrl_tx={self.pub_msg_count}"
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


# =============================================================================
# Entry Point
# =============================================================================

def main() -> None:
    """
    Spin both contexts in separate threads.

    Context layout:
        ctx_d6 (Domain 6) -- RoverKinematicControlNode  [sub lane, compute]
        ctx_d5 (Domain 5) -- D5InterfaceNode             [pub ctrl_cmd, sub encoders stub]
    """

    # -- Create Domain 6 context --
    os.environ['ROS_DOMAIN_ID'] = '6'
    rclpy.init()           # bootstrap rclpy (default context, not used directly)
    ctx_d6 = Context()
    ctx_d6.init()

    # -- Create Domain 5 context --
    os.environ['ROS_DOMAIN_ID'] = '5'
    ctx_d5 = Context()
    ctx_d5.init()

    # -- Instantiate nodes --
    d5_node = D5InterfaceNode(context=ctx_d5)
    d6_node = RoverKinematicControlNode(context=ctx_d6, d5_interface=d5_node)

    # -- Create executors --
    exec_d6 = MultiThreadedExecutor(context=ctx_d6)
    exec_d6.add_node(d6_node)

    exec_d5 = MultiThreadedExecutor(context=ctx_d5)
    exec_d5.add_node(d5_node)

    # -- Spin in separate threads --
    thread_d6 = threading.Thread(target=exec_d6.spin, daemon=True)
    thread_d5 = threading.Thread(target=exec_d5.spin, daemon=True)

    thread_d6.start()
    thread_d5.start()

    print("[rover_kinematic_control] Running "
          "(D6 sub tpc_rover_nav_lane | D5 pub tpc_rover_ctrl_cmd)")
    print("Press Ctrl+C to stop")

    try:
        thread_d6.join()
        thread_d5.join()
    except KeyboardInterrupt:
        print("\n[rover_kinematic_control] Shutting down...")
    finally:
        exec_d6.shutdown()
        exec_d5.shutdown()
        d6_node.destroy_node()
        d5_node.destroy_node()
        ctx_d6.try_shutdown()
        ctx_d5.try_shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
