#!/usr/bin/env python3
"""
rover_kinematic_control_node.py  (entry point: rover_kinematic_control)
Workspace:  ws_jetson  |  Package: vision_navigation
Architecture: Dual-context single process (Domain 6 sub + Domain 5 pub/sub)

Rover Kinematic Control Node

Purpose:
    Merged domain bridge + kinematic control in one process.
    Runs TWO rclpy contexts so it can subscribe to the vision domain (D6) and
    publish directly to the rover network (D5) without a separate relay process.

        D6 context -- sub: tpc_rover_nav_lane  (from lane_detection)
        D5 context -- pub: tpc_rover_ctrl_cmd   (to chassis_controller_node, RPi)
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
    k_lat: Static feedback gain on lateral offset b (deg/metre)
    k_head: Static feedback gain on heading error theta (deg/deg)
    wheelbase_m: Front-to-rear axle distance, for the Ackermann feedforward
    bev_px_per_m: BEV pixel scale, to convert curvature (1/px) to (1/m)
    ema_alpha: Exponential moving average smoothing factor
    steer_max_deg: Maximum steering angle saturation (±degrees)
    steer_when_lost: Steering command when lane not detected (safety)

Control Law (static-gain feedback + kinematic feedforward, single-step MPC
form -- k_lat/k_head are a fixed LQR solution on the linearized error model
below, not re-solved online each step):
    b_dot     = v*theta
    theta_dot = -(v/L)*delta_fb
    (v ~ 0.15-0.20 m/s cruise speed, L = wheelbase_m, dt ~ 1/20 s control
    loop period -- the loop is event-driven off tpc_rover_nav_lane, not a
    fixed-rate timer, so this dt is the field-measured lane_detection
    publish rate, not a configured constant)

    u_fb    = k_lat*b_ema + k_head*theta_ema      -- feedback on current error
    u_ff    = atan(wheelbase_m * curvature_m_ema) -- Ackermann feedforward on curve ahead
    u_total = u_fb + u_ff                          -- combined, then clamped

    Sign convention (must match, or feedback becomes positive feedback):
    theta/b/steer_angle are all "+ = correct by steering right" in this
    codebase, so both feedback terms carry a PLUS sign here -- not the
    textbook "-k1*e_lat - k2*e_heading" form, which assumes the opposite
    steering-angle-positive convention (e.g. ISO 8855, positive = left).

Speed Control Parameters:
    speed_ref: Desired forward speed when lane is detected (0-100% PWM duty cycle)
    speed_lost_ratio: Speed ratio applied when lane is temporarily lost (default 0.5)
    detection_timeout_sec: Seconds without detection before full safety stop (default 10.0)

    NOTE: Encoder-based closed-loop speed feedback is a planned future feature.
    Activate by uncommenting the tpc_chassis_sensors subscription in D5InterfaceNode
    and adding a speed PID callback.

CSV Logging:
    Records to <ws_jetson>/runs/run_NNN_<stamp>/kinematic_control.csv
    Writes happen on a background thread via a queue so the
    tpc_rover_nav_lane callback never blocks on file I/O.

Author: AlmondMatcha Rover Team
Date: February 27, 2026
"""

import math
import os
import time
import csv
import queue
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray

from vision_navigation.control_filters import (
    ExponentialMovingAverageLPF,
    clamp
)
from vision_navigation.helpers import resolve_run_dir


# =============================================================================
# Domain 5 Interface Node
# =============================================================================

class D5InterfaceNode(Node):
    """
    Runs on Domain 5 (rover network).

    Responsibilities:
        - Publish tpc_rover_ctrl_cmd to chassis_controller_node (RPi)
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

        # ===================== Static Feedback Gains =====================
        # Control law: steer = k_lat*b_ema + k_head*theta_ema + atan(wheelbase_m*curvature_m_ema)
        # A single-step (MPC-style) proportional law. k_lat/k_head are a
        # fixed discrete-time LQR solution (solved offline against the
        # linearized error model in the module docstring, v ~ 0.15-0.20 m/s,
        # L = 0.4875 m, dt ~ 1/20 s) -- not re-solved online, applied as a
        # static gain each step.
        #   LQR solve gave k1 = 3.162 rad/m, k2 = 2.024 rad/rad.
        #   k_lat  = k1 * (180/pi) = 181.17  -- deg/m: b is metres but steer
        #            output is degrees, so this ratio needs the rad->deg
        #            conversion.
        #   k_head = k2             = 2.024  -- deg/deg: theta and steer are
        #            both angles, so the ratio is unit-invariant and needs
        #            no conversion.
        self.declare_parameter('k_lat', 181.17)  # deg per metre of lateral offset b
        self.declare_parameter('k_head', 2.024)  # deg per deg of heading error theta

        self.k_lat: float  = float(self.get_parameter('k_lat').value)
        self.k_head: float = float(self.get_parameter('k_head').value)

        # ===================== Ackermann Feedforward =====================
        # u_ff = atan(wheelbase_m * curvature_m_ema) -- anticipates the curve
        # ahead instead of waiting for heading/offset error (feedback) to
        # build up. curvature (from the lane fit) is in BEV pixels (1/px);
        # converting to real 1/m needs the BEV scale: curvature_m = 2 *
        # bev_px_per_m * curvature_px (see rover_kinematic_control_params.yaml
        # for the A = 1/(2*R*S) derivation).
        self.declare_parameter('wheelbase_m', 0.4875)    # Front-to-rear axle distance
        self.declare_parameter('bev_px_per_m', 200.0)    # BEV scale (LaneDetectionConfig.BEV_PX_PER_M)

        self.wheelbase_m: float  = float(self.get_parameter('wheelbase_m').value)
        self.bev_px_per_m: float = float(self.get_parameter('bev_px_per_m').value)

        # ===================== EMA Filter =====================
        self.declare_parameter('ema_alpha', 0.05)
        ema_alpha: float = float(self.get_parameter('ema_alpha').value)

        self.ema_theta     = ExponentialMovingAverageLPF(ema_alpha)
        self.ema_b         = ExponentialMovingAverageLPF(ema_alpha)
        self.ema_curvature = ExponentialMovingAverageLPF(ema_alpha)

        # ===================== Steering Safety Parameters =====================
        # 45.0 (not the full ±60 mechanical limit) as a conservative margin
        # while this static-gain law is unvalidated on the track.
        self.declare_parameter('steer_max_deg', 45.0)
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
        self.get_logger().info(
            f"  Steering  k_lat={self.k_lat} k_head={self.k_head}  "
            f"wheelbase_m={self.wheelbase_m}  steer_max_deg={self.steer_max_deg}"
        )
        self.get_logger().info(
            f"  Speed ref={self.speed_ref}%  lost_ratio={self.speed_lost_ratio}"
            f"  timeout={self.detection_timeout_sec}s  (unit: 0-100% duty)"
        )

    # ===================== Initialization =====================

    def _init_logging(self) -> None:
        """Initialize CSV logging for the control loop."""
        # One run = one directory: <ws_jetson>/runs/run_NNN_<stamp>/, shared with
        # every other logging node on this machine via $ROVER_RUN_DIR.
        self.csv_path: str = os.path.join(resolve_run_dir("ws_jetson"), "kinematic_control.csv")
        self._csv_header: list = ["time_sec", "theta_ema", "b_ema", "curvature_ema", "u_fb",
                                  "u_ff", "steer_angle", "speed_cmd", "detected"]
        # The file is created by the worker on the first row, not here: a run that
        # never receives lane data then leaves no empty CSV behind, and the file's
        # presence is evidence the control loop actually ran.

        # ===================== Async CSV Logging =====================
        # File I/O runs on a background thread so the tpc_rover_nav_lane
        # callback (and the tpc_rover_ctrl_cmd publish inside it) never
        # blocks on disk write -- same pattern as lane_detection_node.py.
        self._log_queue: "queue.Queue" = queue.Queue()
        self._log_thread = threading.Thread(target=self._log_worker, daemon=True)
        self._log_thread.start()

        self.get_logger().info(f"Logging to: {self.csv_path}")

    # ===================== Subscribers =====================

    def _on_lane_data(self, msg: Float32MultiArray) -> None:
        """
        Process lane detection data and publish combined kinematic control command.

        Expected input  tpc_rover_nav_lane: [curvature, theta_deg, b_offset, detected_flag]
        Published output tpc_rover_ctrl_cmd: [steer_angle, speed_cmd, detected]

        Combines static-gain feedback (on heading + lateral error) with an
        Ackermann feedforward (proportional to curve sharpness ahead) so the
        rover starts steering into a curve before heading/offset error alone
        would trigger it.

        Args:
            msg: Lane parameters from lane_detection node.
        """
        self.lane_msg_count += 1

        # ===== Validate input =====
        if len(msg.data) < 4:
            self.get_logger().warn("tpc_rover_nav_lane message has fewer than 4 fields — skipping")
            return

        curvature = float(msg.data[0])  # Parabola coefficient A (x = A*y^2 + B*y + C), BEV px (1/px)
        theta     = float(msg.data[1])  # Heading error in degrees (+ = needs right turn)
        b         = float(msg.data[2])  # Lateral offset from lane center, metres
        detected  = bool(msg.data[3])   # Raw detection flag from vision

        # ===== Reject non-finite geometry =====
        # A NaN here is not a usable measurement, and it must never reach the
        # filters: clamp() propagates it and the EMA would then be permanently
        # NaN. Treat it as "lane not detected" instead.
        if not (math.isfinite(curvature) and math.isfinite(theta) and math.isfinite(b)):
            detected = False

        now = time.time()

        if detected:
            # ===== Input saturation (prevent filter spikes) =====
            theta = clamp(theta, -35.0, 35.0)
            b     = clamp(b, -0.50, 0.50)   # metres (was +-100.0 px)

            # ===== EMA low-pass filtering =====
            theta_ema     = self.ema_theta.update(theta)
            b_ema         = self.ema_b.update(b)
            curvature_ema = self.ema_curvature.update(curvature)

            # ===== Warm-up guard: wait for filter buffers to fill =====
            detected_valid = (
                self.ema_theta.is_full() and
                self.ema_b.is_full() and
                self.ema_curvature.is_full()
            )

            # ===== Feedback: static gain on heading + lateral error =====
            # Sign: theta/b/steer_angle all use "+ = correct by steering
            # right" in this codebase, so both terms are added, not
            # subtracted (see the Control Law note in the module docstring).
            u_fb = (self.k_lat * b_ema) + (self.k_head * theta_ema)

            # ===== Feedforward: Ackermann angle for the curve ahead =====
            # curvature_ema is the fitted parabola coefficient A in BEV
            # pixels (1/px); A = 1/(2*R*S) with BEV scale S, so the real
            # curvature is 1/R = 2*S*A.
            curvature_m_ema = 2.0 * self.bev_px_per_m * curvature_ema
            u_ff = math.degrees(math.atan(self.wheelbase_m * curvature_m_ema))
            u_total = u_fb + u_ff
        else:
            # ===== Lane lost: hold filter state, don't invent data =====
            # Previously the EMA was updated unconditionally, so every lost
            # frame fed it a placeholder value and only the *output* was
            # gated on `detected`. The filter therefore drifted to whatever
            # the placeholder was while the lane was missing, and the instant
            # detection returned the rover steered on that fabricated error
            # instead of the real measurement. Holding state means recovery
            # resumes from the last genuine reading.
            theta_ema     = self.ema_theta.ema if self.ema_theta.ema is not None else 0.0
            b_ema         = self.ema_b.ema if self.ema_b.ema is not None else 0.0
            curvature_ema = self.ema_curvature.ema if self.ema_curvature.ema is not None else 0.0
            u_fb          = 0.0
            u_ff          = 0.0
            detected_valid = False

        steer_angle = u_total if detected_valid else self.steer_when_lost
        steer_angle = float(np.clip(steer_angle, -self.steer_max_deg, self.steer_max_deg))

        # ===== Speed: detection-timeout safety logic =====
        speed_cmd = self._compute_speed_cmd(detected_valid)

        # ===== Logging =====
        self._log_control_data(now, theta_ema, b_ema, curvature_ema, u_fb, u_ff,
                                steer_angle, speed_cmd, detected_valid)

        # ---- Publish to Domain 5 via D5InterfaceNode (same process, thread-safe) ----
        cmd_msg = Float32MultiArray()
        cmd_msg.data = [steer_angle, float(speed_cmd), float(detected_valid)]
        self.d5_interface.publish_ctrl_cmd(cmd_msg)
        self.pub_msg_count += 1

        # ===== Terminal output =====
        status = "DETECTED" if detected_valid else "LOST"
        self.get_logger().info(
            f"[KIN] θ={theta_ema:.1f}° b={b_ema:.3f}m curv={curvature_ema:.5f} "
            f"u_fb={u_fb:.2f} u_ff={u_ff:.2f} | steer={steer_angle:.1f}° spd={speed_cmd} [{status}]"
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
        curvature_ema: float,
        u_fb: float,
        u_ff: float,
        steer_angle: float,
        speed_cmd: int,
        detected: bool
    ) -> None:
        """Enqueue one control-loop row for asynchronous CSV logging (non-blocking)."""
        row = [
            f"{timestamp:.6f}", f"{theta_ema:.4f}", f"{b_ema:.4f}", f"{curvature_ema:.6f}",
            f"{u_fb:.4f}", f"{u_ff:.4f}",
            f"{steer_angle:.4f}", speed_cmd, int(detected)
        ]
        self._log_queue.put(row)

    def _log_worker(self) -> None:
        """
        Background thread: drains the log queue and writes CSV rows.

        Runs off the tpc_rover_nav_lane callback so a slow disk never delays
        the steering/speed command published to tpc_rover_ctrl_cmd.
        """
        while True:
            item = self._log_queue.get()
            if item is None:
                self._log_queue.task_done()
                break
            try:
                os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
                new_file = not os.path.isfile(self.csv_path)
                with open(self.csv_path, 'a', newline='') as f:
                    w = csv.writer(f)
                    if new_file:
                        w.writerow(self._csv_header)
                    w.writerow(item)
            except Exception as e:
                self.get_logger().warn(f"CSV logging failed: {e}")
            finally:
                self._log_queue.task_done()

    # ===================== Cleanup =====================

    def destroy_node(self) -> None:
        """Drain the log queue and stop the background logging thread on shutdown."""
        try:
            self._log_queue.put(None)
            self._log_thread.join(timeout=2.0)
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
