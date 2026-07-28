#!/usr/bin/env python3
"""
lane_detection_node.py
Workspace:  ws_jetson  |  Package: vision_navigation  |  Domain: 6

Visual Navigation Image Processing Node

Purpose:
    Processes RGB frames from camera to detect lane markers and compute
    navigation parameters (steering angle, lateral offset).

Topics Subscribed:
    /tpc_rover_d415_rgb (sensor_msgs/Image, bgr8) - RGB camera stream

Topics Published:
    /tpc_rover_nav_lane (std_msgs/Float32MultiArray) - Lane parameters [curvature, theta, b, detected]
        curvature: Parabola coefficient A (x = A*y^2 + B*y + C), rover-centered frame
        theta: Steering angle error (degrees)
        b: Lateral offset from lane center
        detected: Boolean flag (1.0 = detected, 0.0 = not detected)

Parameters:
    show_window (bool, default: False) - Display lane detection visualization

Algorithm:
    1. Frame capture from camera topic
    2. Color filtering and edge detection (LAB color space)
    3. Gradient analysis and morphological operations
    4. Perspective transform to bird's-eye view
    5. Lane detection using 2nd-degree polyfit (curve fitting)
    6. Parameter calculation (curvature, theta, b)
    7. Publish results; log to CSV asynchronously (background thread)

CSV Logging:
    Saves detection results to '<ws_jetson>/runs/run_NNN_<stamp>/lane_detection.csv' with:
    timestamp, curvature, theta, b, detected, fps
    fps is a rolling-window measurement of achieved throughput (wall-clock
    time between the last FPS_WINDOW processed frames), not the configured
    camera capture rate.
    Writes happen on a background thread via a queue so the image callback
    never blocks on file I/O.

Author: Vision Navigation System
Date: November 4, 2025
"""

import math
import csv
import os
import queue
import threading
import time
from collections import deque
from datetime import datetime
from typing import Optional, Tuple
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

from vision_navigation.lane_detector import process_frame
from vision_navigation.config import LaneDetectionConfig
from vision_navigation.helpers import resolve_run_dir


class LaneDetectionNode(Node):
    """
    Navigation image processing node using lane detection.
    
    Receives RGB frames, detects lane boundaries, and publishes
    lane parameters for rover steering control.
    """

    def __init__(self) -> None:
        super().__init__('lane_detection_node')

        # ===================== QoS Configuration =====================
        # Sensor data: low latency, best effort delivery
        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # ===================== Subscriptions =====================
        self.sub_rgb = self.create_subscription(
            Image, 
            'tpc_rover_d415_rgb', 
            self._on_rgb_frame,
            qos
        )

        # ===================== Publishers =====================
        self.pub_lane = self.create_publisher(
            Float32MultiArray, 
            'tpc_rover_nav_lane', 
            10
        )

        # ===================== Utilities =====================
        self.bridge = CvBridge()

        # ===================== Parameters =====================
        self.declare_parameter('show_window', False)
        self.show_window: bool = bool(self.get_parameter('show_window').value)

        # ===== ROI / crop / sliding-window parameters =====
        # Exposed as ROS parameters (not hardcoded in lane_detector.py) since
        # the crop makes their correct values depend on the configured
        # camera resolution -- tune via the params yaml, not code edits.
        roi_flat_default = [
            float(v) for pair in LaneDetectionConfig.ROI_BASE_POINTS for v in pair
        ]
        self.declare_parameter('roi_base_points', roi_flat_default)
        self.declare_parameter('roi_base_width', LaneDetectionConfig.ROI_BASE_WIDTH)
        self.declare_parameter('roi_base_height', LaneDetectionConfig.ROI_BASE_HEIGHT)
        self.declare_parameter('crop_margin_px', LaneDetectionConfig.CROP_MARGIN_PX)
        self.declare_parameter('sliding_windows', LaneDetectionConfig.SLIDING_WINDOWS)
        self.declare_parameter('window_margin', LaneDetectionConfig.WINDOW_MARGIN)
        self.declare_parameter('min_window_pixels', LaneDetectionConfig.MIN_WINDOW_PIXELS)
        self.declare_parameter('min_lane_pixels', LaneDetectionConfig.MIN_LANE_PIXELS)
        self.declare_parameter('bev_width_px', LaneDetectionConfig.BEV_WIDTH_PX)
        self.declare_parameter('bev_height_px', LaneDetectionConfig.BEV_HEIGHT_PX)
        self.declare_parameter('search_band_px', LaneDetectionConfig.SEARCH_BAND_PX)

        roi_flat = [float(v) for v in self.get_parameter('roi_base_points').value]
        if len(roi_flat) != 8:
            self.get_logger().warn(
                f"roi_base_points expected 8 values (4 corners x,y), got {len(roi_flat)} "
                f"-- falling back to the default ROI"
            )
            roi_flat = roi_flat_default
        self.roi_base_points = np.float32(roi_flat).reshape(4, 2)
        self.roi_base_width: float = float(self.get_parameter('roi_base_width').value)
        self.roi_base_height: float = float(self.get_parameter('roi_base_height').value)
        self.crop_margin_px: float = float(self.get_parameter('crop_margin_px').value)
        self.sliding_windows: int = int(self.get_parameter('sliding_windows').value)
        self.window_margin: int = int(self.get_parameter('window_margin').value)
        self.min_window_pixels: int = int(self.get_parameter('min_window_pixels').value)
        self.min_lane_pixels: int = int(self.get_parameter('min_lane_pixels').value)
        self.bev_width_px: int = int(self.get_parameter('bev_width_px').value)
        self.bev_height_px: int = int(self.get_parameter('bev_height_px').value)
        self.search_band_px: float = float(self.get_parameter('search_band_px').value)

        # ===== Tracked lane position (wrong-line protection) =====
        # Column the lane was found at last frame, in warped-canvas pixels.
        # Passed back into the detector so the search stays on the line already
        # being followed instead of re-picking the strongest of several
        # parallel painted lines every frame. None = search the canvas centre,
        # which is where the rover starts (straddling the centre line).
        self._search_center: Optional[float] = None

        # ===================== Visualization =====================
        self.window_name = "lane_detection"
        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 1280, 720)

        # ===================== State =====================
        self.frame_count: int = 0

        # ===== Throughput (FPS) measurement =====
        # Rolling window of frame-processed timestamps, used to compute
        # achieved throughput rather than the configured capture rate
        # (camera_stream_node's `fps` param is just a target, not a
        # measurement -- see HANDOFF_vision_fps_logging.md).
        self.FPS_WINDOW: int = 30
        self._frame_times: "deque[float]" = deque(maxlen=self.FPS_WINDOW)

        # One run = one directory: <ws_jetson>/runs/run_NNN_<stamp>/, shared with
        # every other logging node on this machine via $ROVER_RUN_DIR.
        # Created lazily by the log worker, so a run with no frames leaves nothing.
        self.csv_path: str = os.path.join(resolve_run_dir("ws_jetson"), "lane_detection.csv")
        self._csv_header_written: bool = False

        # ===================== Async CSV Logging =====================
        # File I/O runs on a background thread so the image callback never
        # blocks on disk writes (blocking here was dropping camera frames).
        self._log_queue: "queue.Queue" = queue.Queue()
        self._log_thread = threading.Thread(target=self._log_worker, daemon=True)
        self._log_thread.start()

        self.get_logger().info(
            f"Navigation processing node initialized. "
            f"Visualization: {'enabled' if self.show_window else 'disabled'}, "
            f"Logging to: {self.csv_path}"
        )

    # ===================== Subscription Callbacks =====================

    def _on_rgb_frame(self, msg: Image) -> None:
        """
        Process incoming RGB frame and publish lane parameters.
        
        Args:
            msg: ROS Image message (bgr8 encoding)
        """
        # Convert ROS image message to OpenCV BGR format
        try:
            bgr_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # ===================== Lane Detection =====================
        curvature, theta, b, detected = process_frame(
            bgr_frame,
            roi_base_points=self.roi_base_points,
            roi_base_width=self.roi_base_width,
            roi_base_height=self.roi_base_height,
            crop_margin_px=self.crop_margin_px,
            sliding_windows=self.sliding_windows,
            window_margin=self.window_margin,
            min_window_pixels=self.min_window_pixels,
            min_lane_pixels=self.min_lane_pixels,
            bev_width=self.bev_width_px,
            bev_height=self.bev_height_px,
            search_center=self._search_center,
            search_band=self.search_band_px,
        )

        # ===================== Track the lane across frames =====================
        # `b` is the fitted offset from the canvas centre at the canvas bottom
        # row, which is exactly where the next frame's first search window
        # starts -- so it is the correct seed, with no re-derivation needed.
        # On a lost frame the seed is dropped rather than held: a stale seed
        # after a long gap is more likely to be wrong than the centre, and the
        # rover runs centred on the middle line.
        if detected and b is not None and math.isfinite(b):
            self._search_center = (self.bev_width_px / 2.0) + b
        else:
            self._search_center = None

        # ===================== Value Validation =====================
        # Handle NaN and None values
        curvature_clean = self._validate_float(curvature, 0.0)
        theta_clean = self._validate_float(theta, 0.0)
        # Fall back to 0.0 (neutral), matching curvature/theta. NaN must never go
        # on the wire: downstream clamp(NaN, -100, 100) collapses to -100 --
        # max(-100, min(NaN, 100)) -> max(-100, NaN) -> -100 -- so every
        # not-detected frame silently became a full-scale LEFT lateral offset.
        b_clean = self._validate_float(b, 0.0)
        detected_val = 1.0 if detected else 0.0

        # ===================== Publish Results =====================
        lane_msg = Float32MultiArray()
        lane_msg.data = [curvature_clean, theta_clean, b_clean, detected_val]
        self.pub_lane.publish(lane_msg)

        # ===================== Throughput Measurement =====================
        fps = self._update_fps()

        # ===================== Logging =====================
        self._log_to_csv(curvature_clean, theta_clean, b_clean, detected_val, fps)
        self._log_to_terminal(curvature_clean, theta_clean, b_clean, detected_val, fps)

        # ===================== Visualization =====================
        if self.show_window:
            self._visualize_frame(bgr_frame, theta_clean, b_clean, detected)

    # ===================== Throughput Methods =====================

    def _update_fps(self) -> float:
        """
        Record this frame's processing time and return the rolling-window FPS.

        Windowed rather than instantaneous (1/dt of a single frame gap) so
        the logged number reflects sustained throughput instead of jitter.

        Returns:
            Achieved frames-per-second over the last FPS_WINDOW frames, or
            0.0 until at least two samples have been collected.
        """
        now = time.monotonic()
        self._frame_times.append(now)
        if len(self._frame_times) < 2:
            return 0.0
        elapsed = self._frame_times[-1] - self._frame_times[0]
        if elapsed <= 0.0:
            return 0.0
        return (len(self._frame_times) - 1) / elapsed

    # ===================== Data Validation Methods =====================

    @staticmethod
    def _validate_float(value: any, default: float) -> float:
        """
        Validate and convert value to float, returning default if invalid.
        
        Args:
            value: Value to validate
            default: Default value if validation fails
            
        Returns:
            Valid float value or default
        """
        try:
            if value is None:
                return default
            fval = float(value)
            if math.isnan(fval):
                return default
            return fval
        except (TypeError, ValueError):
            return default

    # ===================== Logging Methods =====================

    def _log_to_csv(self, curvature: float, theta: float, b: float, detected: float, fps: float) -> None:
        """
        Enqueue a row for asynchronous CSV logging (non-blocking).

        Args:
            curvature: Parabola coefficient A (x = A*y^2 + B*y + C)
            theta: Steering angle (degrees)
            b: Lateral offset
            detected: Detection flag (1.0 or 0.0)
            fps: Rolling-window achieved throughput (frames/sec)
        """
        timestamp = datetime.now().isoformat()
        self._log_queue.put((timestamp, curvature, theta, b, detected, fps))

    def _log_worker(self) -> None:
        """
        Background thread: drains the log queue and writes CSV rows.

        Runs off the image callback so a slow disk (SD card / eMMC) never
        causes camera frames to be dropped waiting on file I/O.
        """
        while True:
            item = self._log_queue.get()
            if item is None:
                self._log_queue.task_done()
                break
            timestamp, curvature, theta, b, detected, fps = item
            try:
                os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
                file_exists = os.path.isfile(self.csv_path)
                with open(self.csv_path, 'a', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    if not file_exists and not self._csv_header_written:
                        writer.writerow(['timestamp', 'curvature', 'theta', 'b', 'detected', 'fps'])
                        self._csv_header_written = True
                    writer.writerow([timestamp, curvature, theta, b, detected, fps])
            except Exception as e:
                self.get_logger().warn(f"CSV logging failed: {e}")
            finally:
                self._log_queue.task_done()

    def _log_to_terminal(self, curvature: float, theta: float, b: float, detected: float, fps: float) -> None:
        """
        Log frame results to terminal.

        Args:
            curvature: Parabola coefficient A (x = A*y^2 + B*y + C)
            theta: Steering angle (degrees)
            b: Lateral offset
            detected: Detection flag (1.0 or 0.0)
            fps: Rolling-window achieved throughput (frames/sec)
        """
        self.frame_count += 1
        status = "Detected" if detected > 0.5 else "Not Detected"
        self.get_logger().info(
            f"Frame {self.frame_count}: curvature={curvature:9.6f}, theta={theta:7.3f} deg, "
            f"b={b:7.3f}, fps={fps:5.1f}, status={status}"
        )

    # ===================== Visualization Methods =====================

    def _visualize_frame(self, bgr_frame: np.ndarray, theta: float, b: float, detected: bool) -> None:
        """
        Display frame with lane detection overlay.
        
        Args:
            bgr_frame: OpenCV BGR image
            theta: Steering angle (degrees)
            b: Lateral offset
            detected: Detection flag
        """
        vis = bgr_frame.copy()
        h, w = vis.shape[:2]

        # ===== Draw Status Text =====
        status_color = (0, 255, 0) if detected else (0, 0, 255)
        status_text = f"theta={theta:.2f} deg, b={b:.1f}, detected={detected}"
        cv2.putText(
            vis, status_text,
            (30, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0, status_color, 2
        )

        # ===== Draw Crosshairs =====
        center_x, center_y = w // 2, h // 2
        
        # Horizontal centerline
        cv2.line(vis, (0, center_y), (w, center_y), (255, 0, 0), 2)
        
        # Vertical centerline
        cv2.line(vis, (center_x, 0), (center_x, h), (0, 0, 255), 2)

        # ===== Display ===== 
        cv2.imshow(self.window_name, vis)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):  # ESC or 'q'
            self.get_logger().info("Quit key pressed, shutting down...")
            rclpy.shutdown()

    # ===================== Cleanup =====================

    def destroy_node(self) -> None:
        """Clean up visualization and background logging resources."""
        try:
            self._log_queue.put(None)
            self._log_thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            if self.show_window:
                cv2.destroyWindow(self.window_name)
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    try:
        node = LaneDetectionNode()
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

