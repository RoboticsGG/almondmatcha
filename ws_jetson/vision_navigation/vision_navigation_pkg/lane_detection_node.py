#!/usr/bin/env python3
"""
Visual Navigation Image Processing Node

Purpose:
    Processes RGB frames from camera to detect lane markers and compute
    navigation parameters (steering angle, lateral offset).

Topics Subscribed:
    /tpc_rover_d415_rgb (sensor_msgs/Image, bgr8) - RGB camera stream

Topics Published:
    /tpc_rover_nav_lane (std_msgs/Float32MultiArray) - Lane parameters [theta, b, detected]
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
    5. Lane detection using polyfit
    6. Parameter calculation (theta, b)
    7. Publish and log results

CSV Logging:
    Saves detection results to '~/almondmatcha/runs/logs/ws_jetson_lane_detection_TIMESTAMP.csv' with:
    timestamp, theta, b, detected

Author: Vision Navigation System
Date: November 4, 2025
"""

import math
import csv
import os
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

from vision_navigation_pkg.lane_detector import process_frame


class NavProcessNode(Node):
    """
    Navigation image processing node using lane detection.
    
    Receives RGB frames, detects lane boundaries, and publishes
    lane parameters for rover steering control.
    """

    def __init__(self) -> None:
        super().__init__('node_cam1_nav_process')

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
            '/tpc_rover_d415_rgb', 
            self._on_rgb_frame,
            qos
        )

        # ===================== Publishers =====================
        self.pub_lane = self.create_publisher(
            Float32MultiArray, 
            '/tpc_rover_nav_lane', 
            10
        )

        # ===================== Utilities =====================
        self.bridge = CvBridge()

        # ===================== Parameters =====================
        self.declare_parameter('show_window', False)
        self.show_window: bool = bool(self.get_parameter('show_window').value)

        # ===================== Visualization =====================
        self.window_name = "lane_detection"
        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 1280, 720)

        # ===================== State =====================
        self.frame_count: int = 0
        # Centralized logging in runs/logs/ directory
        import time
        log_dir = os.path.expanduser("~/almondmatcha/runs/logs")
        os.makedirs(log_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"ws_jetson_lane_detection_{timestamp}.csv"
        self.csv_path: str = os.path.join(log_dir, filename)
        self._csv_header_written: bool = False

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
        theta, b, detected = process_frame(bgr_frame)

        # ===================== Value Validation =====================
        # Handle NaN and None values
        theta_clean = self._validate_float(theta, 0.0)
        b_clean = self._validate_float(b, float('nan'))
        detected_val = 1.0 if detected else 0.0

        # ===================== Publish Results =====================
        lane_msg = Float32MultiArray()
        lane_msg.data = [theta_clean, b_clean, detected_val]
        self.pub_lane.publish(lane_msg)

        # ===================== Logging =====================
        self._log_to_csv(theta_clean, b_clean, detected_val)
        self._log_to_terminal(theta_clean, b_clean, detected_val)

        # ===================== Visualization =====================
        if self.show_window:
            self._visualize_frame(bgr_frame, theta_clean, b_clean, detected)

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

    def _log_to_csv(self, theta: float, b: float, detected: float) -> None:
        """
        Log frame results to CSV file.
        
        Args:
            theta: Steering angle (degrees)
            b: Lateral offset
            detected: Detection flag (1.0 or 0.0)
        """
        try:
            timestamp = datetime.now().isoformat()
            row = [timestamp, theta, b, detected]
            
            file_exists = os.path.isfile(self.csv_path)
            with open(self.csv_path, 'a', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                if not file_exists and not self._csv_header_written:
                    writer.writerow(['timestamp', 'theta', 'b', 'detected'])
                    self._csv_header_written = True
                writer.writerow(row)
        except Exception as e:
            self.get_logger().warn(f"CSV logging failed: {e}")

    def _log_to_terminal(self, theta: float, b: float, detected: float) -> None:
        """
        Log frame results to terminal.
        
        Args:
            theta: Steering angle (degrees)
            b: Lateral offset
            detected: Detection flag (1.0 or 0.0)
        """
        self.frame_count += 1
        status = "Detected" if detected > 0.5 else "Not Detected"
        self.get_logger().info(
            f"Frame {self.frame_count}: theta={theta:7.3f} deg, "
            f"b={b:7.3f}, status={status}"
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
        """Clean up visualization resources."""
        try:
            if self.show_window:
                cv2.destroyWindow(self.window_name)
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    try:
        node = NavProcessNode()
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

