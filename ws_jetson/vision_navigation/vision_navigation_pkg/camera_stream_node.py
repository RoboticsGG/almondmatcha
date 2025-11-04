#!/usr/bin/env python3
"""
Visual Navigation Camera Stream Node

Purpose:
    Streams RGB and optional depth data from Intel RealSense D415 camera or video file.
    Publishes camera frames as ROS2 sensor_msgs/Image topics.

Topics Published:
    /tpc_rover_d415_rgb (sensor_msgs/Image, bgr8) - RGB video stream
    /tpc_rover_d415_depth (sensor_msgs/Image, 16UC1) - Depth data (if enabled)

Parameters:
    width (int, default: 1280) - Frame width in pixels
    height (int, default: 720) - Frame height in pixels
    fps (int, default: 30) - Frames per second
    json_config (str, default: "") - Path to RealSense advanced mode JSON configuration
    open_cam (bool, default: False) - Display camera feed in OpenCV window (GUI mode)
    enable_depth (bool, default: False) - Stream depth frames (D415 mode only)
    video_path (str, default: "") - Path to video file (if set, uses video instead of D415)
    loop_video (bool, default: True) - Loop video when it reaches the end

Usage:
    # Stream from D415 camera (headless mode)
    ros2 run vision_navigation camera_stream

    # Stream with GUI preview (when monitor is connected)
    ros2 run vision_navigation camera_stream --ros-args -p open_cam:=True

    # Stream from video file with looping
    ros2 run vision_navigation camera_stream --ros-args \\
      -p video_path:="/path/to/video.mp4" -p open_cam:=True

    # Stream D415 with depth and advanced configuration
    ros2 run vision_navigation camera_stream --ros-args \\
      -p enable_depth:=True \\
      -p json_config:="/path/to/config.json"

Author: Vision Navigation System
Date: November 4, 2025
"""

import os
import time
from typing import Optional, Tuple
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


class D415StreamNode(Node):
    """
    Intel RealSense D415 Camera/Video Stream Node
    
    Supports two modes:
    1. D415 Camera Mode: Streams directly from D415 USB camera with optional depth
    2. Video File Mode: Streams from MP4/AVI video file for testing/replay
    """

    def __init__(self) -> None:
        super().__init__('node_cam1_d415_stream')

        # ===================== QoS Configuration =====================
        # Sensor data: low latency, best effort delivery
        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.history = QoSHistoryPolicy.KEEP_LAST
        qos.durability = QoSDurabilityPolicy.VOLATILE

        # ===================== Publishers =====================
        self.pub_rgb = self.create_publisher(Image, 'tpc_rover_d415_rgb', qos)
        self.pub_depth = self.create_publisher(Image, 'tpc_rover_d415_depth', qos)

        # ===================== Utilities =====================
        self.bridge = CvBridge()

        # ===================== Parameter Declaration =====================
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('json_config', '')
        self.declare_parameter('open_cam', False)
        self.declare_parameter('enable_depth', False)
        self.declare_parameter('video_path', '')
        self.declare_parameter('loop_video', True)

        # ===================== Parameter Retrieval =====================
        self.w: int = int(self.get_parameter('width').value)
        self.h: int = int(self.get_parameter('height').value)
        self.fps_param: int = int(self.get_parameter('fps').value)
        self.json_path: str = self.get_parameter('json_config').value
        self.open_cam: bool = bool(self.get_parameter('open_cam').value)
        self.enable_depth: bool = bool(self.get_parameter('enable_depth').value)
        self.video_path: str = str(self.get_parameter('video_path').value).strip()
        self.loop_video: bool = bool(self.get_parameter('loop_video').value)

        # ===================== Mode Selection =====================
        self.use_video: bool = len(self.video_path) > 0
        self.warned_depth_ignored: bool = False

        # ===================== Camera/Video Initialization =====================
        self.pipeline: Optional[object] = None
        self.cap: Optional[cv2.VideoCapture] = None
        self.align: Optional[object] = None
        self.fps_use: int = 1

        if self.use_video:
            self._init_video_mode()
        else:
            self._init_d415_mode()

        # ===================== Timer Setup =====================
        self.timer = self.create_timer(1.0 / max(1, self.fps_use), self._on_timer)

        self.get_logger().info(f"Camera stream node initialized. FPS: {self.fps_use}")

    # ===================== Initialization Methods =====================

    def _init_video_mode(self) -> None:
        """Initialize video file playback mode."""
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open video file: {self.video_path}")

        # Use video FPS if available, otherwise use parameter or default to 30
        fps_file = self.cap.get(cv2.CAP_PROP_FPS)
        if fps_file is None or fps_file <= 0 or np.isnan(fps_file):
            self.fps_use = max(1, self.fps_param)
        else:
            self.fps_use = int(round(fps_file)) if self.fps_param <= 0 else self.fps_param

        self.get_logger().info(
            f"Video Mode: '{os.path.basename(self.video_path)}' | FPS: {self.fps_use} | Loop: {self.loop_video}"
        )

    def _init_d415_mode(self) -> None:
        """Initialize Intel RealSense D415 camera mode."""
        if rs is None:
            raise RuntimeError(
                "pyrealsense2 library not found. Install with: "
                "pip install pyrealsense2 or use video_path parameter for file mode."
            )

        # ===== Pipeline & Configuration =====
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Enable specific D415 device by serial number
        self.config.enable_device('806312060441')

        # Stream configuration: RGB
        self.config.enable_stream(
            rs.stream.color, 
            self.w, self.h, 
            rs.format.bgr8, 
            max(1, self.fps_param)
        )

        # Stream configuration: Depth (optional)
        if self.enable_depth:
            self.config.enable_stream(
                rs.stream.depth,
                self.w, self.h,
                rs.format.z16,
                max(1, self.fps_param)
            )

        # Start pipeline
        self.profile = self.pipeline.start(self.config)

        # ===== Optional Advanced Configuration =====
        if self.json_path and os.path.exists(self.json_path):
            self._load_advanced_config(self.json_path)

        # ===== Alignment Setup =====
        self.align = rs.align(rs.stream.color) if self.enable_depth else None
        self.fps_use = max(1, self.fps_param)

        mode_str = "RGB + Depth" if self.enable_depth else "RGB"
        self.get_logger().info(f"D415 Mode: {mode_str} @ {self.fps_use} FPS")

    def _load_advanced_config(self, json_path: str) -> None:
        """Load advanced configuration from JSON file."""
        try:
            dev = self.profile.get_device()
            adv = rs.rs400_advanced_mode(dev)
            if not adv.is_enabled():
                adv.toggle_advanced_mode(True)
                self.get_logger().info('Advanced mode enabled...')
                time.sleep(2)
            
            with open(json_path, 'r') as f:
                json_str = f.read()
            adv.load_json(json_str)
            self.get_logger().info(f'Advanced config loaded: {os.path.basename(json_path)}')
        except Exception as e:
            self.get_logger().warn(f'Failed to load advanced config: {e}')

    # ===================== Timer Callback =====================

    def _on_timer(self) -> None:
        """Timer callback: stream frames based on mode."""
        if self.use_video:
            self._stream_video_frame()
        else:
            self._stream_d415_frame()

    # ===================== Video Mode Frame Streaming =====================

    def _stream_video_frame(self) -> None:
        """Capture and publish frame from video file."""
        # Warn if depth was requested but unavailable in video mode
        if self.enable_depth and not self.warned_depth_ignored:
            self.get_logger().warn(
                "Depth streaming not available in video file mode (RGB only)"
            )
            self.warned_depth_ignored = True

        # Read frame from video
        ret, frame = self.cap.read()
        if not ret:
            if self.loop_video:
                # Rewind to beginning
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().error("Failed to read frame after rewinding video")
                    self._shutdown()
                    return
            else:
                self.get_logger().info("Video playback complete (loop_video=False)")
                self._shutdown()
                return

        # Resize if necessary
        if self.w > 0 and self.h > 0 and (frame.shape[1] != self.w or frame.shape[0] != self.h):
            frame = cv2.resize(frame, (self.w, self.h), interpolation=cv2.INTER_LINEAR)

        # Publish RGB frame
        self._publish_rgb_frame(frame, 'file_color_frame')

    def _publish_rgb_frame(self, frame: np.ndarray, frame_id: str) -> None:
        """Convert and publish RGB frame."""
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        self.pub_rgb.publish(msg)
        
        # Display frame if GUI mode enabled
        if self.open_cam:
            cv2.imshow('Camera Stream (RGB)', frame)
            cv2.waitKey(1)

    # ===================== D415 Mode Frame Streaming =====================

    def _stream_d415_frame(self) -> None:
        """Capture and publish frames from D415 camera."""
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=100)
        except Exception:
            return

        if not frames:
            return

        # Align depth to color frame
        if self.align is not None:
            frames = self.align.process(frames)

        # Extract frames
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame() if self.align is not None else None

        # Publish RGB
        if color_frame:
            color_img = np.asanyarray(color_frame.get_data())
            self._publish_rgb_frame(color_img, 'd415_color_optical_frame')

        # Publish Depth
        if depth_frame:
            depth_img = np.asanyarray(depth_frame.get_data())
            msg_depth = self.bridge.cv2_to_imgmsg(depth_img, encoding='16UC1')
            msg_depth.header.stamp = self.get_clock().now().to_msg()
            msg_depth.header.frame_id = 'd415_depth_optical_frame'
            self.pub_depth.publish(msg_depth)
            
            # Display depth frame if GUI mode enabled
            if self.open_cam:
                # Normalize depth for visualization
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_img, alpha=0.03), 
                    cv2.COLORMAP_JET
                )
                cv2.imshow('Camera Stream (Depth)', depth_colormap)
                cv2.waitKey(1)

    # ===================== Cleanup Methods =====================

    def _shutdown(self) -> None:
        """Request graceful shutdown."""
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    def destroy_node(self) -> None:
        """Clean up resources."""
        try:
            if self.pipeline is not None:
                self.pipeline.stop()
        except Exception:
            pass

        try:
            if self.use_video and self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        
        # Close all OpenCV windows if they were opened
        if self.open_cam:
            cv2.destroyAllWindows()

        super().destroy_node()


def main() -> None:
    rclpy.init()
    try:
        node = D415StreamNode()
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

