#!/usr/bin/env python3
"""
camera_recorder_node.py
Workspace:  ws_jetson  |  Package: vision_navigation  |  Domain: 6

Field-Run Camera Recorder

Purpose:
    Records the RGB camera stream to a raw (uncompressed) video file for
    offline debugging after a field run -- e.g. "why did lane detection lose
    the lane at 14:32?" is much faster to answer by scrubbing footage than by
    reading CSV numbers.

    Deliberately a separate node from camera_stream_node and lane_detection_node:
    - camera_stream_node is upstream of the entire steering control loop; a bug
      in the recording path must not be able to affect frame capture/publish.
    - lane_detection_node is CPU-bound on frame processing every cycle. Video
      encoding is also CPU-bound (unlike the tiny CSV row writes both nodes
      already do asynchronously), so backgrounding it inside that same process
      still risks contending for CPU/GIL time with the actual detection loop.
      Recording as its own process avoids that risk entirely.

Topics Subscribed:
    /tpc_rover_d415_rgb (sensor_msgs/Image, bgr8) - RGB camera stream

Parameters:
    enabled (bool, default: True) - Master on/off switch
    record_fps (float, default: 10.0) - Target recording rate; incoming frames
        faster than this are throttled (dropped), not written twice
    record_width (int, default: 640) - Output frame width (frames are resized
        down from the camera's native resolution before writing)
    record_height (int, default: 360) - Output frame height
    low_space_warn_mb (int, default: 2048) - Log a warning once free disk
        space on the log filesystem drops below this

Storage:
    Saves to '~/almondmatcha/runs/logs/ws_jetson_camera_TIMESTAMP.avi',
    alongside lane_detection_node's and rover_kinematic_control's CSVs for the
    same run (same directory, same timestamp convention -- correlate by
    filename). Raw/uncompressed (no encode step) to keep CPU low, at the cost
    of large files -- default 640x360 @ 10 FPS is ~25 GB/hour; the Jetson's
    eMMC is 128 GB total, shared with everything else, so don't raise
    record_fps/resolution without checking that math again.

    NOTE: the exact FOURCC used for "uncompressed" output depends on the
    OpenCV/FFmpeg build installed on the actual Jetson and could not be
    verified on dev hardware without a camera attached -- confirm the output
    file actually plays back correctly the first time this runs in the field,
    and adjust `_FOURCC` below if the codec isn't available.

Writes happen on a background thread via a queue, same pattern as the CSV
loggers in this package, so the image callback never blocks on disk I/O.

Author: Vision Navigation System
Date: July 27, 2026
"""

import os
import queue
import shutil
import signal
import threading
import time

import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Uncompressed AVI (RGB, no inter-frame prediction) -- see module docstring note
# about confirming this codec is actually available on the Jetson's OpenCV build.
_FOURCC = cv2.VideoWriter_fourcc(*'DIB ')


class CameraRecorderNode(Node):
    """
    Subscribes to the RGB camera stream and writes a throttled, downscaled,
    raw video file per run -- purely for offline debugging, not part of the
    control loop.
    """

    def __init__(self) -> None:
        super().__init__('camera_recorder_node')

        # ===================== Parameters =====================
        self.declare_parameter('enabled', True)
        self.declare_parameter('record_fps', 10.0)
        self.declare_parameter('record_width', 640)
        self.declare_parameter('record_height', 360)
        self.declare_parameter('low_space_warn_mb', 2048)

        self.enabled: bool = bool(self.get_parameter('enabled').value)
        self.record_fps: float = float(self.get_parameter('record_fps').value)
        self.record_width: int = int(self.get_parameter('record_width').value)
        self.record_height: int = int(self.get_parameter('record_height').value)
        self.low_space_warn_mb: int = int(self.get_parameter('low_space_warn_mb').value)

        if not self.enabled:
            self.get_logger().info("camera_recorder_node: disabled via 'enabled' param, idling")
            return

        # ===================== QoS (match camera_stream_node's publisher) =====================
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        self.sub_rgb = self.create_subscription(
            Image,
            'tpc_rover_d415_rgb',
            self._on_rgb_frame,
            qos
        )

        # ===================== Utilities =====================
        self.bridge = CvBridge()

        # ===================== Output file (alongside the run's CSVs) =====================
        log_dir = os.path.expanduser("~/almondmatcha/runs/logs")
        os.makedirs(log_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.video_path: str = os.path.join(log_dir, f"ws_jetson_camera_{timestamp}.avi")

        self.writer = cv2.VideoWriter(
            self.video_path, _FOURCC, self.record_fps,
            (self.record_width, self.record_height)
        )
        if not self.writer.isOpened():
            self.get_logger().error(
                f"Failed to open video writer at {self.video_path} -- recording disabled "
                f"for this run. Check that FOURCC 'DIB ' is supported by this OpenCV build."
            )
            self.writer = None

        # ===================== Frame-rate throttle state =====================
        self._min_frame_interval_sec: float = 1.0 / self.record_fps if self.record_fps > 0 else 0.0
        self._last_written_time: float = 0.0
        self._frames_written: int = 0
        self._frames_dropped_throttle: int = 0

        # ===================== Disk-space check state =====================
        self._low_space_warned: bool = False
        self._space_check_every: int = 50  # ~5s at 10 FPS
        self._frames_since_space_check: int = 0

        # ===================== Async video writing =====================
        # Same pattern as the CSV loggers in this package: the write itself
        # happens on a background thread so a slow eMMC never blocks/drops
        # the incoming image callback.
        self._write_queue: "queue.Queue" = queue.Queue()
        self._write_thread = threading.Thread(target=self._write_worker, daemon=True)
        self._write_thread.start()

        self.get_logger().info(
            f"Camera recorder initialized. Recording to: {self.video_path} "
            f"({self.record_width}x{self.record_height} @ {self.record_fps} FPS, raw/uncompressed)"
        )

    # ===================== Subscription Callback =====================

    def _on_rgb_frame(self, msg: Image) -> None:
        """Throttle to record_fps, resize, and enqueue the frame for writing."""
        if self.writer is None:
            return

        now = time.monotonic()
        if now - self._last_written_time < self._min_frame_interval_sec:
            self._frames_dropped_throttle += 1
            return
        self._last_written_time = now

        try:
            bgr_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"camera_recorder: failed to convert image: {e}")
            return

        resized = cv2.resize(bgr_frame, (self.record_width, self.record_height))
        # .copy() not needed here -- cv2.resize always allocates a new array,
        # so this is already safe to hand across the thread boundary.
        self._write_queue.put(resized)

        self._frames_since_space_check += 1
        if self._frames_since_space_check >= self._space_check_every:
            self._frames_since_space_check = 0
            self._check_disk_space()

    def _check_disk_space(self) -> None:
        """Log once (not repeatedly) if free space on the log filesystem gets low."""
        if self._low_space_warned:
            return
        try:
            free_mb = shutil.disk_usage(os.path.dirname(self.video_path)).free / (1024 * 1024)
        except OSError:
            return
        if free_mb < self.low_space_warn_mb:
            self._low_space_warned = True
            self.get_logger().warn(
                f"camera_recorder: free disk space low ({free_mb:.0f} MB) -- "
                f"recording will keep running but may fail or fill the disk soon"
            )

    # ===================== Background Write Thread =====================

    def _write_worker(self) -> None:
        """Drains the write queue and appends frames to the video file."""
        while True:
            frame = self._write_queue.get()
            if frame is None:
                self._write_queue.task_done()
                break
            try:
                self.writer.write(frame)
                self._frames_written += 1
            except Exception as e:
                self.get_logger().warn(f"camera_recorder: frame write failed: {e}")
            finally:
                self._write_queue.task_done()

    # ===================== Cleanup =====================

    def destroy_node(self) -> None:
        """Drain the write queue and release the video file cleanly."""
        if self.enabled and self.writer is not None:
            try:
                self._write_queue.put(None)
                self._write_thread.join(timeout=5.0)
            except Exception:
                pass
            try:
                self.writer.release()
                self.get_logger().info(
                    f"camera_recorder: closed {self.video_path} "
                    f"({self._frames_written} frames written, "
                    f"{self._frames_dropped_throttle} throttled)"
                )
            except Exception:
                pass
        super().destroy_node()


def _raise_keyboard_interrupt(signum, frame) -> None:
    """
    Signal handler that routes SIGTERM/SIGHUP through the same shutdown path
    as Ctrl-C (SIGINT).

    Python only auto-converts SIGINT into a catchable KeyboardInterrupt --
    SIGTERM (sent by `pkill -TERM`, e.g. launch_field.sh's emergency stop)
    and SIGHUP (sent when `tmux kill-session` closes the pane's pty) both
    default to killing the process immediately, bypassing main()'s
    try/finally entirely. That skips destroy_node()'s writer.release(),
    which leaves the AVI without a valid trailer/index -- corrupting the
    whole file rather than just losing the last few frames.
    """
    raise KeyboardInterrupt


def main() -> None:
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    signal.signal(signal.SIGHUP, _raise_keyboard_interrupt)

    rclpy.init()
    node = None
    try:
        node = CameraRecorderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Unlike the CSV loggers elsewhere in this package, explicitly release
        # the video writer here rather than relying on process exit: an
        # un-released cv2.VideoWriter can leave the AVI without a valid
        # trailer/index, corrupting the *entire* file rather than just
        # losing the last few buffered rows the way an unflushed CSV would.
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
