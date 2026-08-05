#!/usr/bin/env python3
"""
capture_roi_debug.py
Workspace:  ws_jetson  |  Package: vision_navigation

Headless ROI/BEV sanity check -- no display required.

Purpose:
    docs/VISION_PIPELINE.md's own "Known limits" section calls this out as
    the highest-value pre-run check: confirm the ROI trapezoid actually
    lands on the track, with all three painted lines inside it. Normally
    that's done with `show_window: true` and a monitor on the Jetson. This
    script does the same check without a display -- it grabs one frame,
    draws the ROI overlay and the warped bird's-eye view, and writes both to
    disk as PNGs. Copy them off (scp / VS Code remote / a file share) and
    look at them from wherever you do have a screen.

    Standalone script, not a ROS node -- no `ros2 run` / rclpy required, so
    it can't collide with a running camera_stream_node (RealSense only
    allows one pipeline per device) as long as nothing else is using the
    camera when this runs.

Usage (on the Jetson, over SSH):
    cd ~/almondmatcha/ws_jetson/src/vision_navigation/vision_navigation
    python3 capture_roi_debug.py

    # Custom output location, resolution, or a specific camera by serial:
    python3 capture_roi_debug.py --out-dir ~/roi_check --width 1280 --height 720

    # No D415 attached / testing off a recording instead:
    python3 capture_roi_debug.py --video-path /path/to/some_video.mp4

Then from your laptop:
    scp yupi@192.168.1.5:~/almondmatcha/ws_jetson/roi_debug_captures/*.png .
"""

import argparse
import os
import sys
from datetime import datetime

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision_navigation.config import CameraConfig, LaneDetectionConfig
from vision_navigation.lane_detector import (
    get_scaled_roi_points,
    crop_to_roi,
    preprocess_frame,
    perspective_transform,
)

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


def grab_frame_d415(width: int, height: int, fps: int, device_serial: str, warmup_frames: int) -> np.ndarray:
    """Open the D415, let auto-exposure settle, return one BGR frame. Mirrors
    camera_stream_node._init_d415_mode()'s connection logic, minus ROS."""
    if rs is None:
        raise RuntimeError(
            "pyrealsense2 not installed. Install it, or pass --video-path to "
            "capture from a recording instead."
        )

    pipeline = rs.pipeline()
    config = rs.config()
    if device_serial:
        config.enable_device(device_serial)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, max(1, fps))

    try:
        pipeline.start(config)
    except RuntimeError as e:
        connected = [d.get_info(rs.camera_info.serial_number) for d in rs.context().devices]
        raise RuntimeError(
            f"Failed to start RealSense pipeline: {e}. Connected devices: {connected or ['none']}"
        ) from e

    try:
        # Auto-exposure/white-balance need a few frames to converge; the
        # first frame or two off a cold start is often over/under-exposed.
        for _ in range(warmup_frames):
            pipeline.wait_for_frames(timeout_ms=2000)
        frames = pipeline.wait_for_frames(timeout_ms=2000)
        color = frames.get_color_frame()
        if not color:
            raise RuntimeError("Got a frameset but no color frame in it.")
        return np.asanyarray(color.get_data())
    finally:
        pipeline.stop()


def grab_frame_video(video_path: str) -> np.ndarray:
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video file: {video_path}")
    try:
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError(f"Could not read a frame from: {video_path}")
        return frame
    finally:
        cap.release()


def draw_roi_overlay(frame_bgr: np.ndarray, roi_points: np.ndarray) -> np.ndarray:
    """Raw frame with the ROI trapezoid drawn on it -- this is the frame to
    eyeball against the track: all three painted lines should fall inside
    the green quadrilateral."""
    overlay = frame_bgr.copy()
    pts = roi_points.astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(overlay, [pts], isClosed=True, color=(0, 255, 0), thickness=3)
    labels = ["BL", "BR", "TR", "TL"]
    for (x, y), label in zip(roi_points, labels):
        cv2.circle(overlay, (int(x), int(y)), 6, (0, 0, 255), -1)
        cv2.putText(overlay, label, (int(x) + 8, int(y) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    return overlay


def build_bev_debug(frame_bgr: np.ndarray, roi_points: np.ndarray) -> np.ndarray:
    """Crop -> preprocess -> warp, same pipeline lane_detection_node runs,
    so the operator can see whether the painted lines come out straight and
    parallel in the warped view (a correct rectification keeps them so)."""
    h, w = frame_bgr.shape[:2]
    cropped, x_off, y_off = crop_to_roi(
        frame_bgr, roi_points,
        margin_x=LaneDetectionConfig.CROP_MARGIN_PX * w / LaneDetectionConfig.ROI_BASE_WIDTH,
        margin_y=LaneDetectionConfig.CROP_MARGIN_PX * h / LaneDetectionConfig.ROI_BASE_HEIGHT,
    )
    roi_in_crop = roi_points - np.float32([x_off, y_off])
    binary = preprocess_frame(cropped)
    warped, _, _ = perspective_transform(
        binary, (cropped.shape[1], cropped.shape[0]), roi_in_crop,
        bev_size=(LaneDetectionConfig.BEV_WIDTH_PX, LaneDetectionConfig.BEV_HEIGHT_PX),
    )
    # Binary mask (0/1) -> viewable grayscale
    return (warped * 255).astype(np.uint8)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out-dir", default=os.path.expanduser("~/almondmatcha/ws_jetson/roi_debug_captures"))
    p.add_argument("--width", type=int, default=CameraConfig.WIDTH)
    p.add_argument("--height", type=int, default=CameraConfig.HEIGHT)
    p.add_argument("--fps", type=int, default=CameraConfig.FPS)
    p.add_argument("--device-serial", default=CameraConfig.D415_SERIAL)
    p.add_argument("--warmup-frames", type=int, default=15,
                    help="Frames to discard before capture, for auto-exposure to settle")
    p.add_argument("--video-path", default="",
                    help="Read one frame from a video file instead of the live D415")
    args = p.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    if args.video_path:
        print(f"Reading a frame from video: {args.video_path}")
        frame = grab_frame_video(args.video_path)
    else:
        print(f"Connecting to D415 (serial={args.device_serial or 'first available'}, "
              f"{args.width}x{args.height} @ {args.fps} FPS)...")
        frame = grab_frame_d415(args.width, args.height, args.fps, args.device_serial, args.warmup_frames)
        print("Captured 1 frame.")

    h, w = frame.shape[:2]
    roi_points = get_scaled_roi_points(
        w, h, LaneDetectionConfig.ROI_BASE_POINTS,
        LaneDetectionConfig.ROI_BASE_WIDTH, LaneDetectionConfig.ROI_BASE_HEIGHT,
    )

    overlay = draw_roi_overlay(frame, roi_points)
    bev = build_bev_debug(frame, roi_points)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_path = os.path.join(args.out_dir, f"roi_debug_{stamp}_raw_overlay.png")
    bev_path = os.path.join(args.out_dir, f"roi_debug_{stamp}_bev.png")
    cv2.imwrite(raw_path, overlay)
    cv2.imwrite(bev_path, bev)

    print(f"\nSaved:\n  {raw_path}\n  {bev_path}")
    print("\nWhat to check in the raw/overlay image: all three painted lines "
          "(two lane edges + centre line) should fall inside the green "
          "trapezoid, not cut off at an edge.")
    print("What to check in the BEV image: the lines should come out roughly "
          "straight and parallel (not fanning out/in) if the mount geometry "
          "and tilt used to derive the ROI match reality. See "
          "docs/VISION_PIPELINE.md \"Known limits\".")
    print(f"\nFrom your laptop: scp <user>@<jetson-ip>:{args.out_dir}/*.png .")


if __name__ == "__main__":
    main()
