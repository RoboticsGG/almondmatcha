#!/usr/bin/env python3
"""
capture_d415_rgb.py

Standalone Windows utility -- captures the RGB stream from an Intel
RealSense D415 attached over USB and saves it to disk as an .mp4 file.

Not part of any ROS workspace; run directly with Python on the laptop
the camera is plugged into.

Requirements:
    pip install pyrealsense2 opencv-python

Usage:
    python capture_d415_rgb.py
    (press 'q' in the preview window, or Ctrl+C in the terminal, to stop)

Output:
    <this script's directory>\\videocapture\\capture_<YYYYmmdd_HHMMSS>.mp4
"""

import os
from datetime import datetime

import cv2
import numpy as np
import pyrealsense2 as rs

WIDTH = 1280
HEIGHT = 720
FPS = 30

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "videocapture")


def main() -> None:
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_path = os.path.join(OUTPUT_DIR, f"capture_{timestamp}.mp4")

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    pipeline.start(config)

    # mp4v is bundled with opencv-python's own FFmpeg build, so it writes an
    # .mp4 without needing extra codecs installed on the laptop.
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(output_path, fourcc, FPS, (WIDTH, HEIGHT))
    if not writer.isOpened():
        pipeline.stop()
        raise RuntimeError(f"Failed to open video writer at {output_path}")

    print(f"Recording to {output_path}  ({WIDTH}x{HEIGHT} @ {FPS} FPS)")
    print("Press 'q' in the preview window or Ctrl+C in this terminal to stop.")

    frame_count = 0
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            writer.write(frame)
            frame_count += 1

            cv2.imshow("D415 RGB (press 'q' to stop)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        print("\nStopping (Ctrl+C)...")
    finally:
        writer.release()
        pipeline.stop()
        cv2.destroyAllWindows()
        print(f"Saved {frame_count} frames to {output_path}")


if __name__ == "__main__":
    main()
