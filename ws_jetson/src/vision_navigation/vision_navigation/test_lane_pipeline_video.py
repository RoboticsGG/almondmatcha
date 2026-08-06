#!/usr/bin/env python3
"""
test_lane_pipeline_video.py
Workspace:  ws_jetson  |  Package: vision_navigation

Offline validation harness for the shipped lane detection pipeline
(lane_detector.py) against a recorded video file.

Purpose:
    Runs the exact same per-frame pipeline lane_detection_node.py runs live
    (crop -> segment colors -> warp to BEV -> shape-filter -> sliding-window
    fit -> compute_lane_params), including the node's cross-frame
    search_center tracking and the implausible-lock rejection
    (MAX_ABS_B_M), against a video file instead of a live camera topic. This
    is a read-only, non-ROS harness for validating the algorithm against
    real D415 footage -- it does not modify or retune the pipeline itself.

Outputs:
    1. A CSV with one row per processed frame: timestamp_sec, frame_idx,
       curvature, theta_deg, b_m, detected -- the same fields
       lane_detection_node publishes/logs.
    2. An overlay video: the original frame with the ROI trapezoid and the
       back-projected fit line drawn on it (matching plot_lane_lines'
       geometry), so detections can be eyeballed against the real track.

Standalone script, not a ROS node -- no rclpy required.

Usage:
    cd ~/almondmatcha/ws_jetson/src/vision_navigation/vision_navigation
    python3 test_lane_pipeline_video.py --video-path /path/to/capture.mp4 \\
        --out-csv /path/to/out.csv --out-video /path/to/out_overlay.mp4
"""

import argparse
import csv
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision_navigation.config import LaneDetectionConfig
from vision_navigation.lane_detector import (
    get_scaled_roi_points,
    crop_to_roi,
    segment_track_colors,
    perspective_transform,
    filter_line_candidates_bev,
    compute_lane_params,
)


def draw_overlay(frame_bgr, roi_points_full, M_inv, x_offset, y_offset,
                  bev_width, bev_height, theta, b, detected):
    """Original frame with the ROI trapezoid and the back-projected fit line
    drawn on it. Mirrors lane_detector.plot_lane_lines' fit-line geometry
    (x = tan(theta)*y + (b_px + bev_width/2) in the BEV canvas), but
    back-projects through M_inv into the ORIGINAL frame's coordinates
    instead of the crop, and actually returns the image for saving --
    plot_lane_lines builds a visualization but never returns or persists it."""
    vis = frame_bgr.copy()

    pts = roi_points_full.astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(vis, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

    if detected and np.isfinite(theta) and np.isfinite(b):
        b_px = b * LaneDetectionConfig.BEV_PX_PER_M
        slope = np.tan(np.radians(theta))
        y_vals = np.linspace(0, bev_height - 1, num=bev_height)
        x_vals = slope * y_vals + (b_px + bev_width / 2.0)

        pts_bev = np.vstack([x_vals, y_vals]).T.reshape(-1, 1, 2).astype(np.float32)
        pts_crop = cv2.perspectiveTransform(pts_bev, M_inv)
        pts_frame = pts_crop.reshape(-1, 2) + np.float32([x_offset, y_offset])
        cv2.polylines(vis, [pts_frame.astype(np.int32)], isClosed=False,
                       color=(0, 0, 255), thickness=3)

    status_color = (0, 255, 0) if detected else (0, 0, 255)
    status_text = f"theta={theta:6.2f} deg  b={b:6.3f} m  detected={detected}"
    cv2.putText(vis, status_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, status_color, 2, cv2.LINE_AA)

    return vis


def _segment_path(out_video, segment_idx):
    """out_video='dev/foo.mp4', segment_idx=2 -> 'dev/foo_part02.mp4'."""
    base, ext = os.path.splitext(out_video)
    return f"{base}_part{segment_idx:02d}{ext}"


def process_video(video_path, out_csv, out_video, frame_stride=1, max_frames=None,
                   progress_every=300, segment_seconds=None):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {video_path}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    roi_base_points = LaneDetectionConfig.ROI_BASE_POINTS
    roi_base_width = LaneDetectionConfig.ROI_BASE_WIDTH
    roi_base_height = LaneDetectionConfig.ROI_BASE_HEIGHT
    crop_margin_px = LaneDetectionConfig.CROP_MARGIN_PX
    bev_width = LaneDetectionConfig.BEV_WIDTH_PX
    bev_height = LaneDetectionConfig.BEV_HEIGHT_PX

    roi_points_full = get_scaled_roi_points(
        frame_width, frame_height, roi_base_points, roi_base_width, roi_base_height
    )
    margin_x = crop_margin_px * (frame_width / roi_base_width)
    margin_y = crop_margin_px * (frame_height / roi_base_height)

    os.makedirs(os.path.dirname(out_csv) or ".", exist_ok=True)
    os.makedirs(os.path.dirname(out_video) or ".", exist_ok=True)

    out_fps = fps / max(1, frame_stride)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")

    # Segmented output: a long round-trip recording is unwieldy to scrub
    # through as one file, and -- as this run itself hit once already --
    # a single multi-hundred-MB in-progress mp4 has no valid moov atom
    # (and is therefore unplayable) if the process dies before
    # writer.release(). Segmenting bounds how much of a segment in flight
    # is lost to that, same rationale as the CSV being appended row-by-row
    # rather than written once at the end.
    frames_per_segment = (
        int(round(segment_seconds * out_fps)) if segment_seconds else None
    )
    segment_idx = 1
    frames_in_segment = 0
    current_out_path = _segment_path(out_video, segment_idx) if frames_per_segment else out_video
    writer = cv2.VideoWriter(current_out_path, fourcc, out_fps, (frame_width, frame_height))
    if not writer.isOpened():
        raise RuntimeError(f"Cannot open VideoWriter for: {current_out_path}")
    written_paths = [current_out_path]

    csv_file = open(out_csv, "w", newline="", encoding="utf-8")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["timestamp_sec", "frame_idx", "curvature", "theta_deg", "b_m", "detected"])

    # Mirrors lane_detection_node.py's cross-frame tracking state exactly --
    # see that file's _search_center for why (keeps the search locked onto
    # the line already being followed instead of re-picking the strongest
    # of several parallel painted lines every frame).
    search_center = None

    n_detected = 0
    n_processed = 0
    frame_idx = 0
    t_start = time.monotonic()

    while True:
        ok, frame_bgr = cap.read()
        if not ok:
            break

        if frame_idx % frame_stride == 0:
            cropped_frame, x_offset, y_offset = crop_to_roi(frame_bgr, roi_points_full, margin_x, margin_y)
            crop_width, crop_height = cropped_frame.shape[1], cropped_frame.shape[0]
            roi_points_cropped = roi_points_full - np.float32([x_offset, y_offset])

            _red_track_mask, white_candidates = segment_track_colors(cropped_frame)
            warped_white, M, M_inv = perspective_transform(
                white_candidates, (crop_width, crop_height), roi_points_cropped,
                bev_size=(bev_width, bev_height)
            )
            warped = filter_line_candidates_bev(warped_white, bev_height=bev_height)
            params = compute_lane_params(
                warped,
                sliding_windows=LaneDetectionConfig.SLIDING_WINDOWS,
                window_margin=LaneDetectionConfig.WINDOW_MARGIN,
                min_window_pixels=LaneDetectionConfig.MIN_WINDOW_PIXELS,
                min_lane_pixels=LaneDetectionConfig.MIN_LANE_PIXELS,
                search_center=search_center,
                search_band=LaneDetectionConfig.SEARCH_BAND_PX,
            )
            curvature, theta, b, detected = (
                params["curvature"], params["theta"], params["b"], params["detected"]
            )

            # Same implausible-lock rejection as lane_detection_node._on_rgb_frame.
            if detected and np.isfinite(b) and abs(b) > LaneDetectionConfig.MAX_ABS_B_M:
                detected = False
                curvature = theta = b = float("nan")

            search_center = (
                (bev_width / 2.0) + (b * LaneDetectionConfig.BEV_PX_PER_M)
                if detected and np.isfinite(b) else None
            )

            timestamp_sec = frame_idx / fps
            csv_writer.writerow([f"{timestamp_sec:.4f}", frame_idx,
                                  curvature, theta, b, int(detected)])
            csv_file.flush()  # survive a kill mid-run with only the last row lost

            vis = draw_overlay(frame_bgr, roi_points_full, M_inv, x_offset, y_offset,
                                bev_width, bev_height, theta, b, detected)
            writer.write(vis)
            frames_in_segment += 1

            if frames_per_segment and frames_in_segment >= frames_per_segment:
                writer.release()  # finalizes the moov atom so this segment is playable now
                segment_idx += 1
                frames_in_segment = 0
                current_out_path = _segment_path(out_video, segment_idx)
                writer = cv2.VideoWriter(current_out_path, fourcc, out_fps, (frame_width, frame_height))
                if not writer.isOpened():
                    raise RuntimeError(f"Cannot open VideoWriter for: {current_out_path}")
                written_paths.append(current_out_path)

            n_processed += 1
            n_detected += int(detected)

            if n_processed % progress_every == 0:
                elapsed = time.monotonic() - t_start
                rate = n_processed / elapsed if elapsed > 0 else 0.0
                print(f"  frame {frame_idx}/{total_frames}  "
                      f"processed={n_processed}  detected={n_detected} "
                      f"({100.0 * n_detected / n_processed:.1f}%)  "
                      f"rate={rate:.1f} fps  elapsed={elapsed:.0f}s")

        frame_idx += 1
        if max_frames is not None and n_processed >= max_frames:
            break

    csv_file.close()
    writer.release()
    cap.release()

    elapsed = time.monotonic() - t_start
    print(f"\nDone: {video_path}")
    print(f"  Frames processed: {n_processed} / {total_frames} total in file")
    print(f"  Detected: {n_detected} ({100.0 * n_detected / max(1, n_processed):.1f}%)")
    print(f"  Elapsed: {elapsed:.1f}s ({n_processed / elapsed if elapsed > 0 else 0:.1f} fps)")
    print(f"  CSV:   {out_csv}")
    if len(written_paths) > 1:
        print(f"  Video: {len(written_paths)} segments:")
        for p in written_paths:
            print(f"    {p}")
    else:
        print(f"  Video: {written_paths[0]}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--video-path", required=True)
    p.add_argument("--out-csv", required=True)
    p.add_argument("--out-video", required=True)
    p.add_argument("--frame-stride", type=int, default=1,
                    help="Process every Nth frame (1 = every frame)")
    p.add_argument("--max-frames", type=int, default=None,
                    help="Stop after this many PROCESSED frames (for a quick sanity run)")
    p.add_argument("--segment-seconds", type=float, default=None,
                    help="Split the overlay video into multiple files of this many seconds "
                         "each (named <out-video-stem>_partNN<ext>), instead of one file")
    args = p.parse_args()

    process_video(args.video_path, args.out_csv, args.out_video,
                  frame_stride=args.frame_stride, max_frames=args.max_frames,
                  segment_seconds=args.segment_seconds)


if __name__ == "__main__":
    main()
