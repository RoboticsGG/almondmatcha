#!/usr/bin/env python3
"""
regenerate_roi.py
Workspace:  ws_jetson  |  Package: vision_navigation

Recompute the ROI trapezoid (config.LaneDetectionConfig.ROI_BASE_POINTS) and
the steering feedforward gain (config.ControlConfig.K_FF) from the physical
camera mount and chassis geometry.

Why this exists: both constants are derived from a hand-projection (see
docs/VISION_PIPELINE.md §0 and "Regenerating the ROI", and
docs/CONTROL_LAW.md §1.7) that was previously redone by hand with a
calculator every time the mount changed, then hand-typed into config.py and
both YAML files. That is exactly the kind of arithmetic that should not be
retyped from scratch each time -- this script is that math, runnable.

Pure functions, no ROS import, so it can run standalone or be unit tested
(matches the lane_detector.py convention: no ROS dependency in geometry code).

Usage:
    # Print the values for the currently-shipped mount geometry (the
    # defaults below ARE the shipped geometry -- keep them in sync with
    # config.py if the mount changes again):
    python3 regenerate_roi.py

    # Recompute for a different mount:
    python3 regenerate_roi.py --tilt-deg 18 --height-m 0.55

Output is meant to be copied directly into:
    - vision_navigation/config.py       (ROI_BASE_POINTS, K_FF)
    - config/vision_nav_gui.yaml        (roi_base_points)
    - config/vision_nav_headless.yaml   (roi_base_points)
    - rover_kinematic_control_params.yaml (k_ff, as a starting/reference value --
      it still ships at 0.0 by default; see docs/CONTROL_LAW.md §1.7 for why)
"""

import argparse
import math
from typing import Tuple

# ===================== Shipped geometry (defaults) =====================
# Keep these in sync with config.py's LaneDetectionConfig / ControlConfig
# and the physical rover. Update BOTH places together if the mount changes.

CAMERA_HEIGHT_M = 0.50          # Height above ground
CAMERA_TILT_DEG = 15.0          # Down from horizontal
CAMERA_AXLE_OFFSET_M = 0.08     # Behind the front axle (camera is on the
                                 # centerline laterally, so no X offset)
WHEELBASE_M = 0.4875            # Front to rear axle

CAPTURE_WIDTH_PX = 1280
CAPTURE_HEIGHT_PX = 720
# Datasheet FOV for the D415 colour stream at this resolution -- not read
# from the device (camera_stream_node never calls get_intrinsics(), see the
# "Intrinsics caveat" in docs/VISION_PIPELINE.md §0).
HFOV_DEG = 69.4
VFOV_DEG = 42.5

# Ground rectangle the ROI projects (design choice, independent of the mount
# angle -- see "Why the ROI starts 1.30 m out" in docs/VISION_PIPELINE.md §1).
# X is lateral (camera-centered), Z is forward from the camera.
ROI_X_HALF_WIDTH_M = 0.90
ROI_Z_NEAR_M = 1.30
ROI_Z_FAR_M = 3.00

BEV_PX_PER_M = 200.0


def intrinsics_from_fov(
    width_px: int, height_px: int, hfov_deg: float, vfov_deg: float
) -> Tuple[float, float, float, float]:
    """Pinhole intrinsics (fx, fy, cx, cy) from datasheet FOV. cx/cy assume
    the principal point is the frame center (no device calibration data)."""
    fx = (width_px / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    fy = (height_px / 2.0) / math.tan(math.radians(vfov_deg) / 2.0)
    return fx, fy, width_px / 2.0, height_px / 2.0


def ground_to_pixel(
    x_m: float, z_m: float, height_m: float, tilt_deg: float,
    fx: float, fy: float, cx: float, cy: float,
) -> Tuple[float, float]:
    """Project a ground point (X lateral, Z forward from camera) to a pixel
    (u, v). Inverse of pixel_to_ground() below. See docs/VISION_PIPELINE.md
    "Regenerating the ROI" for the derivation."""
    phi = math.radians(tilt_deg)
    denom = height_m * math.sin(phi) + z_m * math.cos(phi)
    u = cx + fx * x_m / denom
    v = cy + fy * (height_m * math.cos(phi) - z_m * math.sin(phi)) / denom
    return u, v


def pixel_to_ground(
    u: float, v: float, height_m: float, tilt_deg: float,
    fx: float, fy: float, cx: float, cy: float,
) -> Tuple[float, float]:
    """Project a pixel (u, v) to a ground point (X, Z). Inverse of
    ground_to_pixel(). Used here only to build the coverage table / verify
    round-tripping; the running node does not need this direction."""
    phi = math.radians(tilt_deg)
    dy = (v - cy) / fy * math.cos(phi) + math.sin(phi)
    dz = -(v - cy) / fy * math.sin(phi) + math.cos(phi)
    t = height_m / dy
    return t * (u - cx) / fx, t * dz


def horizon_row(height_m: float, tilt_deg: float, fy: float, cy: float) -> float:
    """Row where the ground plane vanishes to infinity (d_y = 0)."""
    return cy - fy * math.tan(math.radians(tilt_deg))


def compute_roi_points(
    height_m: float, tilt_deg: float, fx: float, fy: float, cx: float, cy: float,
    x_half_m: float, z_near_m: float, z_far_m: float,
) -> list:
    """Four ROI corners, bottom-left / bottom-right / top-right / top-left --
    the order lane_detection_node's get_scaled_roi_points() expects."""
    corners_ground = [
        (-x_half_m, z_near_m),  # bottom-left
        (x_half_m, z_near_m),   # bottom-right
        (x_half_m, z_far_m),    # top-right
        (-x_half_m, z_far_m),   # top-left
    ]
    return [
        ground_to_pixel(x, z, height_m, tilt_deg, fx, fy, cx, cy)
        for x, z in corners_ground
    ]


def compute_k_ff(wheelbase_m: float, px_per_m: float) -> float:
    """Feedforward gain: delta_deg = k_ff * curvature, derived from Ackermann
    geometry (delta = atan(L/R) =~ L/R for small angles) and the BEV's
    curvature-radius relation (A = 1/(2*R*S)). See docs/CONTROL_LAW.md §1.7."""
    return 2.0 * px_per_m * wheelbase_m * (180.0 / math.pi)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--height-m", type=float, default=CAMERA_HEIGHT_M)
    p.add_argument("--tilt-deg", type=float, default=CAMERA_TILT_DEG)
    p.add_argument("--axle-offset-m", type=float, default=CAMERA_AXLE_OFFSET_M,
                    help="Camera position behind the front axle")
    p.add_argument("--wheelbase-m", type=float, default=WHEELBASE_M)
    p.add_argument("--width-px", type=int, default=CAPTURE_WIDTH_PX)
    p.add_argument("--height-px", type=int, default=CAPTURE_HEIGHT_PX)
    p.add_argument("--hfov-deg", type=float, default=HFOV_DEG)
    p.add_argument("--vfov-deg", type=float, default=VFOV_DEG)
    p.add_argument("--roi-x-half-m", type=float, default=ROI_X_HALF_WIDTH_M)
    p.add_argument("--roi-z-near-m", type=float, default=ROI_Z_NEAR_M)
    p.add_argument("--roi-z-far-m", type=float, default=ROI_Z_FAR_M)
    p.add_argument("--px-per-m", type=float, default=BEV_PX_PER_M)
    args = p.parse_args()

    fx, fy, cx, cy = intrinsics_from_fov(args.width_px, args.height_px, args.hfov_deg, args.vfov_deg)

    corners = compute_roi_points(
        args.height_m, args.tilt_deg, fx, fy, cx, cy,
        args.roi_x_half_m, args.roi_z_near_m, args.roi_z_far_m,
    )
    labels = ["Bottom-left", "Bottom-right", "Top-right", "Top-left"]
    ground = [(-args.roi_x_half_m, args.roi_z_near_m), (args.roi_x_half_m, args.roi_z_near_m),
              (args.roi_x_half_m, args.roi_z_far_m), (-args.roi_x_half_m, args.roi_z_far_m)]

    k_ff = compute_k_ff(args.wheelbase_m, args.px_per_m)

    horizon = horizon_row(args.height_m, args.tilt_deg, fy, cy)

    print(f"Intrinsics (from {args.hfov_deg}° x {args.vfov_deg}° FOV @ "
          f"{args.width_px}x{args.height_px}): fx={fx:.1f} fy={fy:.1f} cx={cx:.0f} cy={cy:.0f}")
    print(f"Horizon row: {horizon:.0f} of {args.height_px}\n")

    print("ROI corners (ground -> pixel):")
    for label, (x, z), (u, v) in zip(labels, ground, corners):
        print(f"  {label:13s} X={x:+.2f} m, Z={z:.2f} m  ->  ({u:.0f}, {v:.0f})")

    flat = []
    for u, v in corners:
        flat += [round(u), round(v)]
    print(f"\nROI_BASE_POINTS (config.py, [[u,v], ...] order):")
    for (u, v) in corners:
        print(f"    [{round(u)}, {round(v)}],")
    print(f"\nroi_base_points (YAML, flat list): {flat}")

    print(f"\nAhead of front axle: near edge {args.roi_z_near_m - args.axle_offset_m:.2f} m, "
          f"far edge {args.roi_z_far_m - args.axle_offset_m:.2f} m")
    print(f"Ahead of rear axle:  near edge "
          f"{args.roi_z_near_m - args.axle_offset_m + args.wheelbase_m:.2f} m")

    print(f"\nk_ff = 2 * {args.px_per_m:.0f} * {args.wheelbase_m} * (180/pi) = {k_ff:.1f}")
    print("Ackermann cross-check (delta_ff = k_ff / (2*px_per_m*R) vs exact atan(L/R)):")
    for r in (10.0, 20.0, args.roi_z_far_m * 12.17):  # last one ~ old doc's 36.5 m reference bend
        approx = k_ff / (2.0 * args.px_per_m * r)
        exact = math.degrees(math.atan(args.wheelbase_m / r))
        print(f"  R={r:5.1f} m:  approx {approx:.2f} deg   exact {exact:.2f} deg")

    print("\nCoverage table (distance ahead of camera / full-FOV ground width) at "
          "bottom-of-frame and ROI edges:")
    for row_name, v in (("bottom of frame", args.height_px), ("ROI near edge", corners[0][1]),
                         ("ROI far edge", corners[2][1])):
        x_left, z = pixel_to_ground(0, v, args.height_m, args.tilt_deg, fx, fy, cx, cy)
        x_right, _ = pixel_to_ground(args.width_px, v, args.height_m, args.tilt_deg, fx, fy, cx, cy)
        print(f"  {row_name:16s} row {v:5.0f}:  {z:.2f} m ahead, "
              f"{x_right - x_left:.2f} m wide")


if __name__ == "__main__":
    main()
