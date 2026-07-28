"""
lane_detector.py
Workspace:  ws_jetson  |  Package: vision_navigation

Lane Detection Image Processing Pipeline

Provides image processing functions for detecting lane markers in RGB frames.
Includes preprocessing, perspective transformation, and lane fitting algorithms.

Author: AlmondMatcha Rover Team
Date: February 27, 2026
"""

import cv2
import numpy as np
from typing import Tuple
import matplotlib.pyplot as plt

from vision_navigation.config import LaneDetectionConfig

# ================================
# 1. Threshold + Preprocess
# ================================
def preprocess_frame(frame_bgr: np.ndarray) -> np.ndarray:
    """
    Preprocess frame: color filtering and edge detection.

    Removes green background (trees) via LAB color-space masking -- the
    track surface is red with white lane lines against green surroundings,
    so this segmentation is required for a clean edge signal, not optional.
    Gradient magnitude/direction use cv2.cartToPolar (replaces the slower
    np.sqrt / np.arctan2 calls), and small noise blobs are removed with a
    vectorized morphological opening instead of a per-contour Python loop.

    Args:
        frame_bgr: Input BGR image from camera

    Returns:
        Binary image with detected lane markers
    """
    # Convert to LAB color space for filtering. Operates directly on
    # frame_bgr -- the previous BGR->RGB blur/median-blur pass here was
    # computed but never consumed by the rest of the pipeline.
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
    A = lab[:, :, 1]
    B = lab[:, :, 2]

    # Remove green pixels (tree/background) from the LAB image
    green_mask = (A < 120) & (B > 130)
    lab_no_green = lab.copy()
    lab_no_green[green_mask] = 0

    # Convert back to RGB for gradient computation
    img_rgb_no_green = cv2.cvtColor(lab_no_green, cv2.COLOR_LAB2RGB)
    gray = cv2.cvtColor(img_rgb_no_green, cv2.COLOR_RGB2GRAY)

    # Sobel x, y (CV_32F required by cv2.cartToPolar)
    sobel_x = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    sobel_y = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    abs_sobel_x = np.absolute(sobel_x).astype(np.float32)
    abs_sobel_y = np.absolute(sobel_y).astype(np.float32)

    gradx = np.zeros_like(gray, dtype=np.uint8)
    grady = np.zeros_like(gray, dtype=np.uint8)
    gradx[(abs_sobel_x >= 50) & (abs_sobel_x <= 100)] = 1
    grady[(abs_sobel_y >= 50) & (abs_sobel_y <= 100)] = 1

    # Magnitude + direction in one optimized call (replaces np.sqrt / np.arctan2)
    mag_f, dir_f = cv2.cartToPolar(abs_sobel_x, abs_sobel_y, angleInDegrees=False)
    mag = cv2.normalize(mag_f, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    mag_binary = np.zeros_like(mag)
    mag_binary[(mag >= 30) & (mag <= 100)] = 1

    # Direction
    dir_binary = np.zeros_like(gray, dtype=np.uint8)
    dir_binary[(dir_f >= 0.7) & (dir_f <= 1.3)] = 1

    # White pixel detection (emphasize white lane lines)
    white_binary = np.zeros_like(gray, dtype=np.uint8)
    white_binary[gray > 180] = 1

    # Combined binary mask
    combined = np.zeros_like(gray, dtype=np.uint8)
    combined[((gradx == 1) & (grady == 1)) |
             ((mag_binary == 1) & (dir_binary == 1)) |
             (white_binary == 1)] = 1

    # Noise filtering: vectorized morphological opening (replaces the
    # per-contour Python loop). Small 3x3 kernel clears speckle noise
    # without eroding away thin lane-line strokes.
    noise_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, noise_kernel)

    return combined


# ================================
# 2. ROI Scaling & Cropping
# ================================
def get_scaled_roi_points(
    frame_width: int,
    frame_height: int,
    roi_base_points: np.ndarray,
    roi_base_width: float = LaneDetectionConfig.ROI_BASE_WIDTH,
    roi_base_height: float = LaneDetectionConfig.ROI_BASE_HEIGHT,
) -> np.ndarray:
    """
    Scale ROI corner points (authored at roi_base_width x roi_base_height) to
    the actual frame size.

    Args:
        frame_width: Actual frame width (pixels)
        frame_height: Actual frame height (pixels)
        roi_base_points: Array-like, shape (4, 2) -- ROI corners in base-resolution coordinates
        roi_base_width: Reference width roi_base_points were authored against
        roi_base_height: Reference height roi_base_points were authored against

    Returns:
        np.float32 array, shape (4, 2): ROI corners scaled to (frame_width, frame_height)
    """
    roi_base_points = np.float32(roi_base_points)
    sx = frame_width / roi_base_width
    sy = frame_height / roi_base_height
    return np.float32([[p[0] * sx, p[1] * sy] for p in roi_base_points])


def crop_to_roi(
    frame_bgr: np.ndarray,
    roi_points: np.ndarray,
    margin_x: float = 0.0,
    margin_y: float = 0.0,
) -> Tuple[np.ndarray, int, int]:
    """
    Crop a frame to the bounding box of the ROI polygon (plus margin).

    Runs before the expensive per-pixel preprocessing so CPU cost scales
    with the ROI area the perspective transform actually uses, rather than
    the full camera frame -- unlike a sensor-level resolution cut, this
    doesn't reduce pixel density (ground-sample-distance) inside the ROI,
    so it doesn't cost curve-fitting accuracy the way downscaling does.

    Args:
        frame_bgr: Input BGR frame
        roi_points: np.float32 array, shape (4, 2) -- ROI corners already
            scaled to this frame's size (see get_scaled_roi_points)
        margin_x: Extra margin added left/right of the ROI bounding box (pixels)
        margin_y: Extra margin added above/below the ROI bounding box (pixels)

    Returns:
        Tuple of (cropped_frame, x_offset, y_offset):
            - cropped_frame: BGR sub-image
            - x_offset, y_offset: top-left corner of the crop, in the
              original frame's coordinate system (subtract from roi_points
              to re-express them relative to the cropped frame)
    """
    frame_height, frame_width = frame_bgr.shape[:2]

    x_min = int(np.clip(np.min(roi_points[:, 0]) - margin_x, 0, frame_width))
    x_max = int(np.clip(np.max(roi_points[:, 0]) + margin_x, 0, frame_width))
    y_min = int(np.clip(np.min(roi_points[:, 1]) - margin_y, 0, frame_height))
    y_max = int(np.clip(np.max(roi_points[:, 1]) + margin_y, 0, frame_height))

    cropped_frame = frame_bgr[y_min:y_max, x_min:x_max]
    return cropped_frame, x_min, y_min


# ================================
# 3. Perspective Transform
# ================================
def perspective_transform(
    binary: np.ndarray,
    frame_size: Tuple[int, int],
    roi_points: np.ndarray,
    bev_size: Tuple[int, int] = None
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Apply perspective transformation to bird eye view.

    Transforms the image from camera view to top-down view for easier
    lane detection in the transformed coordinate system.

    The output canvas is `bev_size`, independent of the input crop size. This
    matters: sizing the canvas from the crop made the bird's-eye view
    anisotropic (the crop is much wider than it is tall), which inflated the
    reported heading angle by the canvas aspect ratio and left `theta` as a
    canvas-dependent number rather than an angle. With a fixed canvas chosen so
    that both axes carry the same metres-per-pixel, `theta` is a real heading
    angle, `b` is a real cross-track offset and `curvature` is a real 1/m arc.

    Args:
        binary: Binary input image
        frame_size: Tuple of (width, height) of `binary`
        roi_points: np.float32 array, shape (4, 2) -- source ROI quadrilateral
            corners [bottom-left, bottom-right, top-right, top-left], already
            scaled (and, if cropped upstream, shifted) to match `binary`
        bev_size: Tuple of (width, height) for the warped output canvas.
            Defaults to `frame_size`, preserving the previous behaviour.

    Returns:
        Tuple of (warped, M, M_inv):
            - warped: Transformed binary image (bird eye view)
            - M: Perspective transformation matrix
            - M_inv: Inverse transformation matrix
    """
    w, h = frame_size if bev_size is None else bev_size

    dst = np.float32([
        [w * 0.25, h * 1.0],
        [w * 0.75, h * 1.0],
        [w * 0.75, h * 0.0],
        [w * 0.25, h * 0.0]
    ])

    M = cv2.getPerspectiveTransform(roi_points, dst)
    M_inv = cv2.getPerspectiveTransform(dst, roi_points)

    warped = cv2.warpPerspective(binary, M, (w, h))

    return warped, M, M_inv


# ================================
# 4. Lane Finding (single center line)
# ================================
def find_center_line(
    binary_warped: np.ndarray,
    num_windows: int = 9,
    window_margin: int = 100,
    min_pixels: int = 50,
    search_center: float = None,
    search_band: float = None
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Find lane center line pixels using sliding window technique.

    The search starts from the strongest column of the bottom-half histogram,
    but only within `search_band` pixels of `search_center`. On a track with
    several parallel painted lines in view (e.g. two lane edges plus a centre
    line) an unrestricted argmax has no way to tell them apart -- it picks
    whichever line happens to carry the most pixels that frame, and that choice
    flips as lighting, wear or the rover's own drift changes the balance. Each
    flip is a step change in the reported offset of roughly one line spacing,
    which no downstream filter can absorb because it is a genuine step, not
    noise. Bounding the search keeps the detector on the line it was already
    following, and bounds how far that line can appear to move per frame.

    Args:
        binary_warped: Binary bird eye view image
        num_windows: Number of horizontal windows
        window_margin: Width of search window (plus-minus margin). Keep below
            half the spacing between adjacent painted lines, otherwise one
            window straddles two lines and recenters into the gap between them.
        min_pixels: Minimum pixels to recenter window
        search_center: Column the lane is expected at (typically the previous
            frame's result). Defaults to the canvas centre.
        search_band: Half-width of the band around `search_center` the initial
            histogram may search. None or <= 0 searches the full width (the
            previous, unbounded behaviour).

    Returns:
        Tuple of (x_coords, y_coords) of detected lane pixels
    """
    height, width = binary_warped.shape[:2]

    if search_center is None or not np.isfinite(search_center):
        search_center = width / 2.0
    search_center = float(np.clip(search_center, 0, width - 1))

    if search_band is None or search_band <= 0:
        band_low, band_high = 0, width
    else:
        band_low = int(max(0, np.floor(search_center - search_band)))
        band_high = int(min(width, np.ceil(search_center + search_band) + 1))
        if band_high <= band_low:            # degenerate band -- fall back
            band_low, band_high = 0, width

    histogram = np.sum(binary_warped[height // 2:, band_low:band_high], axis=0)
    if histogram.size == 0 or histogram.max() == 0:
        # Nothing in the band. Hold the expected position; the windows will
        # come back empty and min_lane_pixels will report "not detected",
        # rather than seeding the search from an arbitrary column.
        base_x = int(round(search_center))
    else:
        base_x = int(np.argmax(histogram)) + band_low

    window_height = binary_warped.shape[0] // num_windows
    nonzero = binary_warped.nonzero()
    nonzero_y, nonzero_x = np.array(nonzero[0]), np.array(nonzero[1])

    current_x = base_x
    lane_indices_list = []

    for window_idx in range(num_windows):
        win_y_low = binary_warped.shape[0] - (window_idx + 1) * window_height
        win_y_high = binary_warped.shape[0] - window_idx * window_height
        win_x_low = current_x - window_margin
        win_x_high = current_x + window_margin

        good_indices = (
            (nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
            (nonzero_x >= win_x_low) & (nonzero_x < win_x_high)
        ).nonzero()[0]
        lane_indices_list.append(good_indices)

        if len(good_indices) > min_pixels:
            current_x = int(np.mean(nonzero_x[good_indices]))

    lane_indices = np.concatenate(lane_indices_list) if lane_indices_list else np.array([], dtype=int)
    x_coords = nonzero_x[lane_indices]
    y_coords = nonzero_y[lane_indices]

    return x_coords, y_coords


# ================================
# 5. Fit single line & Params
# ================================
def compute_lane_params(
    binary_warped: np.ndarray,
    sliding_windows: int = LaneDetectionConfig.SLIDING_WINDOWS,
    window_margin: int = LaneDetectionConfig.WINDOW_MARGIN,
    min_window_pixels: int = LaneDetectionConfig.MIN_WINDOW_PIXELS,
    min_lane_pixels: int = LaneDetectionConfig.MIN_LANE_PIXELS,
    search_center: float = None,
    search_band: float = LaneDetectionConfig.SEARCH_BAND_PX,
) -> dict:
    """
    Compute lane parameters (curvature, steering angle, lateral offset) from binary image.

    Detected pixels are translated into a lookahead-centered frame before
    fitting: y is measured from the bottom row of the canvas (y=0) and x from
    the canvas's horizontal center (x=0). Fitting the parabola
    x = A*y^2 + B*y + C directly in this frame means the fit coefficients
    themselves ARE the quantities the controller needs at that point -- B is
    the heading slope and C is the lateral offset -- with no separate
    "evaluate the polynomial at y=height" step (and its extra floating-point
    error) afterward.

    NOTE: y=0 is the NEAR EDGE OF THE ROI, not the rover. With the shipped
    camera geometry that is 1.30 m ahead of the camera, i.e. 1.23 m ahead of
    the front axle and 1.73 m ahead of the rear axle. So `b` is a lookahead
    cross-track error, not the error at the rover: on a curve it is non-zero
    even when the rover is perfectly on the line. That is a usable
    (pure-pursuit-like) signal, but it overlaps with `curvature`, so k_e2 and
    k_ff act on partly the same information when tuning.

    sliding_windows/window_margin/min_window_pixels/min_lane_pixels are
    exposed as arguments (not hardcoded) because their correct values are
    tied to the working canvas size -- since the ROI is now cropped before
    this point, the canvas is much smaller than the un-cropped frame, so
    these need to be tunable without editing code.

    Args:
        binary_warped: Binary bird eye view image
        sliding_windows: Number of vertical search windows (see find_center_line)
        window_margin: Search window half-width in pixels (see find_center_line)
        min_window_pixels: Minimum pixels to recenter a window (see find_center_line)
        min_lane_pixels: Minimum total detected pixels required for a valid fit
        search_center: Column the lane is expected at, in the warped canvas
            (see find_center_line). Defaults to the canvas centre.
        search_band: Half-width of the search band around `search_center`

    Returns:
        Dictionary with keys:
            - curvature: Parabola coefficient A (x = A*y^2 + B*y + C), NaN if not detected
            - theta: Steering angle (degrees) at the fit origin, NaN if not detected
            - b: Lateral offset from center (pixels) at the fit origin, NaN if not detected
            - detected: Boolean flag indicating valid detection
    """
    x_coords, y_coords = find_center_line(
        binary_warped,
        num_windows=sliding_windows,
        window_margin=window_margin,
        min_pixels=min_window_pixels,
        search_center=search_center,
        search_band=search_band,
    )
    height, width = binary_warped.shape[:2]

    result = {"curvature": np.nan, "theta": np.nan, "b": np.nan, "detected": False}

    if len(x_coords) >= min_lane_pixels:
        # Translate to rover-centered frame: y=0 at the bottom row (rover
        # position), x=0 at the horizontal center, before fitting.
        y_shifted = y_coords.astype(np.float64) - height
        x_shifted = x_coords.astype(np.float64) - (width / 2.0)

        # Fit parabola: x = A*y^2 + B*y + C in the shifted frame
        coeff_a, coeff_b, coeff_c = np.polyfit(y_shifted, x_shifted, 2)

        # At y_shifted = 0 (the rover's position), the fit coefficients are
        # directly the heading slope (B) and lateral offset (C).
        theta = np.degrees(np.arctan(coeff_b))
        b_centered = coeff_c

        result = {
            "curvature": coeff_a,
            "theta": theta,
            "b": b_centered,
            "detected": True,
        }

    return result

# ================================
# 6. Full Pipeline
# ================================
def process_frame(
    frame_bgr: np.ndarray,
    roi_base_points: np.ndarray = None,
    roi_base_width: float = LaneDetectionConfig.ROI_BASE_WIDTH,
    roi_base_height: float = LaneDetectionConfig.ROI_BASE_HEIGHT,
    crop_margin_px: float = LaneDetectionConfig.CROP_MARGIN_PX,
    sliding_windows: int = LaneDetectionConfig.SLIDING_WINDOWS,
    window_margin: int = LaneDetectionConfig.WINDOW_MARGIN,
    min_window_pixels: int = LaneDetectionConfig.MIN_WINDOW_PIXELS,
    min_lane_pixels: int = LaneDetectionConfig.MIN_LANE_PIXELS,
    bev_width: int = LaneDetectionConfig.BEV_WIDTH_PX,
    bev_height: int = LaneDetectionConfig.BEV_HEIGHT_PX,
    search_center: float = None,
    search_band: float = LaneDetectionConfig.SEARCH_BAND_PX,
) -> Tuple[float, float, float, bool]:
    """
    Complete lane detection pipeline: crop -> preprocess -> transform -> detect -> compute params.

    Crops to the ROI's bounding box (plus margin) before the expensive
    per-pixel preprocessing runs, so CPU cost scales with the ROI area the
    perspective transform actually uses rather than the full camera frame,
    without reducing pixel density inside the ROI the way a sensor-level
    resolution cut would.

    Args:
        frame_bgr: Input BGR frame from camera
        roi_base_points: ROI trapezoid corners at (roi_base_width x roi_base_height);
            defaults to LaneDetectionConfig.ROI_BASE_POINTS
        roi_base_width: Reference width roi_base_points were authored against
        roi_base_height: Reference height roi_base_points were authored against
        crop_margin_px: Extra margin (in roi_base_width/height units) added
            around the ROI bounding box before cropping
        sliding_windows: Number of vertical search windows
        window_margin: Search window half-width in pixels
        min_window_pixels: Minimum pixels to recenter a search window
        min_lane_pixels: Minimum total detected pixels required for a valid fit
        bev_width: Warped output canvas width (pixels)
        bev_height: Warped output canvas height (pixels). Together with
            bev_width this fixes the bird's-eye scale, so the reported values
            are physical rather than crop-size-dependent.
        search_center: Column the lane is expected at in the warped canvas,
            normally `bev_width/2 + b` from the previous detected frame.
            Defaults to the canvas centre.
        search_band: Half-width of the search band around `search_center`

    Returns:
        Tuple of (curvature, theta, b, detected):
            - curvature: Parabola coefficient A (x = A*y^2 + B*y + C)
            - theta: Steering angle error (degrees)
            - b: Lateral offset (pixels)
            - detected: Boolean detection flag
    """
    if roi_base_points is None:
        roi_base_points = LaneDetectionConfig.ROI_BASE_POINTS

    frame_width, frame_height = frame_bgr.shape[1], frame_bgr.shape[0]

    # Scale the ROI (and its margin) from the base calibration resolution to
    # the actual frame size.
    roi_points_full = get_scaled_roi_points(
        frame_width, frame_height, roi_base_points, roi_base_width, roi_base_height
    )
    margin_x = crop_margin_px * (frame_width / roi_base_width)
    margin_y = crop_margin_px * (frame_height / roi_base_height)

    # Crop to the ROI bounding box before the expensive per-pixel preprocessing
    cropped_frame, x_offset, y_offset = crop_to_roi(frame_bgr, roi_points_full, margin_x, margin_y)
    crop_width, crop_height = cropped_frame.shape[1], cropped_frame.shape[0]

    # Re-express the ROI corners relative to the cropped frame's new origin
    roi_points_cropped = roi_points_full - np.float32([x_offset, y_offset])

    binary = preprocess_frame(cropped_frame)
    warped, M, M_inv = perspective_transform(
        binary, (crop_width, crop_height), roi_points_cropped, bev_size=(bev_width, bev_height)
    )
    params = compute_lane_params(
        warped,
        sliding_windows=sliding_windows,
        window_margin=window_margin,
        min_window_pixels=min_window_pixels,
        min_lane_pixels=min_lane_pixels,
        search_center=search_center,
        search_band=search_band,
    )
    plot_lane_lines(cropped_frame, warped, M_inv, params["theta"], params["b"], params["detected"])

    return params["curvature"], params["theta"], params["b"], params["detected"]


def plot_lane_lines(
    frame_bgr: np.ndarray,
    warped: np.ndarray,
    M_inv: np.ndarray,
    theta: float,
    b: float,
    detected: bool
) -> None:
    """
    Visualize detected lane lines in bird eye and original views.
    
    Args:
        frame_bgr: Original BGR frame
        warped: Bird eye view binary image
        M_inv: Inverse perspective transformation matrix
        theta: Steering angle (degrees)
        b: Lateral offset (pixels)
        detected: Detection flag
    """
    height, width = warped.shape[:2]

    # Generate y-coordinates
    y_vals = np.linspace(0, height - 1, num=height)
    if detected:
        slope = np.tan(np.radians(theta))
        x_vals = slope * y_vals + (b + width // 2)
    else:
        x_vals = np.array([])
        y_vals = np.array([])

    # Bird eye view visualization
    bird_eye_vis = cv2.cvtColor((warped * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    bird_eye_vis = bird_eye_vis[:, 100:width - 100]
    width_cropped = width - 200
    x_vals = x_vals - 100

    if len(x_vals) > 0:
        pts = np.vstack([x_vals, y_vals]).T.astype(np.int32)
        cv2.polylines(bird_eye_vis, [pts], isClosed=False, color=(0, 0, 255), thickness=3)
        cv2.line(bird_eye_vis, (width_cropped // 2, 0), (width_cropped // 2, height), (0, 255, 0), 1)
        cv2.line(bird_eye_vis, (0, height // 2), (width_cropped, height // 2), (255, 0, 0), 1)

    # Original view visualization
    orig_vis = frame_bgr.copy()
    if len(x_vals) > 0:
        pts_warped = np.vstack([x_vals, y_vals]).T.reshape(-1, 1, 2).astype(np.float32)
        pts_original = cv2.perspectiveTransform(pts_warped, M_inv)
        pts_int = pts_original.astype(np.int32)
        cv2.polylines(orig_vis, [pts_int], isClosed=False, color=(0, 0, 255), thickness=3)
        cv2.line(orig_vis, (width // 2, 0), (width // 2, height), (0, 255, 0), 1)
        cv2.line(orig_vis, (0, height // 2), (width, height // 2), (255, 0, 0), 1)

    # Combined visualization
    if orig_vis.shape[1] != bird_eye_vis.shape[1]:
        orig_vis = cv2.resize(orig_vis, (bird_eye_vis.shape[1], bird_eye_vis.shape[0]))
    
    screen_width = 720
    combined_vis = np.vstack([bird_eye_vis, orig_vis])
    h, w = combined_vis.shape[:2]
    if w > screen_width:
        scale = screen_width / w
        combined_vis = cv2.resize(combined_vis, (int(w * scale), int(h * scale)))