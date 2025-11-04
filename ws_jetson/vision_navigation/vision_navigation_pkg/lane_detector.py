"""
Lane Detection Image Processing Pipeline

Provides image processing functions for detecting lane markers in RGB frames.
Includes preprocessing, perspective transformation, and lane fitting algorithms.

Author: Vision Navigation System
Date: November 4, 2025
"""

import cv2
import numpy as np
from typing import Tuple
import matplotlib.pyplot as plt

# ================================
# 1. Threshold + Preprocess
# ================================
def preprocess_frame(frame_bgr: np.ndarray, min_area: int = 100) -> np.ndarray:
    """
    Preprocess frame: color filtering and edge detection.
    
    Removes green lane markers and detects lane boundaries using
    LAB color space filtering and gradient-based edge detection.
    
    Args:
        frame_bgr: Input BGR image from camera
        min_area: Minimum contour area for noise filtering (pixels)
        
    Returns:
        Binary image with detected lane markers
    """
    img_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    frame = cv2.GaussianBlur(img_rgb, (5, 5), 0)
    frame = cv2.medianBlur(frame, 5)

    # Convert to LAB color space for filtering
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
    L = lab[:, :, 0]
    A = lab[:, :, 1]
    B = lab[:, :, 2]

    # Create mask for green pixels (to remove from processing)
    green_mask = np.zeros_like(A, dtype=np.uint8)
    green_mask[(A < 120) & (B > 130)] = 1

    # Create mask for red pixels (to keep)
    red_mask = np.zeros_like(A, dtype=np.uint8)
    red_mask[(A > 140) & (B < 140)] = 1

    # Remove green pixels from LAB image
    lab_no_green = lab.copy()
    lab_no_green[green_mask == 1] = 0

    # Convert back to RGB for gradient computation
    img_rgb_no_green = cv2.cvtColor(lab_no_green, cv2.COLOR_LAB2RGB)
    gray = cv2.cvtColor(img_rgb_no_green, cv2.COLOR_RGB2GRAY)

    # Sobel x, y
    abs_sobel_x = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3))
    abs_sobel_y = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3))
    gradx = np.zeros_like(gray, dtype=np.uint8)
    grady = np.zeros_like(gray, dtype=np.uint8)
    gradx[(abs_sobel_x >= 50) & (abs_sobel_x <= 100)] = 1
    grady[(abs_sobel_y >= 50) & (abs_sobel_y <= 100)] = 1

    # Magnitude
    mag = np.sqrt(abs_sobel_x**2 + abs_sobel_y**2)
    mag = (mag / (np.max(mag) / 255.0 + 1e-6)).astype(np.uint8)
    mag_binary = np.zeros_like(mag)
    mag_binary[(mag >= 30) & (mag <= 100)] = 1

    # Direction
    dir_binary = np.zeros_like(gray)
    absgraddir = np.arctan2(abs_sobel_y, abs_sobel_x + 1e-9)
    dir_binary[(absgraddir >= 0.7) & (absgraddir <= 1.3)] = 1

    # White pixel detection (emphasize white lane lines)
    white_binary = np.zeros_like(gray, dtype=np.uint8)
    white_binary[gray > 180] = 1

    # Combined binary mask
    combined = np.zeros_like(gray, dtype=np.uint8)
    combined[((gradx == 1) & (grady == 1)) |
             ((mag_binary == 1) & (dir_binary == 1)) |
             (white_binary == 1)] = 1

    # Noise filtering: remove small contours
    contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv2.contourArea(cnt) < min_area:
            cv2.drawContours(combined, [cnt], 0, 0, -1)

    return combined


# ================================
# 2. Perspective Transform
# ================================
def perspective_transform(
    binary: np.ndarray,
    frame_size: Tuple[int, int]
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Apply perspective transformation to bird eye view.
    
    Transforms the image from camera view to top-down view for easier
    lane detection in the transformed coordinate system.
    
    Args:
        binary: Binary input image
        frame_size: Tuple of (width, height)
        
    Returns:
        Tuple of (warped, M, M_inv):
            - warped: Transformed binary image (bird eye view)
            - M: Perspective transformation matrix
            - M_inv: Inverse transformation matrix
    """
    h, w = frame_size

    ROI_BASE = np.float32([[0, 500], [1280, 500], [900, 200], [400, 200]])
    sx, sy = w / 1280.0, h / 720.0
    roi_points = np.float32([[p[0] * sx, p[1] * sy] for p in ROI_BASE])

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
# 3. Lane Finding (single center line)
# ================================
def find_center_line(
    binary_warped: np.ndarray,
    num_windows: int = 9,
    window_margin: int = 100,
    min_pixels: int = 50
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Find lane center line pixels using sliding window technique.
    
    Args:
        binary_warped: Binary bird eye view image
        num_windows: Number of horizontal windows
        window_margin: Width of search window (plus-minus margin)
        min_pixels: Minimum pixels to recenter window
        
    Returns:
        Tuple of (x_coords, y_coords) of detected lane pixels
    """
    histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis=0)
    base_x = int(np.argmax(histogram))

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
# 4. Fit single line & Params
# ================================
def compute_lane_params(binary_warped: np.ndarray) -> dict:
    """
    Compute lane parameters (steering angle, lateral offset) from binary image.
    
    Args:
        binary_warped: Binary bird eye view image
        
    Returns:
        Dictionary with keys:
            - theta: Steering angle (degrees), NaN if not detected
            - b: Lateral offset from center (pixels), NaN if not detected
            - detected: Boolean flag indicating valid detection
    """
    x_coords, y_coords = find_center_line(binary_warped)
    height, width = binary_warped.shape[:2]

    result = {"theta": np.nan, "b": np.nan, "detected": False}

    if len(x_coords) >= 50:
        # Fit line: x = m*y + b (y is vertical axis)
        slope, intercept = np.polyfit(y_coords, x_coords, 1)
        theta = np.degrees(np.arctan(slope))
        b_centered = intercept - (width // 2)

        result = {"theta": theta, "b": b_centered, "detected": True}

    return result

# ================================
# 5. Full Pipeline
# ================================
# ================================
# 5. Full Pipeline
# ================================
def process_frame(frame_bgr: np.ndarray) -> Tuple[float, float, bool]:
    """
    Complete lane detection pipeline: preprocess -> transform -> detect -> compute params.
    
    Args:
        frame_bgr: Input BGR frame from camera
        
    Returns:
        Tuple of (theta, b, detected):
            - theta: Steering angle error (degrees)
            - b: Lateral offset (pixels)
            - detected: Boolean detection flag
    """
    binary = preprocess_frame(frame_bgr)
    frame_width, frame_height = frame_bgr.shape[1], frame_bgr.shape[0]
    warped, M, M_inv = perspective_transform(binary, (frame_width, frame_height))
    params = compute_lane_params(warped)
    plot_lane_lines(frame_bgr, warped, M_inv, params["theta"], params["b"], params["detected"])
    
    return params["theta"], params["b"], params["detected"]


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


def plot_lane_lines(frame_bgr, warped, Minv, theta, b, detected):
    H, W = warped.shape[:2]

    # ----- Step 1: เตรียมแกน y สำหรับ Bird’s eye view -----
    y_vals = np.linspace(0, H-1, num=H)
    if detected:
        m = np.tan(np.radians(theta))
        x_vals = m * y_vals + (b + W//2)  # ย้าย origin กลับจาก center
    else:
        x_vals = np.array([])
        y_vals = np.array([])

    # ----- Step 2: Bird's-eye view -----
    bird_eye_vis = cv2.cvtColor((warped*255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    # ตัดขอบตามแนวยาว ฝั่งละ 100 px
    bird_eye_vis = bird_eye_vis[:, 100:W-100]
    warped = warped[:, 100:W-100]
    W = W - 200  # ปรับขนาด W หลังตัดขอบ
    x_vals = x_vals - 100  # ปรับตำแหน่ง x ให้ตรงกับภาพที่ถูกตัดขอบ
    # cv2.imshow("Bird's Eye View", bird_eye_vis)

    

    if len(x_vals) > 0:
        pts = np.vstack([x_vals, y_vals]).T.astype(np.int32)
        cv2.polylines(bird_eye_vis, [pts], isClosed=False, color=(0,0,255), thickness=3)

        # วาดแกน (0,0) ที่กึ่งกลางภาพ
        cv2.line(bird_eye_vis, (W//2,0), (W//2,H), (0,255,0), 1)
        cv2.line(bird_eye_vis, (0,H//2), (W,H//2), (255,0,0), 1)

    # ----- Step 3: Original view (inverse warp line กลับ) -----
    orig_vis = frame_bgr.copy()
    if len(x_vals) > 0:
        pts = np.vstack([x_vals, y_vals]).T.reshape(-1,1,2).astype(np.float32)
        pts = cv2.perspectiveTransform(pts, Minv)  

        pts_int = pts.astype(np.int32)
        cv2.polylines(orig_vis, [pts_int], isClosed=False, color=(0,0,255), thickness=3)

        # วาดแกน (0,0) กลางภาพ
        cv2.line(orig_vis, (W//2,0), (W//2,H), (0,255,0), 1)
        cv2.line(orig_vis, (0,H//2), (W,H//2), (255,0,0), 1)

    # Resize combined visualization to fit screen width
    # Resize orig_vis to match bird_eye_vis width
    if orig_vis.shape[1] != bird_eye_vis.shape[1]:
        orig_vis = cv2.resize(orig_vis, (bird_eye_vis.shape[1], bird_eye_vis.shape[0]))
    
    screen_width = 720  
    combined_vis = np.vstack([bird_eye_vis, orig_vis])
    h, w = combined_vis.shape[:2]
    scale = screen_width / w if w > screen_width else 1.0
    if scale < 1.0:
        combined_vis = cv2.resize(combined_vis, (int(w * scale), int(h * scale)))
    # cv2.imshow("Lane Detection (Bird's Eye View + Original)", combined_vis)