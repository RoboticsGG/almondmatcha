"""
Helper Functions Module - Vision Navigation System

Centralizes utility functions used across multiple nodes and modules.
This follows Python best practices for code organization and reusability.

PYTHON HELPERS BEST PRACTICES:
==============================

1. ORGANIZATION:
   - Separate utility/helper functions from business logic
   - Group related helpers by functionality
   - Create dedicated modules for each category

2. FILE NAMING:
   - helpers.py: General-purpose utilities (common choice)
   - utils.py: Utilities (also valid, slightly different connotation)
   - tools.py: Tool functions
   - common.py: Commonly used functions
   - Best practice: Use helpers.py for domain-specific, utils.py for general

3. COMPARISON WITH OTHER LANGUAGES:
   
   C/C++:        util.h / util.cpp
   Java:         UtilityClass.java, *Helper.java
   C#/.NET:      Helpers.cs, Extensions.cs
   Python:       helpers.py, utils.py
   
   Python advantage: Modules are first-class, don't need wrapper classes

4. TYPES OF HELPER FUNCTIONS:
   a) Conversion helpers: Convert data between formats
   b) Validation helpers: Check data validity
   c) Math helpers: Mathematical operations
   d) I/O helpers: File/stream operations
   e) Logging helpers: Structured logging
   f) ROS helpers: ROS-specific utilities

5. BEST PRACTICES:
   - Single responsibility: Each function does one thing well
   - Pure functions: No side effects when possible
   - Type hints: Always include type annotations
   - Documentation: Every function has a docstring
   - Testing: Easy to unit test (no complex dependencies)

Author: Vision Navigation System
Date: November 4, 2025
"""

import os
import csv
import time
from typing import Tuple, List, Dict, Optional, Any
from datetime import datetime
import numpy as np
import cv2


# ===================== CONVERSION HELPERS =====================

def degrees_to_radians(degrees: float) -> float:
    """
    Convert degrees to radians.
    
    Args:
        degrees: Angle in degrees
        
    Returns:
        Angle in radians
    """
    return degrees * np.pi / 180.0


def radians_to_degrees(radians: float) -> float:
    """
    Convert radians to degrees.
    
    Args:
        radians: Angle in radians
        
    Returns:
        Angle in degrees
    """
    return radians * 180.0 / np.pi


def normalize_angle(angle_degrees: float, center: float = 0.0) -> float:
    """
    Normalize angle to range [center-180, center+180].
    
    Useful for ensuring steering angles wrap correctly.
    
    Args:
        angle_degrees: Raw angle in degrees
        center: Center of normalization range (default 0)
        
    Returns:
        Normalized angle
        
    Example:
        normalize_angle(370) -> 10
        normalize_angle(-190) -> 170
    """
    lower = center - 180.0
    upper = center + 180.0
    
    while angle_degrees < lower:
        angle_degrees += 360.0
    while angle_degrees >= upper:
        angle_degrees -= 360.0
    
    return angle_degrees


# ===================== VALIDATION HELPERS =====================

def is_valid_number(value: Any, allow_nan: bool = False, allow_inf: bool = False) -> bool:
    """
    Check if value is a valid number.
    
    Args:
        value: Value to check
        allow_nan: Whether NaN is acceptable
        allow_inf: Whether infinity is acceptable
        
    Returns:
        True if valid number, False otherwise
    """
    try:
        fval = float(value)
        
        if not allow_nan and np.isnan(fval):
            return False
        if not allow_inf and np.isinf(fval):
            return False
        
        return True
    except (TypeError, ValueError):
        return False


def validate_image(img: np.ndarray, expected_channels: Optional[int] = None) -> bool:
    """
    Validate OpenCV image array.
    
    Args:
        img: Image array to validate
        expected_channels: Expected number of channels (None = any)
        
    Returns:
        True if valid, False otherwise
    """
    if not isinstance(img, np.ndarray):
        return False
    
    if len(img.shape) < 2:
        return False
    
    if expected_channels is not None:
        if len(img.shape) < 3:
            actual_channels = 1
        else:
            actual_channels = img.shape[2]
        
        if actual_channels != expected_channels:
            return False
    
    return True


def validate_roi_points(points: np.ndarray) -> bool:
    """
    Validate ROI points for perspective transform.
    
    Args:
        points: Array of 4 points [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]
        
    Returns:
        True if valid 4-point ROI, False otherwise
    """
    if not isinstance(points, (np.ndarray, list)):
        return False
    
    if len(points) != 4:
        return False
    
    for point in points:
        if len(point) != 2:
            return False
        if not all(isinstance(p, (int, float)) for p in point):
            return False
    
    return True


# ===================== MATH HELPERS =====================

def clamp(value: float, min_val: float, max_val: float) -> float:
    """
    Constrain value to [min_val, max_val] range.
    
    Args:
        value: Value to clamp
        min_val: Minimum boundary
        max_val: Maximum boundary
        
    Returns:
        Clamped value
        
    Example:
        clamp(150, -100, 100) -> 100
        clamp(-50, -100, 100) -> -50
    """
    return max(min_val, min(value, max_val))


def lerp(start: float, end: float, t: float) -> float:
    """
    Linear interpolation between two values.
    
    Args:
        start: Start value
        end: End value
        t: Parameter [0, 1]
        
    Returns:
        Interpolated value
        
    Example:
        lerp(0, 100, 0.5) -> 50
    """
    return start + (end - start) * t


def smooth_step(t: float) -> float:
    """
    Smoothstep function for smooth interpolation.
    
    Maps [0, 1] to [0, 1] with smooth acceleration/deceleration.
    
    Args:
        t: Parameter [0, 1]
        
    Returns:
        Smoothed parameter value
        
    Formula: 3*t^2 - 2*t^3
    """
    t = clamp(t, 0.0, 1.0)
    return 3 * t**2 - 2 * t**3


def exponential_moving_average(
    new_value: float,
    previous_avg: Optional[float],
    alpha: float
) -> float:
    """
    Compute exponential moving average.
    
    Args:
        new_value: New measurement
        previous_avg: Previous average (None for first value)
        alpha: Smoothing factor (0 < alpha <= 1)
        
    Returns:
        Updated average
        
    Formula: EMA = alpha * new_value + (1 - alpha) * previous_avg
    """
    if previous_avg is None:
        return new_value
    
    return alpha * new_value + (1 - alpha) * previous_avg


def calculate_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """
    Calculate Euclidean distance between two 2D points.
    
    Args:
        p1: First point (x, y)
        p2: Second point (x, y)
        
    Returns:
        Distance
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return np.sqrt(dx**2 + dy**2)


# ===================== IMAGE HELPERS =====================

def resize_image(
    img: np.ndarray,
    target_width: int,
    target_height: Optional[int] = None,
    maintain_aspect: bool = True
) -> np.ndarray:
    """
    Resize image with optional aspect ratio preservation.
    
    Args:
        img: Input image
        target_width: Target width
        target_height: Target height (if None, calculated from aspect ratio)
        maintain_aspect: Whether to maintain aspect ratio
        
    Returns:
        Resized image
    """
    current_height, current_width = img.shape[:2]
    
    if maintain_aspect and target_height is None:
        # Calculate height maintaining aspect ratio
        aspect_ratio = current_height / current_width
        target_height = int(target_width * aspect_ratio)
    elif target_height is None:
        target_height = target_width  # Square
    
    return cv2.resize(img, (target_width, target_height), interpolation=cv2.INTER_LINEAR)


def crop_image(
    img: np.ndarray,
    x_min: int,
    y_min: int,
    x_max: int,
    y_max: int
) -> np.ndarray:
    """
    Crop image to region [x_min:x_max, y_min:y_max].
    
    Args:
        img: Input image
        x_min, y_min: Top-left corner
        x_max, y_max: Bottom-right corner
        
    Returns:
        Cropped image
    """
    return img[y_min:y_max, x_min:x_max]


def draw_crosshair(
    img: np.ndarray,
    center_x: int,
    center_y: int,
    size: int = 30,
    color: Tuple[int, int, int] = (0, 255, 0),
    thickness: int = 2
) -> np.ndarray:
    """
    Draw crosshair on image.
    
    Args:
        img: Input image
        center_x, center_y: Crosshair center
        size: Crosshair arm length
        color: Color (BGR)
        thickness: Line thickness
        
    Returns:
        Image with crosshair drawn
    """
    img_copy = img.copy()
    
    # Horizontal line
    cv2.line(img_copy, (center_x - size, center_y), (center_x + size, center_y), color, thickness)
    
    # Vertical line
    cv2.line(img_copy, (center_x, center_y - size), (center_x, center_y + size), color, thickness)
    
    # Center circle
    cv2.circle(img_copy, (center_x, center_y), 3, color, -1)
    
    return img_copy


def draw_text_box(
    img: np.ndarray,
    text: str,
    position: Tuple[int, int],
    bg_color: Tuple[int, int, int] = (0, 0, 0),
    text_color: Tuple[int, int, int] = (255, 255, 255),
    font_scale: float = 0.7,
    thickness: int = 1
) -> np.ndarray:
    """
    Draw text with background box on image.
    
    Args:
        img: Input image
        text: Text to draw
        position: (x, y) position
        bg_color: Background color (BGR)
        text_color: Text color (BGR)
        font_scale: Font scale
        thickness: Text thickness
        
    Returns:
        Image with text drawn
    """
    img_copy = img.copy()
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    # Get text size
    (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    
    x, y = position
    
    # Draw background rectangle
    cv2.rectangle(img_copy, (x - 5, y - text_height - 10), (x + text_width + 5, y + baseline + 5), bg_color, -1)
    
    # Draw text
    cv2.putText(img_copy, text, (x, y), font, font_scale, text_color, thickness, cv2.LINE_AA)
    
    return img_copy


# ===================== LOGGING HELPERS =====================

def setup_csv_logging(
    log_path: str,
    headers: List[str],
    create_if_missing: bool = True
) -> Tuple[bool, str]:
    """
    Setup CSV file for logging with headers.
    
    Args:
        log_path: Path to CSV file
        headers: Column headers
        create_if_missing: Create directory if missing
        
    Returns:
        Tuple of (success: bool, message: str)
    """
    try:
        # Create directory if needed
        log_dir = os.path.dirname(log_path)
        if log_dir and not os.path.exists(log_dir):
            if create_if_missing:
                os.makedirs(log_dir, exist_ok=True)
            else:
                return False, f"Directory does not exist: {log_dir}"
        
        # Write headers if file is new
        file_exists = os.path.isfile(log_path)
        if not file_exists:
            with open(log_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow(headers)
        
        return True, f"CSV logging ready: {log_path}"
    
    except Exception as e:
        return False, f"CSV setup failed: {str(e)}"


def log_csv_row(log_path: str, row: List[Any]) -> bool:
    """
    Append row to CSV file.
    
    Args:
        log_path: Path to CSV file
        row: Data row to append
        
    Returns:
        True if successful
    """
    try:
        with open(log_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(row)
        return True
    except Exception:
        return False


def get_timestamp_string(include_milliseconds: bool = True) -> str:
    """
    Get current timestamp as formatted string.
    
    Args:
        include_milliseconds: Include .ms in timestamp
        
    Returns:
        Timestamp string ISO format or with milliseconds
        
    Example:
        "2025-11-04T14:30:45.123" or "2025-11-04 14:30:45"
    """
    if include_milliseconds:
        return datetime.now().isoformat()
    else:
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


# ===================== TIMING HELPERS =====================

class Timer:
    """Simple timer for measuring execution time"""
    
    def __init__(self, name: str = ""):
        """Initialize timer"""
        self.name = name
        self.start_time = None
        self.elapsed = 0.0
    
    def start(self) -> None:
        """Start timer"""
        self.start_time = time.time()
    
    def stop(self) -> float:
        """Stop timer and return elapsed time"""
        if self.start_time is None:
            return 0.0
        self.elapsed = time.time() - self.start_time
        return self.elapsed
    
    def get_elapsed(self) -> float:
        """Get elapsed time without stopping"""
        if self.start_time is None:
            return 0.0
        return time.time() - self.start_time
    
    def __str__(self) -> str:
        """String representation"""
        if self.name:
            return f"{self.name}: {self.elapsed*1000:.2f}ms"
        return f"{self.elapsed*1000:.2f}ms"


# ===================== ROS HELPERS =====================

def get_message_timestamp(ros_message) -> float:
    """
    Extract timestamp from ROS message header.
    
    Args:
        ros_message: ROS message with header
        
    Returns:
        Timestamp in seconds (float)
    """
    if not hasattr(ros_message, 'header'):
        return 0.0
    
    header = ros_message.header
    if not hasattr(header, 'stamp'):
        return 0.0
    
    stamp = header.stamp
    return stamp.sec + stamp.nanosec / 1e9


def create_float_array_message(data: List[float]) -> 'Float32MultiArray':
    """
    Create ROS std_msgs/Float32MultiArray from Python list.
    
    Args:
        data: List of float values
        
    Returns:
        Float32MultiArray message
        
    Note: Requires 'from std_msgs.msg import Float32MultiArray' in calling code
    """
    from std_msgs.msg import Float32MultiArray
    
    msg = Float32MultiArray()
    msg.data = data
    return msg


# ===================== USAGE EXAMPLE =====================

if __name__ == "__main__":
    """
    EXAMPLE: How to use helper functions
    
    In your node file:
    
        from vision_navigation_pkg import helpers
        
        # Math helpers
        clamped_value = helpers.clamp(150, -100, 100)  # 100
        
        # Image helpers
        resized_img = helpers.resize_image(img, 640, maintain_aspect=True)
        
        # Logging helpers
        success, msg = helpers.setup_csv_logging("logs/data.csv", ["time", "value"])
        if success:
            helpers.log_csv_row("logs/data.csv", [time.time(), 42.5])
        
        # Timing helpers
        timer = helpers.Timer("processing")
        timer.start()
        # ... do work ...
        elapsed = timer.stop()
        print(timer)  # "processing: 125.34ms"
    
    BENEFITS:
    
    1. CODE REUSE: Use same function in multiple nodes
    2. CONSISTENCY: All nodes use same helper logic
    3. TESTING: Easy to unit test helper functions
    4. MAINTENANCE: Fix bug once, benefits all nodes
    5. CLARITY: Self-documenting function names
    """
    
    print("Helper Functions Module Loaded")
    print("Usage: from vision_navigation_pkg.helpers import <function_name>")
