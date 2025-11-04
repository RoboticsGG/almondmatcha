"""
Configuration Module - Vision Navigation System

Centralizes all system parameters and tunable constants.

Author: Vision Navigation System
Date: November 4, 2025
"""


# ===================== CAMERA CONFIGURATION =====================

class CameraConfig:
    """Camera streaming parameters"""
    
    # D415 Resolution and Framerate
    WIDTH = 1280
    HEIGHT = 720
    FPS = 30
    
    # Camera modes
    OPEN_CAMERA_DISPLAY = False      # Show camera feed in OpenCV window
    ENABLE_DEPTH_STREAM = False      # Stream depth data from D415
    LOOP_VIDEO = True                # Loop video when file reaches end
    
    # File paths
    VIDEO_PATH = ""                  # Empty string = use D415 camera
    JSON_CONFIG_PATH = ""             # Path to RealSense advanced mode JSON
    
    # D415 specific
    D415_SERIAL = "806312060441"     # Specific D415 camera serial number
    
    @classmethod
    def get_resolution(cls):
        """Get camera resolution as tuple"""
        return (cls.WIDTH, cls.HEIGHT)
    
    @classmethod
    def get_fps(cls):
        """Get frames per second"""
        return cls.FPS
    
    @classmethod
    def is_video_mode(cls):
        """Check if running in video file mode"""
        return len(cls.VIDEO_PATH) > 0


# ===================== LANE DETECTION CONFIGURATION =====================

class LaneDetectionConfig:
    """Lane detection algorithm parameters"""
    
    # ===== Color Space Thresholds (LAB) =====
    # LAB color space ranges for filtering
    LAB_GREEN_A_MAX = 120
    LAB_GREEN_B_MIN = 130
    LAB_RED_A_MIN = 140
    LAB_RED_B_MAX = 140
    
    # ===== Gradient Detection Thresholds =====
    # Sobel edge detection parameters
    GRADIENT_SOBEL_MIN = 50
    GRADIENT_SOBEL_MAX = 100
    GRADIENT_SOBEL_KERNEL = 3
    
    # Magnitude detection
    MAGNITUDE_MIN = 30
    MAGNITUDE_MAX = 100
    
    # Direction detection
    DIRECTION_MIN = 0.7
    DIRECTION_MAX = 1.3
    
    # White line detection
    WHITE_THRESHOLD = 180
    
    # ===== Contour Filtering =====
    MIN_CONTOUR_AREA = 100  # Pixels, remove smaller contours
    
    # ===== Lane Finding Parameters =====
    MIN_LANE_PIXELS = 50    # Minimum pixels for valid detection
    SLIDING_WINDOWS = 9     # Number of vertical search windows
    WINDOW_MARGIN = 100     # Horizontal search margin (±pixels)
    MIN_WINDOW_PIXELS = 50  # Minimum pixels to recenter window
    
    # ===== Perspective Transform =====
    # ROI (Region of Interest) points for bird eye view (1280x720 base)
    ROI_BASE_POINTS = [
        [0, 500],        # Bottom-left
        [1280, 500],     # Bottom-right
        [900, 200],      # Top-right
        [400, 200]       # Top-left
    ]
    
    # Destination margins for warped image
    PERSPECTIVE_LEFT_MARGIN = 0.25   # 25% from left
    PERSPECTIVE_RIGHT_MARGIN = 0.75  # 75% from right
    
    # ===== Visualization =====
    SHOW_WINDOW = False              # Display lane detection output
    VISUALIZATION_WIDTH = 720        # Screen display width
    CROP_MARGIN = 100                # Crop sides for visualization
    
    @classmethod
    def get_roi_points(cls):
        """Get ROI points as numpy array"""
        import numpy as np
        return np.float32(cls.ROI_BASE_POINTS)


# ===================== STEERING CONTROL CONFIGURATION =====================

class ControlConfig:
    """Rover steering control parameters"""
    
    # ===== PID Gains =====
    # Control law: u = kp*e + ki*∫e*dt + kd*de/dt
    K_P = 4.0          # Proportional gain (primary response)
    K_I = 0.0          # Integral gain (steady-state correction)
    K_D = 0.0          # Derivative gain (damping)
    
    # ===== Error Weights =====
    # Combined error: e_total = k_e1*theta + k_e2*b
    K_E1 = 1.0         # Weight on heading error (theta)
    K_E2 = 0.1         # Weight on lateral offset (b)
    
    # ===== Filtering =====
    EMA_ALPHA = 0.05   # Exponential moving average smoothing factor
    # Higher alpha = more responsive to new data
    # Lower alpha = more smoothing
    
    # ===== Output Saturation =====
    STEER_MAX_DEGREES = 60.0        # Maximum steering angle (±degrees)
    STEER_WHEN_LOST = 0.0           # Steering angle when lane not detected
    
    # ===== Integral Anti-Windup =====
    INTEGRAL_LIMIT = 200.0          # Anti-windup saturation for integral term
    
    # ===== Timing =====
    CONTROL_LOOP_RATE = 50          # Hz (20ms per cycle)
    
    @classmethod
    def get_pid_gains(cls):
        """Get PID gains as dict"""
        return {
            "kp": cls.K_P,
            "ki": cls.K_I,
            "kd": cls.K_D
        }
    
    @classmethod
    def get_error_weights(cls):
        """Get error combination weights"""
        return {
            "k_e1": cls.K_E1,
            "k_e2": cls.K_E2
        }


# ===================== LOGGING CONFIGURATION =====================

class LoggingConfig:
    """Data logging and CSV output parameters"""
    
    # ===== File Paths =====
    LOG_DIRECTORY = "logs"           # Base directory for all logs
    CAMERA_LOG_FILE = "camera_stream.csv"
    LANE_DETECTION_LOG = "lane_pub_log.csv"
    CONTROL_LOG_FILE = "rover_ctl_log_ver_3.csv"
    
    # ===== CSV Headers =====
    CAMERA_LOG_COLUMNS = ["timestamp", "frame_id", "width", "height"]
    LANE_DETECTION_COLUMNS = ["timestamp", "theta", "b", "detected"]
    CONTROL_LOG_COLUMNS = ["time_sec", "theta_ema", "b_ema", "u", "e_sum"]
    
    # ===== Logging Control =====
    ENABLE_CAMERA_LOGGING = False
    ENABLE_LANE_LOGGING = True
    ENABLE_CONTROL_LOGGING = True
    LOG_FRAME_INTERVAL = 1           # Log every Nth frame (1 = every frame)


# ===================== TOPIC CONFIGURATION =====================

class TopicConfig:
    """ROS2 topic names - follows tpc_* convention"""
    
    # Camera topics
    RGB_STREAM = "/tpc_rover_d415_rgb"
    DEPTH_STREAM = "/tpc_rover_d415_depth"
    
    # Lane detection topics
    LANE_PARAMETERS = "/tpc_rover_nav_lane"    # [theta, b, detected]
    
    # Control topics
    STEERING_COMMAND = "/tpc_rover_fmctl"      # [steer_angle, detected]
    
    @classmethod
    def get_all_topics(cls):
        """Get dict of all topics"""
        return {
            "rgb": cls.RGB_STREAM,
            "depth": cls.DEPTH_STREAM,
            "lane": cls.LANE_PARAMETERS,
            "steering": cls.STEERING_COMMAND
        }


# ===================== SYSTEM CONFIGURATION =====================

class SystemConfig:
    """Overall system parameters"""
    
    # ===== ROS2 Domain =====
    ROS_DOMAIN_ID = 5               # Domain for inter-robot communication
    
    # ===== Node Names =====
    CAMERA_NODE_NAME = "node_cam1_d415_stream"
    LANE_NODE_NAME = "node_cam1_nav_process"
    CONTROL_NODE_NAME = "node_rover_ctl"
    
    # ===== Initialization Timing =====
    CAMERA_INIT_DELAY = 2.0         # Seconds: wait for camera to initialize
    LANE_INIT_DELAY = 0.5           # Seconds: wait after camera ready
    CONTROL_INIT_DELAY = 0.5        # Seconds: wait after lane detection ready
    
    # ===== Quality of Service =====
    QOS_DEPTH = 5                   # Message queue depth
    QOS_RELIABILITY = "BEST_EFFORT" # "BEST_EFFORT" or "RELIABLE"
    
    # ===== Debug/Development =====
    DEBUG_MODE = False               # Extra logging and visualization
    PROFILE_PERFORMANCE = False      # Measure execution time


# ===================== HELPER: CONFIGURATION OVERRIDE ======================

def override_from_environment():
    """Override config values from environment variables.
    
    Pattern: MODULENAME_PARAMNAME (e.g., CAMERA_WIDTH, CONTROL_K_P)
    """
    import os
    
    # Camera overrides
    if "CAMERA_WIDTH" in os.environ:
        CameraConfig.WIDTH = int(os.environ["CAMERA_WIDTH"])
    if "CAMERA_HEIGHT" in os.environ:
        CameraConfig.HEIGHT = int(os.environ["CAMERA_HEIGHT"])
    if "CAMERA_FPS" in os.environ:
        CameraConfig.FPS = int(os.environ["CAMERA_FPS"])
    
    # Control overrides
    if "CONTROL_K_P" in os.environ:
        ControlConfig.K_P = float(os.environ["CONTROL_K_P"])
    if "CONTROL_K_I" in os.environ:
        ControlConfig.K_I = float(os.environ["CONTROL_K_I"])
    if "CONTROL_K_D" in os.environ:
        ControlConfig.K_D = float(os.environ["CONTROL_K_D"])




if __name__ == "__main__":
    """Display configuration values"""
    print("Vision Navigation System Configuration")
    print("=" * 50)
    print(f"\nCamera Configuration:")
    print(f"  Resolution: {CameraConfig.get_resolution()}")
    print(f"  FPS: {CameraConfig.get_fps()}")
    print(f"  Video mode: {CameraConfig.is_video_mode()}")
    
    print(f"\nControl Configuration:")
    print(f"  PID gains: {ControlConfig.get_pid_gains()}")
    print(f"  Error weights: {ControlConfig.get_error_weights()}")
    print(f"  Max steering: {ControlConfig.STEER_MAX_DEGREES} degrees")
    
    print(f"\nLogging Configuration:")
    print(f"  Directory: {LoggingConfig.LOG_DIRECTORY}")
    print(f"  Enabled: Lane={LoggingConfig.ENABLE_LANE_LOGGING}, "
          f"Control={LoggingConfig.ENABLE_CONTROL_LOGGING}")
    
    print(f"\nTopic Configuration:")
    for name, topic in TopicConfig.get_all_topics().items():
        print(f"  {name}: {topic}")
