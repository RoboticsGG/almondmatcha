#!/usr/bin/env python3
"""
node_cam1_nav_process.py (updated)

ROS 2 node: รับภาพจาก /tpc_rover_d415_rgb → คำนวณเลนด้วย lane_detector.py
แล้ว publish /tpc_rover_nav_lane เป็น [theta, b, detected]

Usage:
  ros2 run my_package node_cam1_nav_process --ros-args -p show_window:=true
"""

import math
import numpy as np
import cv2
import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

# === lane detector pipeline ===
from pkg_imagproc.lane_detector import process_frame


class Cam1NavProcessNode(Node):
    def __init__(self):
        super().__init__('node_cam1_nav_process')

        # QoS เหมาะกับ sensor data
        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        # Subscriber → จาก node_cam1_d415_stream (RGB)
        self.sub = self.create_subscription(Image, '/tpc_rover_d415_rgb', self._on_image, qos)

        # Publisher → ส่งผลลัพธ์ (theta, b, detected)
        self.pub_lane = self.create_publisher(Float32MultiArray, '/tpc_rover_nav_lane', 10)
        self.bridge = CvBridge()

        # Params
        self.declare_parameter('show_window', False)
        self.show_window = bool(self.get_parameter('show_window').value)

        self.window_name = "nav_vis"
        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 1280, 720)

        self.frame_idx = 0
        self.get_logger().info('cam1_nav_process started (RGB -> lane params).')

    def _on_image(self, msg: Image):
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # ---- run lane detection pipeline ----
        theta, b, detected = process_frame(bgr)

        # clean values
        a = 0.0 if (theta is None or (isinstance(theta, float) and math.isnan(theta))) else float(theta)
        b_pub = float(b) if isinstance(b, (float, np.floating)) else float('nan')
        detected_val = 1.0 if detected else 0.0

        # publish lane summary [theta, b, detected]
        out_lane = Float32MultiArray()
        out_lane.data = [a, b_pub, detected_val]
        self.pub_lane.publish(out_lane)

        # save log to CSV
        csv_path = 'lane_pub_log.csv'
        timestamp = datetime.now().isoformat()
        row = [timestamp, a, b_pub, detected_val]
        file_exists = os.path.isfile(csv_path)
        with open(csv_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(['timestamp', 'theta', 'b', 'detected'])
            writer.writerow(row)

        # log terminal
        self.frame_idx += 1
        print(f"Frame {self.frame_idx}: theta={a:.3f}, b={b_pub:.3f}, detected={detected_val}")

        # show preview
        if self.show_window:
            vis = bgr.copy()
            cv2.putText(vis, f"theta={a:.2f}, b={b_pub:.1f}, detected={detected}",
                        (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            h, w = vis.shape[:2]
            center_x, center_y = w // 2, h // 2

            cv2.line(vis, (0, center_y), (w, center_y), (255, 0, 0), 2)

            cv2.line(vis, (center_x, 0), (center_x, h), (0, 0, 255), 2)
            
            cv2.imshow(self.window_name, vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                self.get_logger().info("ESC/q pressed, shutting down...")
                rclpy.shutdown()

def main():
    rclpy.init()
    node = Cam1NavProcessNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
