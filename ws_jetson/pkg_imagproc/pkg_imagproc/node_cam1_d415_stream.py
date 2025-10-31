#!/usr/bin/env python3
"""
node_cam1_d415_stream.py

ROS 2 (rclpy) node for Intel RealSense D415 streaming OR video file playback:
  (pub) /tpc_rover_d415_rgb   : sensor_msgs/Image (bgr8)
  (pub) /tpc_rover_d415_depth : sensor_msgs/Image (16UC1)  [D415 only]

Params:
  width:int=1280
  height:int=720
  fps:int=30
  json_config:str=""    # optional path to RealSense advanced mode JSON
  open_cam:bool=False   # True = เปิดหน้าต่าง OpenCV ดูภาพสด
  enable_depth:bool=False  # ใช้ได้เฉพาะโหมด D415
  video_path:str=""     # ถ้าไม่ว่าง จะสตรีมจากไฟล์วิดีโอแทน D415
  loop_video:bool=True  # วนไฟล์วิดีโอเมื่อจบ

Usage:
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash

  # เล่นจากไฟล์
  ros2 run my_package node_cam1_d415_stream --ros-args -p video_path:="/path/to/video.mp4" -p open_cam:=True

  # ใช้ D415 ตามเดิม
  ros2 run my_package node_cam1_d415_stream --ros-args -p open_cam:=True -p enable_depth:=True
"""

import os
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# นำเข้า pyrealsense2 แบบปลอดภัย (หากไม่มี ก็ยังเล่นไฟล์วิดีโอได้)
try:
    import pyrealsense2 as rs
except Exception:
    rs = None


class D415StreamNode(Node):
    def __init__(self):
        super().__init__('node_cam1_d415_stream')

        # QoS แบบ sensor data
        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.history = QoSHistoryPolicy.KEEP_LAST
        qos.durability = QoSDurabilityPolicy.VOLATILE

        # Publishers
        self.pub_rgb = self.create_publisher(Image, 'tpc_rover_d415_rgb', qos)
        self.pub_depth = self.create_publisher(Image, 'tpc_rover_d415_depth', qos)

        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('json_config', '')
        self.declare_parameter('open_cam', False)
        self.declare_parameter('enable_depth', False)
        self.declare_parameter('video_path', '')
        self.declare_parameter('loop_video', True)

        self.w = int(self.get_parameter('width').value)
        self.h = int(self.get_parameter('height').value)
        self.fps_param = int(self.get_parameter('fps').value)
        self.json_path = self.get_parameter('json_config').value
        self.open_cam = bool(self.get_parameter('open_cam').value)
        self.enable_depth = bool(self.get_parameter('enable_depth').value)
        self.video_path = str(self.get_parameter('video_path').value).strip()
        self.loop_video = bool(self.get_parameter('loop_video').value)

        # สถานะโหมด
        self.use_video = len(self.video_path) > 0
        self.warned_depth_ignored = False

        # เตรียมแหล่งภาพ
        if self.use_video:
            # โหมดไฟล์วิดีโอ
            self.cap = cv2.VideoCapture(self.video_path)
            if not self.cap.isOpened():
                raise RuntimeError(f"Cannot open video file: {self.video_path}")

            # ใช้ FPS จากไฟล์ ถ้าไม่ได้ให้ใช้จากพารามิเตอร์
            fps_file = self.cap.get(cv2.CAP_PROP_FPS)
            if fps_file is None or fps_file <= 0 or np.isnan(fps_file):
                self.fps_use = max(1, self.fps_param)
            else:
                self.fps_use = int(round(fps_file)) if self.fps_param <= 0 else self.fps_param

            self.get_logger().info(
                f"Video mode: '{self.video_path}' | timer {self.fps_use} FPS | loop={self.loop_video}"
            )
            self.align = None
            self.pipeline = None
        else:
            # โหมด RealSense D415
            if rs is None:
                raise RuntimeError(
                    "ต้องการ pyrealsense2 สำหรับ D415 mode แต่ไม่พบไลบรารี "
                    "(หากต้องการเล่นไฟล์ ให้ตั้งค่า -p video_path:=\".../file.mp4\")"
                )

            self.pipeline = rs.pipeline()
            self.config = rs.config()

            # camera id 806312060441
            self.config.enable_device('806312060441')

            self.config.enable_stream(rs.stream.color, self.w, self.h, rs.format.bgr8, max(1, self.fps_param))
            if self.enable_depth:
                self.config.enable_stream(rs.stream.depth, self.w, self.h, rs.format.z16, max(1, self.fps_param))

            self.profile = self.pipeline.start(self.config)

            # Optional advanced JSON
            if self.json_path and os.path.exists(self.json_path):
                try:
                    dev = self.profile.get_device()
                    adv = rs.rs400_advanced_mode(dev)
                    if not adv.is_enabled():
                        adv.toggle_advanced_mode(True)
                        self.get_logger().info('Enable advanced mode...')
                        time.sleep(2)
                    with open(self.json_path, 'r') as f:
                        js = f.read()
                    adv.load_json(js)
                    self.get_logger().info(f'Loaded JSON config: {self.json_path}')
                except Exception as e:
                    self.get_logger().warn(f'Load JSON failed: {e}')

            # ถ้ามี depth ให้ align กับ color
            self.align = rs.align(rs.stream.color) if self.enable_depth else None
            self.fps_use = max(1, self.fps_param)

            self.get_logger().info('D415 mode started (RGB{}).'.format(' + DEPTH' if self.enable_depth else ''))

        # Timer ตาม FPS
        self.timer = self.create_timer(1.0 / max(1, self.fps_use), self._on_timer)

    def _on_timer(self):
        if self.use_video:
            self._on_timer_video()
        else:
            self._on_timer_realsense()

    # -------------------- Video mode --------------------
    def _on_timer_video(self):
        # depth ใช้ไม่ได้ในโหมดวิดีโอ
        if self.enable_depth and not self.warned_depth_ignored:
            self.get_logger().warn("enable_depth ถูกละเลยในโหมด video_path (จะปล่อยเฉพาะ RGB)")
            self.warned_depth_ignored = True

        ret, frame = self.cap.read()
        if not ret:
            if self.loop_video:
                # วนกลับไปต้นไฟล์
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().error("ไม่สามารถอ่านเฟรมจากไฟล์วิดีโอได้ แม้จะวนกลับต้นไฟล์แล้ว")
                    self._request_shutdown()
                    return
            else:
                self.get_logger().info("วิดีโอจบการเล่น (loop_video=False) — กำลังปิดตัวลง")
                self._request_shutdown()
                return

        # ปรับขนาดถ้าจำเป็นให้ตรงกับพารามิเตอร์ width/height
        if self.w > 0 and self.h > 0 and (frame.shape[1] != self.w or frame.shape[0] != self.h):
            frame = cv2.resize(frame, (self.w, self.h), interpolation=cv2.INTER_LINEAR)

        # Publish RGB
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'file_color_frame'
        self.pub_rgb.publish(msg)

    # -------------------- RealSense mode --------------------
    def _on_timer_realsense(self):
        # รอ frame แบบมี timeout ลด CPU
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=100)
        except Exception:
            return
        if not frames:
            return

        if self.align is not None:
            frames = self.align.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame() if self.align is not None else None

        # Publish RGB
        if color_frame:
            color_img = np.asanyarray(color_frame.get_data())  # BGR
            msg = self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'd415_color_optical_frame'
            self.pub_rgb.publish(msg)

        # Publish Depth (ถ้าเปิดใช้งาน)
        if depth_frame:
            depth_img = np.asanyarray(depth_frame.get_data())  # uint16
            msgd = self.bridge.cv2_to_imgmsg(depth_img, encoding='16UC1')
            msgd.header.stamp = self.get_clock().now().to_msg()
            msgd.header.frame_id = 'd415_depth_optical_frame'
            self.pub_depth.publish(msgd)

    # -------------------- Utils --------------------
    def _request_shutdown(self):
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    def destroy_node(self):
        try:
            if self.pipeline is not None:
                self.pipeline.stop()
        except Exception:
            pass
        try:
            if self.use_video and hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        return super().destroy_node()


def main():
    rclpy.init()
    node = D415StreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl-C received, stopping gracefully...')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
