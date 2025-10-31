#!/usr/bin/env python3
"""
Node name : node_rover_ctl
Description : Closed-loop control for lane following (steer-only). Saturation ±40°.
Location : RPi4

Publishing:
  - tpc_rover_fmctl : std_msgs/Float32MultiArray  [steer_angle_deg, detected_flag]

Subscribing:
  - tpc_rover_nav_lane : std_msgs/Float32MultiArray [zeta_deg, offset_val, detected_flag]

Sign convention:
  +steer_angle => turn RIGHT
  -steer_angle => turn LEFT
"""

import time
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
import os
import csv

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class MovingAverageLPF:
    def __init__(self, window_size):
        self.window_size = window_size
        self.buffer = []
    
    def update(self, new_value):
        self.buffer.append(new_value)
        
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)
        
        if len(self.buffer) == self.window_size:
            return sum(self.buffer) / self.window_size
        else:
            return new_value 

class ExponentialMovingAverageLPF:
    def __init__(self, alpha, maxlen=10):
        self.alpha = alpha
        self.ema = None
        self.maxlen = maxlen                   
        self.buffer = deque(maxlen=maxlen)     

    def update(self, new_value):
        self.buffer.append(new_value)          
        if self.ema is None:
            self.ema = new_value
        else:
            self.ema = self.alpha * new_value + (1 - self.alpha) * self.ema
        return self.ema
    
def clamp(value, min_val, max_val):
        return max(min_val, min(value, max_val))

class RoverCtlNode(Node):
    def __init__(self):
        super().__init__('node_rover_ctl')

        # ---- Control gains ----
        self.declare_parameter('k_e1', 0.5)     # weight for zeta (heading error)
        self.declare_parameter('k_e2', 0.01)     # weight for b (lateral offset)
        self.declare_parameter('k_p', 1.2)     # proportional gain
        self.declare_parameter('k_i', 0.001)     # integral gain
        
        self.k_e1    = float(self.get_parameter('k_e1').value)
        self.k_e2    = float(self.get_parameter('k_e2').value)
        self.k_p     = float(self.get_parameter('k_p').value)
        self.k_i     = float(self.get_parameter('k_i').value)
        
        # ---- Filter params ----
        # self.declare_parameter('ma_window', 30)   # for MA
        self.declare_parameter('ema_alpha', 0.05) # for EMA

        # ma_window = int(self.get_parameter('ma_window').value)
        ema_alpha = float(self.get_parameter('ema_alpha').value)

        # Init filters
        # self.ma_zeta = MovingAverageLPF(ma_window)
        # self.ma_b    = MovingAverageLPF(ma_window)
        self.ema_zeta = ExponentialMovingAverageLPF(ema_alpha)
        self.ema_b    = ExponentialMovingAverageLPF(ema_alpha)

        # ---- Behavior/Safety ----
        self.declare_parameter('steer_max_deg', 60.0)  # saturation (±deg)
        self.declare_parameter('steer_when_lost', 0.0) # steering to use when not detected (default: straight)

        self.steer_max_deg   = float(self.get_parameter('steer_max_deg').value)
        self.steer_when_lost = float(self.get_parameter('steer_when_lost').value)

        # ---- State ----
        self.integral  = 0.0
        self.last_time = None

        # ---- Publishers ----
        self.pub_fmctl = self.create_publisher(Float32MultiArray, 'tpc_rover_fmctl', 10)

        # ---- Subscribers ----
        self.sub_lane = self.create_subscription(Float32MultiArray, 'tpc_rover_nav_lane', self.lane_callback, 10)

        # ---- Logging to CSV ----
        log_dir = "logs"
        os.makedirs(log_dir, exist_ok=True)
        self.csv_path = os.path.join(log_dir, "rover_ctl_log_ver_2.csv")
        self.csv_file = open(self.csv_path, mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time_sec", "zeta_ema", "b_ema", "u", "e_sum"])

        self.get_logger().info('node_rover_ctl started (steer-only, closed-loop lane following).')

    # ------------------- Callbacks -------------------

    def lane_callback(self, msg: Float32MultiArray):
        # Expect msg.data = [zeta_deg, b, detected_flag]
        if len(msg.data) < 3:
            self.get_logger().warn('lane msg.data too short')
            return

        zeta = float(msg.data[0])       # heading/angle error (+ = needs right turn)
        b    = float(msg.data[1])       # lateral offset (unit depends on upstream)
        # detected = bool(msg.data[2])  # 1.0 or 0.0

        # Clamp inputs to avoid spikes
        zeta = clamp(zeta, -35, 35)
        b = clamp(b, -100, 100)
        
        # zeta = max(-35.0, min(35.0, zeta))
        # b    = max(-100.0,  min(100.0,  b))
        
        # Update both filters
        # zeta_ma = self.ma_zeta.update(zeta)  # not used further
        # b_ma    = self.ma_b.update(b)        # not used further
        zeta_ema = self.ema_zeta.update(zeta)
        b_ema    = self.ema_b.update(b)
        
        # ใช้ detect = False จนกว่า buffer จะเต็ม
        if len(self.ema_zeta.buffer) < self.ema_zeta.maxlen or \
           len(self.ema_b.buffer)   < self.ema_b.maxlen:
            detected = False
        else:
            detected = bool(msg.data[2])
        
        # Combine errors
        e_sum = (self.k_e1 * zeta_ema) + (self.k_e2 * b_ema)
                
        # PI control with wall-clock dt
        now = time.time()
        
        dt = (now - self.last_time) if (self.last_time is not None) else 0.05
        self.last_time = now

        self.integral += e_sum * dt 
        self.integral = clamp(self.integral, -200, 200) # clamp controller integral term.
        
        u = self.k_p * e_sum + self.k_i * self.integral
                
        # Steering (deg)
        steer_angle = u

        # If lane is not detected -> set to configured safe steering (default 0° = straight)
        if not detected:
            steer_angle = self.steer_when_lost

        # Saturation ±steer_max_deg
        steer_angle = float(np.clip(steer_angle, -self.steer_max_deg, self.steer_max_deg))
        
        timestamp = time.time()
        self.csv_writer.writerow([timestamp, zeta_ema, b_ema, u, e_sum])
        self.csv_file.flush()

        # Publish front-module control: [steer_deg, detected]
        fm = Float32MultiArray()
        fm.data = [steer_angle, float(detected)]
        self.pub_fmctl.publish(fm)

        self.get_logger().info(
            f"lane (LPF): zeta={zeta_ema:.2f}, b={b_ema:.2f}\n"
            f"lane: e={e_sum:.2f}, u={u:.2f}, "
            f"steer={steer_angle:.2f} deg, detected={int(detected)}\n\n"
        )

# ------------------- Entry point -------------------

def main():
    rclpy.init()
    node = RoverCtlNode()
    try:
        rclpy.spin(node)
    finally:
        if hasattr(node, "csv_file") and node.csv_file:
            node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
