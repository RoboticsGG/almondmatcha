#!/usr/bin/env python3
"""
Local Rover Monitoring Node (Jetson - Domain 4)

Subscribes to /tpc_telemetry_relay (Domain 4) and logs all telemetry data to CSV files.
Future-ready for database integration (replacing CSV with SQLite/PostgreSQL).

Architecture:
- Single subscription to aggregated TelemetryRelay message
- Event-driven CSV logging at 5 Hz (relay rate)
- Runs on Jetson (high storage capacity, high bandwidth)
- Isolated from rover control domain (Domain 5)

Benefits vs previous node_rover_monitoring:
- No Domain 5 participant (saves STM32 memory)
- Single subscription vs 10+ subscriptions (cleaner)
- Centralized logging on high-capacity Jetson storage
- Easy migration path to database backend
"""

import rclpy
from rclpy.node import Node
from msgs_ifaces.msg import TelemetryRelay
import csv
import os
from datetime import datetime
from pathlib import Path
import glob


class RoverLocalMonitoringNode(Node):
    def __init__(self):
        super().__init__('rover_local_monitoring_node')
        
        # Initialize CSV logging
        self.init_csv_logging()
        
        # Subscribe to telemetry relay (Domain 4)
        self.sub_telemetry_relay = self.create_subscription(
            TelemetryRelay,
            '/tpc_telemetry_relay',
            self.telemetry_relay_callback,
            10
        )
        
        self.get_logger().info('=== Local Rover Monitoring Node (Jetson) ===')
        self.get_logger().info('Domain: 4 (base telemetry)')
        self.get_logger().info('Subscribes to: /tpc_telemetry_relay (5 Hz)')
        self.get_logger().info(f'CSV logging to: {self.log_dir}')
        self.get_logger().info('CSV files:')
        self.get_logger().info('  - telemetry_unified.csv (all data at 5 Hz)')
        self.get_logger().info('  - rtk_gnss.csv (RTK position data)')
        self.get_logger().info('  - spresense_gnss.csv (Spresense position data)')
        self.get_logger().info('  - chassis_data.csv (sensors + IMU + commands)')
        self.get_logger().info('  - mission_state.csv (mission status)')
        self.get_logger().info('Future: Database backend (SQLite/PostgreSQL)')
        self.get_logger().info('============================================')

    def get_next_run_number(self, runs_dir):
        """Find the next run number by checking existing run directories"""
        pattern = os.path.join(runs_dir, "run_*")
        existing_runs = glob.glob(pattern)
        max_run = 0
        
        for run_dir in existing_runs:
            dirname = os.path.basename(run_dir)
            if dirname.startswith("run_"):
                try:
                    num_str = dirname[4:7]
                    num = int(num_str)
                    if num > max_run:
                        max_run = num
                except ValueError:
                    pass
        
        return max_run + 1

    def init_csv_logging(self):
        """Initialize CSV logging with run directories"""
        # Get ws_jetson path
        ws_jetson_path = os.getcwd()
        
        # Try to find ws_jetson root
        while ws_jetson_path and 'ws_jetson' in ws_jetson_path:
            if os.path.exists(os.path.join(ws_jetson_path, 'src')):
                break
            ws_jetson_path = os.path.dirname(ws_jetson_path)
        
        # Create runs directory
        runs_dir = os.path.join(ws_jetson_path, 'runs')
        os.makedirs(runs_dir, exist_ok=True)
        
        # Get run number and timestamp
        run_number = self.get_next_run_number(runs_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Create run subdirectory: run_NNN_YYYYMMDD_HHMMSS/
        self.log_dir = os.path.join(runs_dir, f"run_{run_number:03d}_{timestamp}")
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Open CSV files
        self.csv_unified = open(os.path.join(self.log_dir, 'telemetry_unified.csv'), 'w', newline='')
        self.csv_rtk_gnss = open(os.path.join(self.log_dir, 'rtk_gnss.csv'), 'w', newline='')
        self.csv_spresense_gnss = open(os.path.join(self.log_dir, 'spresense_gnss.csv'), 'w', newline='')
        self.csv_chassis_data = open(os.path.join(self.log_dir, 'chassis_data.csv'), 'w', newline='')
        self.csv_mission_state = open(os.path.join(self.log_dir, 'mission_state.csv'), 'w', newline='')
        
        # Create CSV writers
        self.writer_unified = csv.writer(self.csv_unified)
        self.writer_rtk_gnss = csv.writer(self.csv_rtk_gnss)
        self.writer_spresense_gnss = csv.writer(self.csv_spresense_gnss)
        self.writer_chassis_data = csv.writer(self.csv_chassis_data)
        self.writer_mission_state = csv.writer(self.csv_mission_state)
        
        # Write headers
        self.writer_unified.writerow([
            'Timestamp', 'Mission_Active', 'Distance_Remaining_km',
            'Spresense_Valid', 'Spresense_Lat', 'Spresense_Lon', 'Spresense_Alt', 'Spresense_Sats',
            'Ublox_Valid', 'Ublox_Lat', 'Ublox_Lon', 'Ublox_Alt', 'Ublox_Fix', 'Ublox_Err_cm', 'Ublox_Sats',
            'Chassis_Cmd_Valid', 'Cmd_Left_Speed', 'Cmd_Right_Speed', 'Cmd_Left_Dir', 'Cmd_Right_Dir',
            'Chassis_Sensors_Valid', 'Encoder_Left', 'Encoder_Right', 'Voltage_V', 'Current_A', 'Power_W',
            'Chassis_IMU_Valid', 'Accel_X', 'Accel_Y', 'Accel_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z',
            'Steering_Valid', 'Steering_Cmd',
            'Lane_Valid', 'Lane_Theta', 'Lane_B', 'Lane_Detected',
            'Dest_Valid', 'Dest_Lat', 'Dest_Lon'
        ])
        
        self.writer_rtk_gnss.writerow([
            'Timestamp', 'Valid', 'Latitude', 'Longitude', 'Altitude',
            'Fix_Quality', 'Centimeter_Error', 'Satellites'
        ])
        
        self.writer_spresense_gnss.writerow([
            'Timestamp', 'Valid', 'Latitude', 'Longitude', 'Altitude', 'Satellites'
        ])
        
        self.writer_chassis_data.writerow([
            'Timestamp',
            'Sensors_Valid', 'Encoder_Left', 'Encoder_Right', 'Voltage_V', 'Current_A', 'Power_W',
            'IMU_Valid', 'Accel_X', 'Accel_Y', 'Accel_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z',
            'Cmd_Valid', 'Cmd_Left_Speed', 'Cmd_Right_Speed', 'Cmd_Left_Dir', 'Cmd_Right_Dir'
        ])
        
        self.writer_mission_state.writerow([
            'Timestamp', 'Mission_Active', 'Distance_Remaining_km',
            'Dest_Valid', 'Dest_Latitude', 'Dest_Longitude',
            'Steering_Valid', 'Steering_Command',
            'Lane_Valid', 'Lane_Theta', 'Lane_B', 'Lane_Detected'
        ])
        
        self.get_logger().info(f'CSV logging initialized: Run #{run_number}')

    def telemetry_relay_callback(self, msg):
        """Process telemetry relay message and write to all CSV files"""
        timestamp = datetime.now().isoformat()
        
        # Write to unified CSV (all data in one file)
        self.writer_unified.writerow([
            timestamp, msg.mission_active, msg.distance_remaining_km,
            msg.spresense_valid, msg.spresense_latitude, msg.spresense_longitude, 
            msg.spresense_altitude, msg.spresense_satellites,
            msg.ublox_valid, msg.ublox_latitude, msg.ublox_longitude, 
            msg.ublox_altitude, msg.ublox_fix_quality, msg.ublox_centimeter_error, msg.ublox_satellites,
            msg.chassis_cmd_valid, msg.chassis_cmd_left_speed, msg.chassis_cmd_right_speed, 
            msg.chassis_cmd_left_direction, msg.chassis_cmd_right_direction,
            msg.chassis_sensors_valid, msg.encoder_left, msg.encoder_right, 
            msg.voltage, msg.current, msg.power_watts,
            msg.chassis_imu_valid, msg.accel_x, msg.accel_y, msg.accel_z, 
            msg.gyro_x, msg.gyro_y, msg.gyro_z,
            msg.steering_valid, msg.steering_command,
            msg.lane_valid, msg.lane_theta, msg.lane_b, msg.lane_detected,
            msg.destination_valid, msg.destination_latitude, msg.destination_longitude
        ])
        self.csv_unified.flush()
        
        # Write to RTK GNSS CSV (if valid)
        if msg.ublox_valid:
            self.writer_rtk_gnss.writerow([
                timestamp, msg.ublox_valid, msg.ublox_latitude, msg.ublox_longitude,
                msg.ublox_altitude, msg.ublox_fix_quality, msg.ublox_centimeter_error, msg.ublox_satellites
            ])
            self.csv_rtk_gnss.flush()
        
        # Write to Spresense GNSS CSV (if valid)
        if msg.spresense_valid:
            self.writer_spresense_gnss.writerow([
                timestamp, msg.spresense_valid, msg.spresense_latitude, msg.spresense_longitude,
                msg.spresense_altitude, msg.spresense_satellites
            ])
            self.csv_spresense_gnss.flush()
        
        # Write to chassis data CSV
        self.writer_chassis_data.writerow([
            timestamp,
            msg.chassis_sensors_valid, msg.encoder_left, msg.encoder_right, 
            msg.voltage, msg.current, msg.power_watts,
            msg.chassis_imu_valid, msg.accel_x, msg.accel_y, msg.accel_z, 
            msg.gyro_x, msg.gyro_y, msg.gyro_z,
            msg.chassis_cmd_valid, msg.chassis_cmd_left_speed, msg.chassis_cmd_right_speed, 
            msg.chassis_cmd_left_direction, msg.chassis_cmd_right_direction
        ])
        self.csv_chassis_data.flush()
        
        # Write to mission state CSV
        self.writer_mission_state.writerow([
            timestamp, msg.mission_active, msg.distance_remaining_km,
            msg.destination_valid, msg.destination_latitude, msg.destination_longitude,
            msg.steering_valid, msg.steering_command,
            msg.lane_valid, msg.lane_theta, msg.lane_b, msg.lane_detected
        ])
        self.csv_mission_state.flush()

    def __del__(self):
        """Close all CSV files on node shutdown"""
        try:
            self.csv_unified.close()
            self.csv_rtk_gnss.close()
            self.csv_spresense_gnss.close()
            self.csv_chassis_data.close()
            self.csv_mission_state.close()
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RoverLocalMonitoringNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
