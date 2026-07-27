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


class LazyCsv:
    """
    A CSV file that is created on its first row rather than at node startup.

    Previously all five files were opened with 'w' and given headers in
    __init__, so every launch left a run_NNN directory of header-only CSVs
    behind even when no telemetry ever arrived — which made a dead run look
    identical to a healthy one that simply had not received data yet.

    Creating on first write means the set of files present is itself a
    diagnostic: a missing CSV proves that topic never delivered anything.
    The run directory is created lazily too, so an empty run leaves nothing
    on disk and does not consume a run number.
    """

    def __init__(self, path: str, header: list, logger=None) -> None:
        self._path = path
        self._header = header
        self._logger = logger
        self._fh = None
        self._writer = None
        self._failed = False

    def writerow(self, row: list) -> None:
        if self._fh is None:
            if self._failed:
                return
            try:
                os.makedirs(os.path.dirname(self._path), exist_ok=True)
                self._fh = open(self._path, 'w', newline='')
                self._writer = csv.writer(self._fh)
                self._writer.writerow(self._header)
                if self._logger:
                    self._logger.info(
                        f"Logging {os.path.basename(self._path)} (first message received)"
                    )
            except OSError as e:
                self._failed = True
                if self._logger:
                    self._logger.error(f"Failed to open {self._path}: {e}")
                return
        self._writer.writerow(row)

    def flush(self) -> None:
        if self._fh is not None:
            self._fh.flush()

    def close(self) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None


class RoverLocalMonitoringNode(Node):
    def __init__(self):
        super().__init__('rover_local_monitoring_node')
        
        # Initialize CSV logging
        self.init_csv_logging()
        
        # Subscribe to telemetry relay (Domain 4)
        self.sub_telemetry_relay = self.create_subscription(
            TelemetryRelay,
            'tpc_telemetry_relay',
            self.telemetry_relay_callback,
            10
        )
        
        self.get_logger().info('=== Local Rover Monitoring Node (Jetson) ===')
        self.get_logger().info('Domain: 4 (base telemetry)')
        self.get_logger().info('Subscribes to: tpc_telemetry_relay (5 Hz)')
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

    def resolve_ws_jetson_root(self) -> str:
        """
        Resolve the ws_jetson workspace root.

        Walks up from the cwd for a directory named ws_jetson containing src/
        (launch_jetson_tmux.sh cds into the workspace), else falls back to an
        explicit ~/almondmatcha/ws_jetson.

        The explicit fallback matters: the previous loop left the path as the
        bare cwd when it could not find ws_jetson, scattering run_NNN
        directories wherever the process happened to start.
        """
        path = os.path.abspath(os.getcwd())
        while True:
            if os.path.basename(path) == 'ws_jetson' and os.path.isdir(os.path.join(path, 'src')):
                return path
            parent = os.path.dirname(path)
            if parent == path:
                break
            path = parent

        fallback = os.path.expanduser('~/almondmatcha/ws_jetson')
        self.get_logger().warn(
            f'Not running from inside ws_jetson (cwd={os.getcwd()}) — logging to {fallback}'
        )
        return fallback

    def init_csv_logging(self):
        """
        Prepare CSV logging. Nothing is written to disk here — the run
        directory and each file are created on first use (see LazyCsv).
        """
        runs_dir = os.path.join(self.resolve_ws_jetson_root(), 'runs')

        run_number = self.get_next_run_number(runs_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_dir = os.path.join(runs_dir, f"run_{run_number:03d}_{timestamp}")

        log = self.get_logger()

        self.csv_unified = LazyCsv(
            os.path.join(self.log_dir, 'telemetry_unified.csv'), [
                'Timestamp', 'Mission_Active', 'Distance_Remaining_km',
                'Spresense_Valid', 'Spresense_Lat', 'Spresense_Lon', 'Spresense_Alt', 'Spresense_Sats',
                'Ublox_Valid', 'Ublox_Lat', 'Ublox_Lon', 'Ublox_Alt', 'Ublox_Fix', 'Ublox_Err_cm', 'Ublox_Sats',
                'Chassis_Cmd_Valid', 'Cmd_Left_Speed', 'Cmd_Right_Speed', 'Cmd_Steer_Dir', 'Cmd_Drive_Dir',
                'Chassis_Sensors_Valid', 'Encoder_Left', 'Encoder_Right', 'Voltage_V', 'Current_A', 'Power_W',
                'Chassis_IMU_Valid', 'Accel_X', 'Accel_Y', 'Accel_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z',
                'Steering_Valid', 'Steering_Cmd',
                'Lane_Valid', 'Lane_Theta', 'Lane_B', 'Lane_Detected',
                'Dest_Valid', 'Dest_Lat', 'Dest_Lon'
            ], log)

        self.csv_rtk_gnss = LazyCsv(
            os.path.join(self.log_dir, 'rtk_gnss.csv'), [
                'Timestamp', 'Valid', 'Latitude', 'Longitude', 'Altitude',
                'Fix_Quality', 'Centimeter_Error', 'Satellites'
            ], log)

        self.csv_spresense_gnss = LazyCsv(
            os.path.join(self.log_dir, 'spresense_gnss.csv'), [
                'Timestamp', 'Valid', 'Latitude', 'Longitude', 'Altitude', 'Satellites'
            ], log)

        self.csv_chassis_data = LazyCsv(
            os.path.join(self.log_dir, 'chassis_data.csv'), [
                'Timestamp',
                'Sensors_Valid', 'Encoder_Left', 'Encoder_Right', 'Voltage_V', 'Current_A', 'Power_W',
                'IMU_Valid', 'Accel_X', 'Accel_Y', 'Accel_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z',
                'Cmd_Valid', 'Cmd_Left_Speed', 'Cmd_Right_Speed', 'Cmd_Steer_Dir', 'Cmd_Drive_Dir'
            ], log)

        self.csv_mission_state = LazyCsv(
            os.path.join(self.log_dir, 'mission_state.csv'), [
                'Timestamp', 'Mission_Active', 'Distance_Remaining_km',
                'Dest_Valid', 'Dest_Latitude', 'Dest_Longitude',
                'Steering_Valid', 'Steering_Command',
                'Lane_Valid', 'Lane_Theta', 'Lane_B', 'Lane_Detected'
            ], log)

        log.info(
            f'CSV logging armed: Run #{run_number} — files are created on first '
            f'message, at {self.log_dir}'
        )

    def telemetry_relay_callback(self, msg):
        """Process telemetry relay message and write to all CSV files"""
        timestamp = datetime.now().isoformat()
        
        # Write to unified CSV (all data in one file)
        self.csv_unified.writerow([
            timestamp, msg.mission_active, msg.distance_remaining_km,
            msg.spresense_valid, msg.spresense_latitude, msg.spresense_longitude, 
            msg.spresense_altitude, msg.spresense_satellites,
            msg.ublox_valid, msg.ublox_latitude, msg.ublox_longitude, 
            msg.ublox_altitude, msg.ublox_fix_quality, msg.ublox_centimeter_error, msg.ublox_satellites,
            msg.chassis_cmd_valid, msg.chassis_cmd_left_speed, msg.chassis_cmd_right_speed, 
            msg.chassis_cmd_steer_dir, msg.chassis_cmd_drive_dir,
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
            self.csv_rtk_gnss.writerow([
                timestamp, msg.ublox_valid, msg.ublox_latitude, msg.ublox_longitude,
                msg.ublox_altitude, msg.ublox_fix_quality, msg.ublox_centimeter_error, msg.ublox_satellites
            ])
            self.csv_rtk_gnss.flush()
        
        # Write to Spresense GNSS CSV (if valid)
        if msg.spresense_valid:
            self.csv_spresense_gnss.writerow([
                timestamp, msg.spresense_valid, msg.spresense_latitude, msg.spresense_longitude,
                msg.spresense_altitude, msg.spresense_satellites
            ])
            self.csv_spresense_gnss.flush()
        
        # Write to chassis data CSV
        self.csv_chassis_data.writerow([
            timestamp,
            msg.chassis_sensors_valid, msg.encoder_left, msg.encoder_right, 
            msg.voltage, msg.current, msg.power_watts,
            msg.chassis_imu_valid, msg.accel_x, msg.accel_y, msg.accel_z, 
            msg.gyro_x, msg.gyro_y, msg.gyro_z,
            msg.chassis_cmd_valid, msg.chassis_cmd_left_speed, msg.chassis_cmd_right_speed, 
            msg.chassis_cmd_steer_dir, msg.chassis_cmd_drive_dir
        ])
        self.csv_chassis_data.flush()
        
        # Write to mission state CSV
        self.csv_mission_state.writerow([
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
