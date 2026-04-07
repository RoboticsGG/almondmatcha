#!/usr/bin/env python3
"""
collect_latency.py — ROS2 latency and jitter collector (no LTTng required).

Subscribes to ROS2 topics using rclpy. Auto-discovers message types via DDS.
Records per-message timing to a CSV file.

CSV columns:
  topic          — topic name
  recv_time_s    — wall-clock time when callback fired (seconds since epoch)
  header_stamp_s — message header.stamp in seconds (if the type has a header), else empty
  latency_ms     — recv_time - header_stamp in ms (if applicable), else empty
  interval_ms    — time since previous message on the same topic (jitter measurement)

Usage (run on the SBC while ROS2 nodes are running):
  source /opt/ros/humble/setup.bash
  source ~/almondmatcha/common_ifaces/install/setup.bash
  source ~/almondmatcha/ws_rpi/install/setup.bash   # or ws_jetson
  export ROS_DOMAIN_ID=5
  python3 ~/almondmatcha/ws_base/tools/tracing/collect_latency.py \\
      --topics /tpc_chassis_imu /tpc_chassis_sensors /tpc_chassis_cmd \\
               /tpc_gnss_spresense /tpc_gnss_ublox /tpc_rover_ctrl_cmd \\
               /tpc_telemetry_relay \\
      --out ~/ros2_traces/latency_rpi.csv

Press Ctrl-C to stop.

Note on latency vs jitter:
  - latency_ms is only populated for topics whose message type has a 'header.stamp'
    field (e.g. TelemetryRelay). For all other topics, only interval_ms is recorded.
  - interval_ms (inter-arrival time standard deviation) is the primary jitter metric
    and works for ALL topics regardless of message type.
"""

import argparse
import csv
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


class LatencyCollector(Node):
    def __init__(self, topics: list, out_path: str):
        super().__init__('latency_collector')
        self._pending = set(topics)
        self._lock = threading.Lock()
        self._last_recv: dict = {}
        self._subs = []

        os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
        self._f = open(out_path, 'w', newline='')
        self._w = csv.writer(self._f)
        self._w.writerow(['topic', 'recv_time_s', 'header_stamp_s', 'latency_ms', 'interval_ms'])
        self._f.flush()

        self.get_logger().info(f'Writing to {out_path}')
        self.get_logger().info(f'Waiting for {len(topics)} topic(s) to appear on the DDS bus...')

        # Poll for new topics once per second until all are subscribed
        self._discovery_timer = self.create_timer(1.0, self._discover)

    def _discover(self):
        if not self._pending:
            self._discovery_timer.destroy()
            self.get_logger().info('All topics subscribed. Collecting...')
            return

        known = dict(self.get_topic_names_and_types())
        done = set()

        for topic in list(self._pending):
            if topic not in known or not known[topic]:
                continue  # publisher not seen yet

            msg_type_str = known[topic][0]
            try:
                msg_cls = get_message(msg_type_str)
            except Exception as e:
                self.get_logger().warn(
                    f'Cannot load type {msg_type_str} for {topic}: {e} — skipping'
                )
                done.add(topic)
                continue

            # Match publisher reliability so BEST_EFFORT publishers (e.g. camera_stream_node)
            # are not silently dropped by a mismatched RELIABLE subscription.
            try:
                pub_infos = self.get_publishers_info_by_topic(topic)
                rel = (pub_infos[0].qos_profile.reliability
                       if pub_infos
                       else QoSReliabilityPolicy.RELIABLE)
            except Exception:
                rel = QoSReliabilityPolicy.RELIABLE

            qos = QoSProfile(
                depth=10,
                reliability=rel,
                durability=QoSDurabilityPolicy.VOLATILE,
            )

            self.create_subscription(msg_cls, topic, self._make_cb(topic), qos)
            self.get_logger().info(f'  Subscribed: {topic}  [{msg_type_str}]')
            done.add(topic)

        self._pending -= done

    def _make_cb(self, topic: str):
        def cb(msg):
            recv = time.time()

            with self._lock:
                last = self._last_recv.get(topic)
                interval = f'{(recv - last) * 1000.0:.3f}' if last is not None else ''
                self._last_recv[topic] = recv

            stamp_s = ''
            latency = ''
            try:
                s = msg.header.stamp
                stamp_f = s.sec + s.nanosec * 1e-9
                latency = f'{(recv - stamp_f) * 1000.0:.3f}'
                stamp_s = f'{stamp_f:.9f}'
            except AttributeError:
                pass  # message type has no header.stamp

            with self._lock:
                self._w.writerow([topic, f'{recv:.9f}', stamp_s, latency, interval])
                self._f.flush()

        return cb

    def close(self):
        self._f.close()
        self.get_logger().info('CSV closed.')


def main():
    ap = argparse.ArgumentParser(
        description='ROS2 latency/jitter CSV collector — no LTTng required'
    )
    ap.add_argument('--topics', nargs='+', required=True,
                    help='ROS2 topic names to monitor')
    ap.add_argument('--out', required=True,
                    help='Output CSV file path')
    args = ap.parse_args()

    rclpy.init()
    node = LatencyCollector(args.topics, args.out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
