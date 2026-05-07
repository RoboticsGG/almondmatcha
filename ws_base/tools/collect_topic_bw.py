#!/usr/bin/env python3
"""
collect_topic_bw.py — Per-topic bandwidth collector for ROS2.

Auto-discovers all topics visible on the DDS bus, measures byte throughput
using the CDR-serialised message size (the same encoding DDS uses on the
wire), and writes per-interval summaries to CSV.

CSV columns:
  timestamp   — ISO 8601 UTC wall-clock at end of aggregation interval
  elapsed_s   — seconds since collector started
  topic       — ROS2 topic name
  msg_count   — messages received in this interval
  bytes       — total serialised bytes in this interval
  bps         — bytes / interval_s
  msg_per_s   — msg_count / interval_s

Usage (run on any host with ROS2 sourced and ROS_DOMAIN_ID set):
  source /opt/ros/humble/setup.bash
  source ~/almondmatcha/ws_base/install/setup.bash
  export ROS_DOMAIN_ID=5
  export FASTRTPS_DEFAULT_PROFILES_FILE=~/almondmatcha/ws_base/fastdds_base.xml
  python3 collect_topic_bw.py --out ~/ros2_traces/topic_bw.csv

Press Ctrl-C or send SIGTERM to stop.
"""

import argparse
import csv
import signal
import threading
import time
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

# ROS2 infrastructure topics — not useful to measure
_SKIP_PREFIXES = ('/rosout', '/parameter_events', '/tf', '/tf_static')

_FIELDS = ['timestamp', 'elapsed_s', 'topic', 'msg_count', 'bytes', 'bps', 'msg_per_s']


class TopicBwCollector(Node):
    def __init__(self, topics_filter, interval_s: float, out_path: Path):
        super().__init__('topic_bw_collector')
        self._interval = interval_s
        self._filter   = set(topics_filter) if topics_filter else None
        self._lock     = threading.Lock()
        self._counts: dict = defaultdict(int)
        self._bytes:  dict = defaultdict(int)
        self._known:  set  = set()
        self._subs         = []
        self._t0           = time.time()

        out_path.parent.mkdir(parents=True, exist_ok=True)
        self._f = open(out_path, 'w', newline='')
        self._w = csv.writer(self._f)
        self._w.writerow(_FIELDS)
        self._f.flush()
        print(f'[INFO] Writing to {out_path}')

        # Discover new topics every 3 s; write CSV every interval_s
        self.create_timer(3.0,        self._discover)
        self.create_timer(interval_s, self._flush_interval)

    # ── topic discovery ────────────────────────────────────────────────────────

    def _discover(self):
        for name, types in self.get_topic_names_and_types():
            if name in self._known:
                continue
            if any(name.startswith(p) for p in _SKIP_PREFIXES):
                self._known.add(name)
                continue
            if self._filter and name not in self._filter:
                continue
            if not types:
                continue
            self._subscribe(name, types[0])

    def _subscribe(self, name: str, type_str: str):
        try:
            msg_class = get_message(type_str)
        except Exception as exc:
            self.get_logger().warn(f'[bw] cannot resolve type {type_str} for {name}: {exc}')
            self._known.add(name)
            return

        # Match publisher reliability to avoid DDS QoS incompatibility rejection
        try:
            pub_infos = self.get_publishers_info_by_topic(name)
            rel = (pub_infos[0].qos_profile.reliability
                   if pub_infos
                   else QoSReliabilityPolicy.BEST_EFFORT)
        except Exception:
            rel = QoSReliabilityPolicy.BEST_EFFORT

        qos = QoSProfile(
            depth=10,
            reliability=rel,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        def _cb(msg, t=name):
            try:
                sz = len(serialize_message(msg))
            except Exception:
                sz = 0
            with self._lock:
                self._counts[t] += 1
                self._bytes[t]  += sz

        try:
            sub = self.create_subscription(msg_class, name, _cb, qos)
            self._subs.append(sub)
            self._known.add(name)
            self.get_logger().info(f'[bw] subscribed: {name}  ({type_str})')
        except Exception as exc:
            self.get_logger().warn(f'[bw] subscribe failed for {name}: {exc}')
            self._known.add(name)

    # ── periodic CSV flush ─────────────────────────────────────────────────────

    def _flush_interval(self):
        now     = datetime.now(timezone.utc).isoformat()
        elapsed = time.time() - self._t0
        with self._lock:
            counts = dict(self._counts)
            bytes_ = dict(self._bytes)
            self._counts.clear()
            self._bytes.clear()
        for topic in sorted(counts):
            n = counts[topic]
            b = bytes_.get(topic, 0)
            self._w.writerow([
                now,
                f'{elapsed:.3f}',
                topic,
                n,
                b,
                f'{b / self._interval:.1f}',
                f'{n / self._interval:.3f}',
            ])
        self._f.flush()

    def close(self):
        """Flush the final partial interval and close the file."""
        self._flush_interval()
        self._f.close()


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument('--out',      type=Path,  default=Path('/tmp/topic_bw.csv'),
                    help='Output CSV path (default: /tmp/topic_bw.csv)')
    ap.add_argument('--interval', type=float, default=1.0,
                    help='Aggregation interval in seconds (default: 1.0)')
    ap.add_argument('--topics',   nargs='*',
                    help='Restrict to these topic names; default: all discovered topics')
    args = ap.parse_args()

    rclpy.init()
    node = TopicBwCollector(args.topics, args.interval, args.out)
    print('[INFO] Discovering topics every 3 s. Press Ctrl-C or send SIGTERM to stop.')

    def _shutdown(sig, _frame):
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT,  _shutdown)

    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.close()
        print(f'[OK] topic_bw CSV closed: {args.out}')


if __name__ == '__main__':
    main()
