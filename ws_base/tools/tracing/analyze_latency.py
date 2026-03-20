#!/usr/bin/env python3
"""
analyze_latency.py — ROS2 message latency and jitter analysis from LTTng CTF traces
                     (Single-domain POC vs baseline comparison)

Requires:
    pip install babeltrace2  (or apt install python3-babeltrace2)
    pip install pandas numpy matplotlib

Usage:
    python3 analyze_latency.py --trace-dir ./traces/rpi_<timestamp>
    python3 analyze_latency.py --trace-dir ./traces/rpi_<timestamp> --topics tpc_chassis_imu tpc_chassis_sensors
    python3 analyze_latency.py --baseline ./traces/baseline --poc ./traces/poc  # side-by-side

How it works:
    ros2_tracing instruments rclcpp at compile time. When the tracer is active,
    every publish() call emits a ros2:rclcpp_publish event with a pointer to the
    message. The matching ros2:dispatch_subscription_callback event carries the
    same pointer on the subscriber side, allowing per-message latency computation.

Output:
    - Per-topic latency stats (mean, p50, p95, p99, max, std_dev)
    - Per-topic jitter (std dev of inter-publish intervals)
    - CSV: latency_results.csv
    - Plot: latency_boxplot.png (if matplotlib available)
"""

import argparse
import sys
import csv
import statistics
from pathlib import Path
from collections import defaultdict

try:
    import bt2  # babeltrace2 Python bindings
except ImportError:
    sys.exit(
        "[ERROR] babeltrace2 Python bindings not found.\n"
        "  Install: sudo apt install python3-babeltrace2  OR  pip install bt2"
    )

try:
    import pandas as pd
    import numpy as np
    HAS_PANDAS = True
except ImportError:
    HAS_PANDAS = False

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


# ---------------------------------------------------------------------------
# Trace parsing helpers
# ---------------------------------------------------------------------------

def parse_trace(trace_dir: Path):
    """
    Walk a CTF trace directory and collect:
      - publish events:   {timestamp_ns, topic, msg_ptr}
      - callback events:  {timestamp_ns, topic, msg_ptr}

    Returns two dicts keyed by (topic, msg_ptr):
      publish_ts[topic][msg_ptr] = ns
      deliver_ts[topic][msg_ptr] = ns
    """
    msg_graph = bt2.TraceCollectionMessageIterator(str(trace_dir))

    publish_ts  = defaultdict(dict)  # topic -> {ptr -> ts_ns}
    deliver_ts  = defaultdict(dict)  # topic -> {ptr -> ts_ns}
    pub_times   = defaultdict(list)  # topic -> [ts_ns, ...]  (for jitter)

    for msg in msg_graph:
        if not isinstance(msg, bt2._EventMessageConst):
            continue

        ev   = msg.event
        name = ev.name
        ts   = msg.default_clock_snapshot.ns_from_origin

        # ros2:rclcpp_publish — publisher side
        if name in ("ros2:rclcpp_publish", "ros2:publish"):
            topic = str(ev.payload_field.get("topic_name", ""))
            ptr   = int(ev.payload_field.get("message", 0))
            if topic:
                publish_ts[topic][ptr] = ts
                pub_times[topic].append(ts)

        # ros2:dispatch_subscription_callback — subscriber side
        elif name in ("ros2:dispatch_subscription_callback",
                      "ros2:callback_start"):
            topic = str(ev.payload_field.get("topic_name", ""))
            ptr   = int(ev.payload_field.get("message", 0))
            if topic and ptr:
                # Only record first delivery (could be multiple subscribers)
                if ptr not in deliver_ts[topic]:
                    deliver_ts[topic][ptr] = ts

    return publish_ts, deliver_ts, pub_times


def compute_latency(publish_ts, deliver_ts):
    """Match publish → deliver timestamps and compute latency in ms."""
    results = {}
    for topic in publish_ts:
        latencies_ms = []
        for ptr, pub_t in publish_ts[topic].items():
            if ptr in deliver_ts.get(topic, {}):
                lat_ms = (deliver_ts[topic][ptr] - pub_t) / 1e6
                if 0 < lat_ms < 5000:   # sanity-filter: drop obviously wrong values
                    latencies_ms.append(lat_ms)
        if latencies_ms:
            results[topic] = latencies_ms
    return results


def compute_jitter(pub_times):
    """
    Jitter = std-dev of inter-publish intervals (ms).
    Represents how irregular the publishing cadence is.
    """
    jitter = {}
    for topic, times in pub_times.items():
        if len(times) < 2:
            continue
        sorted_t = sorted(times)
        intervals = [(sorted_t[i+1] - sorted_t[i]) / 1e6
                     for i in range(len(sorted_t) - 1)]
        if intervals:
            jitter[topic] = {
                "mean_interval_ms": statistics.mean(intervals),
                "jitter_std_ms":    statistics.stdev(intervals) if len(intervals) > 1 else 0.0,
                "min_interval_ms":  min(intervals),
                "max_interval_ms":  max(intervals),
                "msg_count":        len(times),
            }
    return jitter


def print_stats(latency_results, jitter_results, label=""):
    tag = f"[{label}] " if label else ""
    print(f"\n{'='*70}")
    print(f"  {tag}LATENCY RESULTS")
    print(f"{'='*70}")
    for topic, lats in sorted(latency_results.items()):
        if not lats:
            continue
        s = sorted(lats)
        n = len(s)
        p = lambda pct: s[int(pct / 100 * n)]
        mean   = statistics.mean(s)
        stddev = statistics.stdev(s) if n > 1 else 0.0
        print(f"\n  Topic: {topic}")
        print(f"    N={n}  mean={mean:.3f}ms  std={stddev:.3f}ms")
        print(f"    p50={p(50):.3f}ms  p95={p(95):.3f}ms  p99={p(99):.3f}ms  max={max(s):.3f}ms")

    print(f"\n{'='*70}")
    print(f"  {tag}JITTER RESULTS (inter-publish interval std-dev)")
    print(f"{'='*70}")
    for topic, j in sorted(jitter_results.items()):
        print(f"\n  Topic: {topic}")
        print(f"    msgs={j['msg_count']}  mean_interval={j['mean_interval_ms']:.2f}ms"
              f"  jitter_std={j['jitter_std_ms']:.3f}ms"
              f"  range=[{j['min_interval_ms']:.2f}, {j['max_interval_ms']:.2f}]ms")


def save_csv(latency_results, jitter_results, out_path: Path, label=""):
    rows = []
    for topic, lats in latency_results.items():
        if not lats:
            continue
        s = sorted(lats)
        n = len(s)
        p = lambda pct: s[int(pct / 100 * n)]
        j = jitter_results.get(topic, {})
        rows.append({
            "label":           label,
            "topic":           topic,
            "n":               n,
            "latency_mean_ms": round(statistics.mean(s), 4),
            "latency_std_ms":  round(statistics.stdev(s) if n > 1 else 0.0, 4),
            "latency_p50_ms":  round(p(50), 4),
            "latency_p95_ms":  round(p(95), 4),
            "latency_p99_ms":  round(p(99), 4),
            "latency_max_ms":  round(max(s), 4),
            "jitter_std_ms":   round(j.get("jitter_std_ms", float("nan")), 4),
            "mean_interval_ms":round(j.get("mean_interval_ms", float("nan")), 4),
        })
    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(f"\n[OK] CSV saved: {out_path}")


def plot_boxplot(latency_results_dict: dict, out_path: Path):
    """latency_results_dict = {label: {topic: [latency_ms, ...]}}"""
    if not HAS_MATPLOTLIB:
        return
    # Collect all topics across all labels
    all_topics = sorted({t for lats in latency_results_dict.values() for t in lats})
    labels     = list(latency_results_dict.keys())
    n_topics   = len(all_topics)
    if n_topics == 0:
        return

    fig, axes = plt.subplots(1, n_topics, figsize=(max(8, 4 * n_topics), 6), sharey=False)
    if n_topics == 1:
        axes = [axes]

    for ax, topic in zip(axes, all_topics):
        data  = [latency_results_dict[lbl].get(topic, []) for lbl in labels]
        bplot = ax.boxplot([d for d in data if d], labels=[l for l, d in zip(labels, data) if d],
                           patch_artist=True, notch=False)
        colors = ["#4c9be8", "#e85c4c"]
        for patch, color in zip(bplot["boxes"], colors):
            patch.set_facecolor(color)
        ax.set_title(topic.lstrip("/"), fontsize=9)
        ax.set_ylabel("Latency (ms)")
        ax.grid(True, alpha=0.3)

    fig.suptitle("ROS2 Message Latency — Single-Domain POC vs Baseline", fontsize=11)
    plt.tight_layout()
    plt.savefig(str(out_path), dpi=150)
    print(f"[OK] Plot saved: {out_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def run_single(trace_dir: Path, label: str, out_dir: Path, topics_filter=None):
    print(f"\n[INFO] Parsing trace: {trace_dir}  (label={label})")
    pub_ts, del_ts, pub_times = parse_trace(trace_dir)

    if topics_filter:
        pub_ts    = {t: v for t, v in pub_ts.items()    if t in topics_filter}
        del_ts    = {t: v for t, v in del_ts.items()    if t in topics_filter}
        pub_times = {t: v for t, v in pub_times.items() if t in topics_filter}

    latency  = compute_latency(pub_ts, del_ts)
    jitter   = compute_jitter(pub_times)
    print_stats(latency, jitter, label=label)
    save_csv(latency, jitter, out_dir / f"latency_{label}.csv", label=label)
    return latency, jitter


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--trace-dir", type=Path,
                        help="Single CTF trace directory to analyse")
    parser.add_argument("--baseline", type=Path,
                        help="Baseline CTF trace (multi-domain)")
    parser.add_argument("--poc", type=Path,
                        help="POC CTF trace (single-domain)")
    parser.add_argument("--topics", nargs="*",
                        help="Filter to specific topic names (space-separated)")
    parser.add_argument("--out-dir", type=Path, default=Path("."),
                        help="Directory for CSV and plot output (default: .)")
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    results = {}

    if args.trace_dir:
        lats, _ = run_single(args.trace_dir, "trace", args.out_dir, args.topics)
        results["trace"] = lats

    if args.baseline:
        lats, _ = run_single(args.baseline, "baseline", args.out_dir, args.topics)
        results["baseline"] = lats

    if args.poc:
        lats, _ = run_single(args.poc, "poc", args.out_dir, args.topics)
        results["poc"] = lats

    if len(results) > 1 and HAS_MATPLOTLIB:
        plot_boxplot(results, args.out_dir / "latency_boxplot.png")

    if not any([args.trace_dir, args.baseline, args.poc]):
        parser.print_help()


if __name__ == "__main__":
    main()
