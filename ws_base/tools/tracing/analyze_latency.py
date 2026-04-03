#!/usr/bin/env python3
"""
analyze_latency.py — ROS2 message jitter and latency analysis from collect_latency.py CSV.

Reads the CSV produced by collect_latency.py and outputs:
  - Per-topic jitter: stats on inter-arrival intervals (all topics)
  - Per-topic latency: stats on header.stamp → recv latency (topics with headers only)
  - Optional side-by-side comparison: baseline (multi-domain) vs POC (single-domain)
  - Summary CSV and optional box-plot

Requires (on base PC):
    pip3 install pandas numpy matplotlib

Usage:
    # Single run:
    python3 analyze_latency.py --csv ws_base/tools/tracing/data/poc_rpi.csv

    # Side-by-side comparison:
    python3 analyze_latency.py \\
        --baseline ws_base/tools/tracing/data/baseline_rpi.csv \\
        --poc      ws_base/tools/tracing/data/poc_rpi.csv

    # Filter topics:
    python3 analyze_latency.py --csv ... --topics /tpc_chassis_imu /tpc_rover_ctrl_cmd

    # Custom output dir:
    python3 analyze_latency.py --csv ... --out-dir ./results/

Note on metrics:
    interval_ms (inter-arrival jitter) is available for ALL topics.
    latency_ms  (header.stamp → recv)  is only available for TelemetryRelay
    (/tpc_telemetry_relay), the only project message type with a header.stamp field.
"""

import argparse
import csv
import statistics
import sys
from collections import defaultdict
from pathlib import Path

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


# ---------------------------------------------------------------------------
# CSV loading
# ---------------------------------------------------------------------------

def load_csv(path: Path, topics_filter=None):
    """
    Read collect_latency.py CSV and return:
      latencies: {topic: [latency_ms, ...]}  — rows where latency_ms is set
      intervals: {topic: [interval_ms, ...]} — inter-arrival times (all rows)
    """
    latencies = defaultdict(list)
    intervals = defaultdict(list)

    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            topic = row['topic']
            if topics_filter and topic not in topics_filter:
                continue
            try:
                if row.get('interval_ms'):
                    intervals[topic].append(float(row['interval_ms']))
            except ValueError:
                pass
            try:
                if row.get('latency_ms'):
                    latencies[topic].append(float(row['latency_ms']))
            except ValueError:
                pass

    return dict(latencies), dict(intervals)


# ---------------------------------------------------------------------------
# Statistics helpers
# ---------------------------------------------------------------------------

def _pct(data, pct):
    s = sorted(data)
    return s[max(0, int(pct / 100 * len(s)) - 1)]


def build_stats(values: list) -> dict:
    if len(values) < 2:
        return {}
    return {
        'n':       len(values),
        'mean_ms': statistics.mean(values),
        'std_ms':  statistics.stdev(values),
        'p50_ms':  _pct(values, 50),
        'p95_ms':  _pct(values, 95),
        'p99_ms':  _pct(values, 99),
        'max_ms':  max(values),
    }


def jitter_stats(intervals: dict) -> dict:
    return {topic: build_stats(vals)
            for topic, vals in intervals.items()
            if build_stats(vals)}


def latency_stats(latencies: dict) -> dict:
    return {topic: build_stats(vals)
            for topic, vals in latencies.items()
            if build_stats(vals)}


# ---------------------------------------------------------------------------
# Output helpers
# ---------------------------------------------------------------------------

WIDTH = 70


def print_section(title: str, stats_dict: dict):
    print(f'\n{"=" * WIDTH}')
    print(f'  {title}')
    print(f'{"=" * WIDTH}')
    if not stats_dict:
        print('  (no data)')
        return
    for topic in sorted(stats_dict):
        s = stats_dict[topic]
        print(f'\n  {topic}')
        print(f'    n={s["n"]}  mean={s["mean_ms"]:.2f}ms  std={s["std_ms"]:.3f}ms')
        print(f'    p50={s["p50_ms"]:.2f}ms  p95={s["p95_ms"]:.2f}ms  '
              f'p99={s["p99_ms"]:.2f}ms  max={s["max_ms"]:.2f}ms')


def save_summary_csv(rows: list, out_path: Path):
    if not rows:
        return
    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(f'\n[OK] Summary CSV → {out_path}')


def stats_to_row(label: str, topic: str, metric_type: str, s: dict) -> dict:
    return {
        'label':       label,
        'topic':       topic,
        'metric_type': metric_type,
        'n':           s['n'],
        'mean_ms':     round(s['mean_ms'], 4),
        'std_ms':      round(s['std_ms'], 4),
        'p50_ms':      round(s['p50_ms'], 4),
        'p95_ms':      round(s['p95_ms'], 4),
        'p99_ms':      round(s['p99_ms'], 4),
        'max_ms':      round(s['max_ms'], 4),
    }


def plot_jitter(raw_intervals_per_label: dict, out_path: Path):
    """
    raw_intervals_per_label = {label: {topic: [interval_ms, ...]}}
    """
    if not HAS_MATPLOTLIB:
        print('[WARN] matplotlib not available — skipping plot')
        return

    all_topics = sorted({t for d in raw_intervals_per_label.values() for t in d})
    labels = list(raw_intervals_per_label.keys())
    n = len(all_topics)
    if n == 0:
        return

    fig, axes = plt.subplots(1, n, figsize=(max(8, 4 * n), 5), sharey=False)
    if n == 1:
        axes = [axes]

    colors = ['#4c9be8', '#e85c4c']
    for ax, topic in zip(axes, all_topics):
        data = [raw_intervals_per_label[lbl].get(topic, []) for lbl in labels]
        valid_data  = [d for d in data if d]
        valid_labels = [lbl for lbl, d in zip(labels, data) if d]
        if valid_data:
            bplot = ax.boxplot(valid_data, labels=valid_labels,
                               patch_artist=True, notch=False)
            for patch, color in zip(bplot['boxes'], colors):
                patch.set_facecolor(color)
        ax.set_title(topic.lstrip('/'), fontsize=8)
        ax.set_ylabel('Inter-arrival (ms)')
        ax.grid(True, alpha=0.3)

    fig.suptitle('ROS2 Message Delivery Jitter — Single-Domain POC vs Baseline', fontsize=11)
    plt.tight_layout()
    plt.savefig(str(out_path), dpi=150)
    print(f'[OK] Plot → {out_path}')


# ---------------------------------------------------------------------------
# Analysis runner
# ---------------------------------------------------------------------------

def run_analysis(csv_path: Path, label: str, topics_filter=None):
    print(f'\n[INFO] Reading {csv_path}  (label={label})')
    lats, intvls = load_csv(csv_path, topics_filter)

    j = jitter_stats(intvls)
    l = latency_stats(lats)

    print_section(f'[{label}] INTER-ARRIVAL JITTER (all topics)', j)
    if l:
        print_section(f'[{label}] END-TO-END LATENCY via header.stamp', l)
    else:
        print(f'\n  [{label}] No header.stamp data — latency only available '
              f'for /tpc_telemetry_relay')

    return lats, intvls, j, l


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument('--csv',      type=Path, help='Single CSV file to analyze')
    ap.add_argument('--baseline', type=Path, help='Baseline CSV (multi-domain)')
    ap.add_argument('--poc',      type=Path, help='POC CSV (single-domain)')
    ap.add_argument('--topics',   nargs='*', help='Filter to these topic names only')
    ap.add_argument('--out-dir',  type=Path, default=Path('.'),
                    help='Directory for output files (default: .)')
    args = ap.parse_args()

    if not args.csv and not args.baseline and not args.poc:
        ap.error('Provide --csv or at least one of --baseline / --poc')

    topics_filter = set(args.topics) if args.topics else None
    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    summary_rows = []
    raw_jitter_per_label = {}   # {label: {topic: [interval_ms, ...]}}

    # ── Single run ────────────────────────────────────────────────────────────
    if args.csv:
        lats, intvls, j, l = run_analysis(args.csv, 'run', topics_filter)
        raw_jitter_per_label['run'] = intvls
        for topic, s in j.items():
            summary_rows.append(stats_to_row('run', topic, 'jitter', s))
        for topic, s in l.items():
            summary_rows.append(stats_to_row('run', topic, 'latency', s))

    # ── Side-by-side comparison ───────────────────────────────────────────────
    if args.baseline:
        lats, intvls, j, l = run_analysis(args.baseline, 'baseline', topics_filter)
        raw_jitter_per_label['baseline'] = intvls
        for topic, s in j.items():
            summary_rows.append(stats_to_row('baseline', topic, 'jitter', s))
        for topic, s in l.items():
            summary_rows.append(stats_to_row('baseline', topic, 'latency', s))

    if args.poc:
        lats, intvls, j, l = run_analysis(args.poc, 'poc', topics_filter)
        raw_jitter_per_label['poc'] = intvls
        for topic, s in j.items():
            summary_rows.append(stats_to_row('poc', topic, 'jitter', s))
        for topic, s in l.items():
            summary_rows.append(stats_to_row('poc', topic, 'latency', s))

    # ── Save outputs ──────────────────────────────────────────────────────────
    if summary_rows:
        save_summary_csv(summary_rows, out_dir / 'latency_summary.csv')

    if len(raw_jitter_per_label) > 1:
        plot_jitter(raw_jitter_per_label, out_dir / 'jitter_boxplot.png')


if __name__ == '__main__':
    main()
