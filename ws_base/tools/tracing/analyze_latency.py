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


Unified timeline (--merge):
    Synchronises all four collectors onto one time axis and produces a single
    multi-panel PNG (unified_timeline.png).  All Linux hosts use NTP so their
    wall-clocks are already aligned to ~1–10 ms.  The STM32 has no RTC; its
    wall_clock column is the base-PC receive time and is used directly.

    python3 analyze_latency.py --merge \\
        --latency-rpi    ws_base/tools/tracing/data/poc_latency_rpi.csv \\
        --latency-jetson ws_base/tools/tracing/data/poc_latency_jetson.csv \\
        --stm32          ~/ros2_traces/stm32_memory_poc.csv \\
        --net-rpi        ws_base/tools/monitoring/data/poc_net_stats_rpi.csv \\
        --net-jetson     ws_base/tools/monitoring/data/poc_net_stats_jetson.csv \\
        --out-dir        ws_base/tools/tracing/results/
"""

import argparse
import csv
import statistics
import sys
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


# ---------------------------------------------------------------------------
# CSV loading — jitter/latency (existing)
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
# CSV loading — merge mode (time-synced)
# ---------------------------------------------------------------------------

def _iso_to_epoch(s: str) -> float:
    """Parse datetime.utcnow().isoformat() string → UNIX epoch float (UTC).
    Both collect_net_stats.py and collect_stm32_memory.py write naive UTC ISO
    strings. We add UTC tzinfo so timestamp() returns the correct epoch."""
    dt = datetime.fromisoformat(s)
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return dt.timestamp()


def load_latency_timeseries(path: Path, topics_filter=None):
    """Load collect_latency.py CSV as a per-topic time-series.
    recv_time_s is already a UNIX epoch float (time.time() on the SBC) — no
    conversion needed; NTP keeps RPi/Jetson clocks within ~1–10 ms of UTC.
    Returns: {topic: [(t_epoch_s, interval_ms), ...]}
    """
    series = defaultdict(list)
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            topic = row.get('topic', '')
            if topics_filter and topic not in topics_filter:
                continue
            try:
                t = float(row['recv_time_s'])
                if row.get('interval_ms'):
                    series[topic].append((t, float(row['interval_ms'])))
            except (ValueError, KeyError):
                pass
    return dict(series)


def load_stm32_csv(path: Path):
    """Load collect_stm32_memory.py CSV.
    wall_clock is datetime.utcnow().isoformat() on the base PC — used directly
    as the absolute timestamp (NTP-synced).  ts_ms (STM32 uptime) is kept for
    reference but wall_clock is the sync anchor.
    Returns: list of dicts with keys: t_epoch, node, heap_used_kb, heap_max_kb
    """
    rows = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            try:
                rows.append({
                    't_epoch':     _iso_to_epoch(row['wall_clock']),
                    'node':        row.get('node', ''),
                    'heap_used_kb': int(row['heap_used']) // 1024
                                   if row.get('heap_used') else None,
                    'heap_max_kb':  int(row['heap_max'])  // 1024
                                   if row.get('heap_max')  else None,
                    'ts_ms':       int(row['ts_ms']) if row.get('ts_ms') else None,
                })
            except (ValueError, KeyError):
                pass
    return rows


def load_net_csv(path: Path, label: str):
    """Load collect_net_stats.py CSV.
    timestamp is datetime.utcnow().isoformat() on the SBC (NTP-synced).
    rx_bps / tx_bps are bytes/s already computed by the collector.
    Returns: list of dicts with keys: t_epoch, rx_kbps, tx_kbps, label
    """
    rows = []
    # Strip NUL bytes before parsing — they appear as zero-filled blocks when
    # the collector process is killed mid-buffer-flush by the OS.
    with open(path, newline='') as f:
        clean = (line.replace('\x00', '') for line in f)
        for row in csv.DictReader(clean):
            if not row.get('timestamp'):
                continue
            try:
                rows.append({
                    't_epoch':  _iso_to_epoch(row['timestamp']),
                    'rx_kbps':  float(row['rx_bps']) / 1024 if row.get('rx_bps') else 0.0,
                    'tx_kbps':  float(row['tx_bps']) / 1024 if row.get('tx_bps') else 0.0,
                    'label':    label,
                })
            except (ValueError, KeyError):
                pass
    return rows


def load_topic_bw_csv(path: Path):
    """Load collect_topic_bw.py CSV.
    Returns: {topic: [(t_epoch_s, bps), ...]}
    """
    from collections import defaultdict
    result: dict = defaultdict(list)
    with open(path, newline='') as f:
        clean = (line.replace('\x00', '') for line in f)
        for row in csv.DictReader(clean):
            if not row.get('timestamp'):
                continue
            try:
                result[row['topic']].append((
                    _iso_to_epoch(row['timestamp']),
                    float(row['bps']),
                ))
            except (ValueError, KeyError):
                pass
    return dict(result) if result else None


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


def _rolling_mean(xs: list, ys: list, window: int = 15):
    """Simple centered rolling mean — no numpy needed."""
    half = window // 2
    ys_out = []
    for i in range(len(ys)):
        lo = max(0, i - half)
        hi = min(len(ys), i + half + 1)
        ys_out.append(sum(ys[lo:hi]) / (hi - lo))
    return xs, ys_out


def plot_unified_timeline(
    latency_rpi=None,       # {topic: [(t_epoch_s, interval_ms), ...]}
    latency_jetson=None,
    stm32_rows=None,        # [{'t_epoch', 'node', 'heap_used_kb', 'heap_max_kb'}, ...]
    net_rpi=None,           # [{'t_epoch', 'rx_kbps', 'tx_kbps'}, ...]
    net_jetson=None,
    topic_bw=None,          # {topic: [(t_epoch_s, bps), ...]}
    out_path=None,
):
    """Synchronise all collectors onto a common elapsed-seconds x-axis and
    produce a stacked multi-panel PNG.

    Time sync rationale:
      - collect_latency.py  → recv_time_s is time.time() (NTP epoch float)
      - collect_net_stats.py → timestamp is datetime.utcnow() ISO (NTP)
      - collect_stm32_memory.py → wall_clock is datetime.utcnow() ISO on base PC (NTP)
        STM32 ts_ms is board uptime ms (no RTC); wall_clock is the sync anchor.
    All Linux hosts are NTP-synced → clocks differ by ~1–10 ms, acceptable for
    a seconds-scale timeline.  Global t_min is subtracted so x=0 is the first
    recorded event across all datasets.
    """
    if not HAS_MATPLOTLIB:
        print('[WARN] matplotlib not available — skipping unified timeline plot')
        return

    if out_path is None:
        out_path = Path('unified_timeline.png')

    # Determine which panels we have data for
    panels = []
    if latency_rpi:
        panels.append('rpi_jitter')
    if latency_jetson:
        panels.append('jetson_jitter')
    if stm32_rows:
        panels.append('stm32_heap')
    if net_rpi or net_jetson:
        panels.append('net_bw')
    if topic_bw:
        panels.append('topic_bw')

    if not panels:
        print('[WARN] --merge: no data provided — nothing to plot')
        return

    # Find global t_min across every dataset
    all_t = []
    for src in (latency_rpi, latency_jetson):
        if src:
            for pts in src.values():
                all_t.extend(t for t, _ in pts)
    for src in (stm32_rows, net_rpi, net_jetson):
        if src:
            all_t.extend(r['t_epoch'] for r in src)
    if topic_bw:
        for pts in topic_bw.values():
            all_t.extend(t for t, _ in pts)
    t_min = min(all_t)

    n = len(panels)
    fig, raw_axes = plt.subplots(n, 1, figsize=(15, 4 * n), sharex=True)
    axes = [raw_axes] if n == 1 else list(raw_axes)
    ax_map = dict(zip(panels, axes))
    cmap = plt.cm.tab10.colors

    # ── RPi jitter ────────────────────────────────────────────────────────────
    if 'rpi_jitter' in ax_map:
        ax = ax_map['rpi_jitter']
        for i, (topic, pts) in enumerate(sorted(latency_rpi.items())):
            if not pts:
                continue
            xs = [t - t_min for t, _ in pts]
            ys = [v for _, v in pts]
            c = cmap[i % len(cmap)]
            ax.scatter(xs, ys, s=5, alpha=0.30, color=c)
            _, ys_s = _rolling_mean(xs, ys)
            ax.plot(xs, ys_s, linewidth=1.3, color=c, label=topic.lstrip('/'))
        ax.set_ylabel('Interval (ms)')
        ax.set_title('RPi — per-topic inter-arrival interval')
        ax.legend(fontsize=7, loc='upper right')
        ax.grid(True, alpha=0.25)

    # ── Jetson jitter ─────────────────────────────────────────────────────────
    if 'jetson_jitter' in ax_map:
        ax = ax_map['jetson_jitter']
        for i, (topic, pts) in enumerate(sorted(latency_jetson.items())):
            if not pts:
                continue
            xs = [t - t_min for t, _ in pts]
            ys = [v for _, v in pts]
            c = cmap[i % len(cmap)]
            ax.scatter(xs, ys, s=5, alpha=0.30, color=c)
            _, ys_s = _rolling_mean(xs, ys)
            ax.plot(xs, ys_s, linewidth=1.3, color=c, label=topic.lstrip('/'))
        ax.set_ylabel('Interval (ms)')
        ax.set_title('Jetson — per-topic inter-arrival interval')
        ax.legend(fontsize=7, loc='upper right')
        ax.grid(True, alpha=0.25)

    # ── STM32 heap ────────────────────────────────────────────────────────────
    if 'stm32_heap' in ax_map:
        ax = ax_map['stm32_heap']
        nodes = sorted({r['node'] for r in stm32_rows})
        for i, node in enumerate(nodes):
            c = cmap[i % len(cmap)]
            used_pts = [(r['t_epoch'] - t_min, r['heap_used_kb'])
                        for r in stm32_rows
                        if r['node'] == node and r['heap_used_kb'] is not None]
            if used_pts:
                xs, ys = zip(*used_pts)
                ax.plot(xs, ys, linewidth=1.5, color=c, label=f'{node} used')
            max_pts = [(r['t_epoch'] - t_min, r['heap_max_kb'])
                       for r in stm32_rows
                       if r['node'] == node and r['heap_max_kb'] is not None]
            if max_pts:
                xs, ys = zip(*max_pts)
                ax.plot(xs, ys, linewidth=0.9, color=c, linestyle='--',
                        alpha=0.6, label=f'{node} high-water')
        ax.set_ylabel('Heap (KB)')
        ax.set_title('STM32 — heap during discovery ramp + steady-state')
        ax.legend(fontsize=7, loc='upper right')
        ax.grid(True, alpha=0.25)

    # ── Network bandwidth ─────────────────────────────────────────────────────
    if 'net_bw' in ax_map:
        ax = ax_map['net_bw']
        color_idx = 0
        for rows, lbl in [(net_rpi, 'RPi'), (net_jetson, 'Jetson')]:
            if not rows:
                continue
            xs = [r['t_epoch'] - t_min for r in rows]
            rx = [r['rx_kbps'] for r in rows]
            tx = [r['tx_kbps'] for r in rows]
            c = cmap[color_idx % len(cmap)]
            ax.plot(xs, rx, linewidth=1.2, color=c,
                    label=f'{lbl} rx')
            ax.plot(xs, tx, linewidth=1.2, color=c, linestyle='--',
                    label=f'{lbl} tx')
            color_idx += 2
        ax.set_ylabel('Bandwidth (KB/s)')
        ax.set_title('Network — rx / tx per SBC')
        ax.legend(fontsize=7, loc='upper right')
        ax.grid(True, alpha=0.25)

    # ── Per-topic bandwidth ───────────────────────────────────────────────────
    if 'topic_bw' in ax_map:
        ax = ax_map['topic_bw']
        for i, (topic, pts) in enumerate(sorted(topic_bw.items())):
            if not pts:
                continue
            xs = [t - t_min for t, _ in pts]
            ys = [bps / 1024 for _, bps in pts]   # bytes/s → KB/s
            ax.plot(xs, ys, linewidth=1.0, color=cmap[i % len(cmap)],
                    label=topic.lstrip('/'))
        ax.set_ylabel('Bandwidth (KB/s)')
        ax.set_title('Per-topic bandwidth (CDR wire size, base PC subscriber)')
        ax.legend(fontsize=6, loc='upper right', ncol=2)
        ax.grid(True, alpha=0.25)

    axes[-1].set_xlabel('Elapsed time (s) — NTP-aligned across all collectors')
    fig.suptitle('Unified Timeline — Single-Domain POC', fontsize=12)
    plt.tight_layout()
    plt.savefig(str(out_path), dpi=150, bbox_inches='tight')
    print(f'[OK] Unified timeline → {out_path}')


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
    ap.add_argument('--csv',            type=Path, nargs='+',
                    help='One or more latency CSV files to analyze. '
                         'Label is derived from filename stem (e.g. latency_rpi.csv → rpi). '
                         'Single file uses label "run" for backward-compatibility.')
    ap.add_argument('--baseline',       type=Path, help='Baseline CSV (multi-domain)')
    ap.add_argument('--poc',            type=Path, help='POC CSV (single-domain)')
    ap.add_argument('--topics',         nargs='*', help='Filter to these topic names only')
    ap.add_argument('--out-dir',        type=Path, default=Path('.'),
                    help='Directory for output files (default: .)')
    # ── Unified timeline (--merge) ────────────────────────────────────────────
    ap.add_argument('--merge',          action='store_true',
                    help='Produce NTP-aligned unified timeline from all collectors')
    ap.add_argument('--latency-rpi',    type=Path,
                    help='Merge: collect_latency.py CSV from RPi')
    ap.add_argument('--latency-jetson', type=Path,
                    help='Merge: collect_latency.py CSV from Jetson')
    ap.add_argument('--stm32',          type=Path,
                    help='Merge: collect_stm32_memory.py CSV (chassis board)')
    ap.add_argument('--stm32-sensors',  type=Path,
                    help='Merge: collect_stm32_memory.py CSV (sensors board)')
    ap.add_argument('--net-rpi',        type=Path,
                    help='Merge: collect_net_stats.py CSV from RPi')
    ap.add_argument('--net-jetson',     type=Path,
                    help='Merge: collect_net_stats.py CSV from Jetson')
    ap.add_argument('--topic-bw',       type=Path,
                    help='Merge: collect_topic_bw.py CSV from base PC (D5)')
    ap.add_argument('--topic-bw-d4',    type=Path,
                    help='Merge: collect_topic_bw.py CSV from base PC (D4)')
    ap.add_argument('--topic-bw-d6',    type=Path,
                    help='Merge: collect_topic_bw.py CSV from Jetson (D6, shared memory)')
    # ── Convenience shorthand — pass a run directory instead of individual paths
    ap.add_argument('--run-dir',         type=Path,
                    help='Path to a poc_run/run_NNN directory; auto-finds all CSVs '
                         'and implies --merge.  Individual --latency-*/--stm32/--net-*/'
                         '--topic-bw* flags are ignored when --run-dir is set.')
    args = ap.parse_args()

    if not args.csv and not args.baseline and not args.poc and not args.merge \
            and not args.run_dir:
        ap.error('Provide --csv, --baseline/--poc, --merge, or --run-dir')

    # ── --run-dir: auto-discover files and activate merge mode ────────────────
    if args.run_dir:
        rd = args.run_dir
        if not rd.is_dir():
            ap.error(f'--run-dir: {rd} is not a directory')

        def _find(name):
            p = rd / name
            return p if p.exists() else None

        args.merge          = True
        args.latency_rpi    = _find('latency_rpi.csv')
        args.latency_jetson = _find('latency_jetson.csv')
        args.stm32          = _find('stm32_chassis.csv')
        args.stm32_sensors  = _find('stm32_sensors.csv')
        args.net_rpi        = _find('net_stats_rpi.csv')
        args.net_jetson     = _find('net_stats_jetson.csv')
        args.topic_bw       = _find('topic_bw.csv')
        # Multi-domain only: D4 and D6 topic bandwidth; silently absent on single-domain.
        args.topic_bw_d4    = _find('topic_bw_d4.csv')
        args.topic_bw_d6    = _find('topic_bw_d6.csv')
        # Multi-domain only: D6 vision latency (latency_jetson_d6.csv).
        # Silently absent on single-domain runs — no error.
        latency_jetson_d6 = _find('latency_jetson_d6.csv')
        if args.out_dir == Path('.'):
            args.out_dir = rd

        found = [name for name, val in [
            ('latency_rpi.csv', args.latency_rpi),
            ('latency_jetson.csv', args.latency_jetson),
            ('latency_jetson_d6.csv', latency_jetson_d6),
            ('stm32_chassis.csv', args.stm32),
            ('stm32_sensors.csv', args.stm32_sensors),
            ('net_stats_rpi.csv', args.net_rpi),
            ('net_stats_jetson.csv', args.net_jetson),
            ('topic_bw.csv', args.topic_bw),
            ('topic_bw_d4.csv', args.topic_bw_d4),
            ('topic_bw_d6.csv', args.topic_bw_d6),
        ] if val is not None]
        print(f'[INFO] --run-dir: found {len(found)} CSV(s): {found}')

        # Feed all latency CSVs (including D6 if present) into the summary path
        latency_csvs = [p for p in [args.latency_rpi, args.latency_jetson, latency_jetson_d6]
                        if p is not None]
        if latency_csvs:
            args.csv = latency_csvs

    topics_filter = set(args.topics) if args.topics else None
    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    summary_rows = []
    raw_jitter_per_label = {}   # {label: {topic: [interval_ms, ...]}}

    # ── Single or multi-file run ──────────────────────────────────────────────
    if args.csv:
        csv_list = args.csv if isinstance(args.csv, list) else [args.csv]
        for csv_path in csv_list:
            # Derive label from stem: latency_rpi.csv→rpi, latency_jetson.csv→jetson
            # Single file keeps 'run' for backward-compatibility
            if len(csv_list) == 1:
                label = 'run'
            else:
                stem = csv_path.stem  # e.g. 'latency_rpi'
                label = stem[len('latency_'):] if stem.startswith('latency_') else stem
            lats, intvls, j, l = run_analysis(csv_path, label, topics_filter)
            raw_jitter_per_label[label] = intvls
            for topic, s in j.items():
                summary_rows.append(stats_to_row(label, topic, 'jitter', s))
            for topic, s in l.items():
                summary_rows.append(stats_to_row(label, topic, 'latency', s))

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

    # ── Unified timeline (--merge) ────────────────────────────────────────────
    if args.merge:
        lat_rpi    = (load_latency_timeseries(args.latency_rpi,    topics_filter)
                      if args.latency_rpi    else None)
        lat_jetson = (load_latency_timeseries(args.latency_jetson, topics_filter)
                      if args.latency_jetson else None)
        stm32_chassis = load_stm32_csv(args.stm32)         if args.stm32         else []
        stm32_sensors = load_stm32_csv(args.stm32_sensors) if args.stm32_sensors else []
        stm32         = (stm32_chassis + stm32_sensors) or None
        net_rpi    = load_net_csv(args.net_rpi,    'RPi')    if args.net_rpi    else None
        net_jetson = load_net_csv(args.net_jetson, 'Jetson') if args.net_jetson else None
        # Merge topic_bw from all domains (D5, D4, D6) into a single dict.
        # Topics are domain-isolated so there are no key collisions.
        topic_bw: dict | None = {}
        for _bw_path in [args.topic_bw,
                         getattr(args, 'topic_bw_d4', None),
                         getattr(args, 'topic_bw_d6', None)]:
            if _bw_path:
                _loaded = load_topic_bw_csv(_bw_path)
                if _loaded:
                    topic_bw.update(_loaded)
        topic_bw = topic_bw or None

        plot_unified_timeline(
            latency_rpi    = lat_rpi,
            latency_jetson = lat_jetson,
            stm32_rows     = stm32,
            net_rpi        = net_rpi,
            net_jetson     = net_jetson,
            topic_bw       = topic_bw,
            out_path       = out_dir / 'unified_timeline.png',
        )


if __name__ == '__main__':
    main()
