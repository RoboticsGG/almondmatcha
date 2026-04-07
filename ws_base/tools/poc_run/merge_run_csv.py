#!/usr/bin/env python3
"""
merge_run_csv.py — Merge all POC experiment CSVs into a single time-bucketed CSV.

Reads the seven data files from a poc_run/run_NNN directory and produces one
wide CSV where every row is a 1-second time bucket:

  elapsed_s               — seconds since the earliest event in the run
  # latency (inter-arrival interval), per topic, per SBC:
  interval_rpi__<topic>__mean_ms    interval_rpi__<topic>__p95_ms
  interval_jetson__<topic>__mean_ms interval_jetson__<topic>__p95_ms
  # end-to-end latency (header.stamp → recv), topics that have it:
  latency_rpi__<topic>__mean_ms     latency_rpi__<topic>__p95_ms
  latency_jetson__<topic>__mean_ms  latency_jetson__<topic>__p95_ms
  # network throughput per SBC (KB/s):
  net_rpi__rx_kbps        net_rpi__tx_kbps
  net_jetson__rx_kbps     net_jetson__tx_kbps
  # per-topic bandwidth on base PC (KB/s):
  bw__<topic>__kbps
  # STM32 heap (KB):
  stm32_chassis__heap_used_kb   stm32_chassis__heap_free_kb
  stm32_sensors__heap_used_kb   stm32_sensors__heap_free_kb

Usage:
  python3 merge_run_csv.py --run-dir ws_base/tools/poc_run/run_001
  python3 merge_run_csv.py --run-dir ws_base/tools/poc_run/run_001 --bucket-s 0.5

Output: <run-dir>/merged_all.csv
"""

import argparse
import csv
import math
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _iso_to_epoch(s: str) -> float:
    dt = datetime.fromisoformat(s)
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return dt.timestamp()


def _pct(data: list, pct: float) -> float:
    if not data:
        return float('nan')
    s = sorted(data)
    idx = max(0, math.ceil(pct / 100 * len(s)) - 1)
    return s[idx]


def _mean(data: list) -> float:
    return sum(data) / len(data) if data else float('nan')


def _fmt(v: float) -> str:
    if math.isnan(v):
        return ''
    return f'{v:.4f}'


# ---------------------------------------------------------------------------
# Loaders
# ---------------------------------------------------------------------------

def load_latency(path: Path):
    """
    Returns:
        intervals: {topic: [(t_epoch, interval_ms), ...]}
        latencies: {topic: [(t_epoch, latency_ms), ...]}
    """
    intervals: dict = defaultdict(list)
    latencies: dict = defaultdict(list)
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            topic = row.get('topic', '')
            try:
                t = float(row['recv_time_s'])
            except (KeyError, ValueError):
                continue
            try:
                if row.get('interval_ms'):
                    intervals[topic].append((t, float(row['interval_ms'])))
            except ValueError:
                pass
            try:
                if row.get('latency_ms'):
                    latencies[topic].append((t, float(row['latency_ms'])))
            except ValueError:
                pass
    return dict(intervals), dict(latencies)


def load_net(path: Path):
    """Returns: [(t_epoch, rx_kbps, tx_kbps), ...]"""
    rows = []
    with open(path, newline='') as f:
        clean = (line.replace('\x00', '') for line in f)
        for row in csv.DictReader(clean):
            if not row.get('timestamp'):
                continue
            try:
                rows.append((
                    _iso_to_epoch(row['timestamp']),
                    float(row.get('rx_bps', 0)) / 1024,
                    float(row.get('tx_bps', 0)) / 1024,
                ))
            except (ValueError, KeyError):
                pass
    return rows


def load_topic_bw(path: Path):
    """Returns: {topic: [(t_epoch, kbps), ...]}"""
    result: dict = defaultdict(list)
    with open(path, newline='') as f:
        clean = (line.replace('\x00', '') for line in f)
        for row in csv.DictReader(clean):
            if not row.get('timestamp'):
                continue
            try:
                result[row['topic']].append((
                    _iso_to_epoch(row['timestamp']),
                    float(row['bps']) / 1024,
                ))
            except (ValueError, KeyError):
                pass
    return dict(result)


def load_stm32(path: Path):
    """Returns: [(t_epoch, heap_used_kb, heap_free_kb), ...]"""
    rows = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            try:
                t  = _iso_to_epoch(row['wall_clock'])
                used = int(row['heap_used']) // 1024 if row.get('heap_used') else None
                mx   = int(row['heap_max'])  // 1024 if row.get('heap_max')  else None
                free = (mx - used) if (used is not None and mx is not None) else None
                rows.append((t, used, free))
            except (ValueError, KeyError):
                pass
    return rows


# ---------------------------------------------------------------------------
# Bucketing
# ---------------------------------------------------------------------------

def bucket_scalar(samples: list, t_min: float, bucket_s: float):
    """Bucket (t_epoch, value) pairs into {bucket_idx: [values]}."""
    buckets: dict = defaultdict(list)
    for t, v in samples:
        b = int((t - t_min) / bucket_s)
        if b >= 0 and v is not None and not math.isnan(float(v)):
            buckets[b].append(v)
    return buckets


def bucket_pair(samples: list, t_min: float, bucket_s: float):
    """Bucket (t_epoch, v1, v2) pairs into {bucket_idx: [(v1, v2)]}."""
    buckets: dict = defaultdict(list)
    for t, v1, v2 in samples:
        b = int((t - t_min) / bucket_s)
        if b >= 0:
            buckets[b].append((v1, v2))
    return buckets


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument('--run-dir',   type=Path, required=True,
                    help='Path to a poc_run/run_NNN directory')
    ap.add_argument('--bucket-s',  type=float, default=1.0,
                    help='Time-bucket width in seconds (default: 1.0)')
    ap.add_argument('--out',       type=Path, default=None,
                    help='Output CSV path (default: <run-dir>/merged_all.csv)')
    args = ap.parse_args()

    rd = args.run_dir
    if not rd.is_dir():
        ap.error(f'{rd} is not a directory')

    out_path = args.out or (rd / 'merged_all.csv')
    B = args.bucket_s

    # ── Load all present files ────────────────────────────────────────────────
    print(f'[INFO] Loading data from {rd}')

    intervals_rpi,  latencies_rpi    = ({}, {})
    intervals_jet,  latencies_jet    = ({}, {})
    net_rpi_rows,   net_jet_rows     = [], []
    bw_rows:  dict                   = {}
    stm32_ch, stm32_se               = [], []

    if (rd / 'latency_rpi.csv').exists():
        intervals_rpi, latencies_rpi = load_latency(rd / 'latency_rpi.csv')
        print(f'  latency_rpi.csv     — {sum(len(v) for v in intervals_rpi.values())} rows')
    if (rd / 'latency_jetson.csv').exists():
        intervals_jet, latencies_jet = load_latency(rd / 'latency_jetson.csv')
        print(f'  latency_jetson.csv  — {sum(len(v) for v in intervals_jet.values())} rows')
    if (rd / 'net_stats_rpi.csv').exists():
        net_rpi_rows = load_net(rd / 'net_stats_rpi.csv')
        print(f'  net_stats_rpi.csv   — {len(net_rpi_rows)} rows')
    if (rd / 'net_stats_jetson.csv').exists():
        net_jet_rows = load_net(rd / 'net_stats_jetson.csv')
        print(f'  net_stats_jetson.csv— {len(net_jet_rows)} rows')
    if (rd / 'topic_bw.csv').exists():
        bw_rows = load_topic_bw(rd / 'topic_bw.csv')
        print(f'  topic_bw.csv        — {sum(len(v) for v in bw_rows.values())} rows')
    if (rd / 'stm32_chassis.csv').exists():
        stm32_ch = load_stm32(rd / 'stm32_chassis.csv')
        print(f'  stm32_chassis.csv   — {len(stm32_ch)} rows')
    if (rd / 'stm32_sensors.csv').exists():
        stm32_se = load_stm32(rd / 'stm32_sensors.csv')
        print(f'  stm32_sensors.csv   — {len(stm32_se)} rows')

    # ── Determine global t_min ─────────────────────────────────────────────────
    all_t = []
    for src in (intervals_rpi, intervals_jet, latencies_rpi, latencies_jet):
        for pts in src.values():
            all_t.extend(t for t, _ in pts)
    for src in (net_rpi_rows, net_jet_rows):
        all_t.extend(t for t, _, _ in src)
    for pts in bw_rows.values():
        all_t.extend(t for t, _ in pts)
    for t, _, _ in (stm32_ch + stm32_se):
        all_t.append(t)

    if not all_t:
        print('[WARN] No data found in any CSV — nothing to merge.')
        return

    t_min   = min(all_t)
    t_max   = max(all_t)
    n_buckets = int((t_max - t_min) / B) + 1
    print(f'[INFO] Time range: {t_max - t_min:.1f}s  |  {n_buckets} buckets at {B}s each')

    # ── Bucket every time series ───────────────────────────────────────────────
    # latency topics
    all_lat_topics = sorted(set(intervals_rpi) | set(intervals_jet)
                            | set(latencies_rpi) | set(latencies_jet))
    all_bw_topics  = sorted(bw_rows)

    b_intvl_rpi = {tp: bucket_scalar(intervals_rpi.get(tp, []), t_min, B) for tp in all_lat_topics}
    b_intvl_jet = {tp: bucket_scalar(intervals_jet.get(tp, []), t_min, B) for tp in all_lat_topics}
    b_lat_rpi   = {tp: bucket_scalar(latencies_rpi.get(tp, []), t_min, B) for tp in all_lat_topics}
    b_lat_jet   = {tp: bucket_scalar(latencies_jet.get(tp, []), t_min, B) for tp in all_lat_topics}

    b_net_rpi   = bucket_pair(net_rpi_rows,  t_min, B)
    b_net_jet   = bucket_pair(net_jet_rows,  t_min, B)
    b_bw        = {tp: bucket_scalar(bw_rows.get(tp, []), t_min, B) for tp in all_bw_topics}

    b_stm32_ch  = bucket_pair(stm32_ch, t_min, B)   # (used_kb, free_kb)
    b_stm32_se  = bucket_pair(stm32_se, t_min, B)

    # ── Build column names ─────────────────────────────────────────────────────
    # Sanitise topic names: strip leading '/', replace '/' with '__'
    def _col(prefix: str, topic: str, suffix: str) -> str:
        clean = topic.lstrip('/').replace('/', '__')
        return f'{prefix}__{clean}__{suffix}'

    columns = ['elapsed_s']

    for tp in all_lat_topics:
        for pfx in ('interval_rpi', 'interval_jetson'):
            columns += [_col(pfx, tp, 'mean_ms'), _col(pfx, tp, 'p95_ms')]
    for tp in all_lat_topics:
        for pfx in ('latency_rpi', 'latency_jetson'):
            columns += [_col(pfx, tp, 'mean_ms'), _col(pfx, tp, 'p95_ms')]

    columns += ['net_rpi__rx_kbps', 'net_rpi__tx_kbps',
                'net_jetson__rx_kbps', 'net_jetson__tx_kbps']

    for tp in all_bw_topics:
        columns.append(_col('bw', tp, 'kbps'))

    columns += ['stm32_chassis__heap_used_kb', 'stm32_chassis__heap_free_kb',
                'stm32_sensors__heap_used_kb', 'stm32_sensors__heap_free_kb']

    # ── Write CSV ──────────────────────────────────────────────────────────────
    written = 0
    with open(out_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(columns)

        for b in range(n_buckets):
            elapsed = round(b * B, 6)
            row_vals = [f'{elapsed:.3f}']

            # interval per topic
            for tp in all_lat_topics:
                d = b_intvl_rpi[tp].get(b, [])
                row_vals += [_fmt(_mean(d)), _fmt(_pct(d, 95))]
            for tp in all_lat_topics:
                d = b_intvl_jet[tp].get(b, [])
                row_vals += [_fmt(_mean(d)), _fmt(_pct(d, 95))]

            # latency per topic
            for tp in all_lat_topics:
                d = b_lat_rpi[tp].get(b, [])
                row_vals += [_fmt(_mean(d)), _fmt(_pct(d, 95))]
            for tp in all_lat_topics:
                d = b_lat_jet[tp].get(b, [])
                row_vals += [_fmt(_mean(d)), _fmt(_pct(d, 95))]

            # net stats — mean rx/tx for the bucket
            nr = b_net_rpi.get(b, [])
            row_vals += [
                _fmt(_mean([x[0] for x in nr])),
                _fmt(_mean([x[1] for x in nr])),
            ]
            nj = b_net_jet.get(b, [])
            row_vals += [
                _fmt(_mean([x[0] for x in nj])),
                _fmt(_mean([x[1] for x in nj])),
            ]

            # topic bw
            for tp in all_bw_topics:
                d = b_bw[tp].get(b, [])
                row_vals.append(_fmt(_mean(d)))

            # stm32
            sc = b_stm32_ch.get(b, [])
            row_vals += [
                _fmt(_mean([x[0] for x in sc if x[0] is not None])),
                _fmt(_mean([x[1] for x in sc if x[1] is not None])),
            ]
            ss = b_stm32_se.get(b, [])
            row_vals += [
                _fmt(_mean([x[0] for x in ss if x[0] is not None])),
                _fmt(_mean([x[1] for x in ss if x[1] is not None])),
            ]

            w.writerow(row_vals)
            written += 1

    print(f'[OK]  merged_all.csv → {out_path}')
    print(f'      {written} rows  |  {len(columns)} columns')


if __name__ == '__main__':
    main()
