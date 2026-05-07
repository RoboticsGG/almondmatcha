#!/bin/bash
# post_run.sh — Automated post-processing for a poc run directory
#
# Produces three output artefacts from the raw CSVs collected by
# launch_poc_experiment.sh:
#
#   merged_all.csv         — 1-second time-bucketed wide CSV (NTP-aligned)
#                            Columns: elapsed_s, interval_rpi__*, interval_jetson__*,
#                            latency_rpi__*, net_rpi__rx_kbps, bw__*, stm32_chassis__*
#                            Use this for cross-source time-series comparison.
#
#   merged_flat.csv        — Vertically stacked, un-bucketed raw events
#                            Each row = one original measurement event with columns:
#                            elapsed_s, source, metric, topic, value
#                            Use this for full-resolution histogram or density analysis.
#
#   latency_summary.csv    — Per-topic jitter/latency stats table
#                            Columns: label, topic, metric_type, n, mean_ms, std_ms,
#                            p50_ms, p95_ms, p99_ms, max_ms
#
#   unified_timeline.png   — Multi-panel time-series chart of all collectors
#
# Usage:
#   bash ws_base/tools/post_run.sh <run-dir>
#   bash ws_base/tools/post_run.sh ws_base/runs/single_domain/run_001
#
#   # Compare two runs (baseline vs POC) side-by-side:
#   bash ws_base/tools/post_run.sh <run-dir> --compare <other-run-dir>
#
# Requirements (base PC):
#   pip3 install pandas numpy matplotlib

set -euo pipefail

# ============================================================================
# Colours
# ============================================================================
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'
log()  { echo -e "${CYAN}[post_run]${NC} $*"; }
ok()   { echo -e "${GREEN}[post_run] OK${NC} $*"; }
warn() { echo -e "${YELLOW}[post_run] WARN${NC} $*"; }
die()  { echo -e "${RED}[post_run] ERROR${NC} $*" >&2; exit 1; }

# ============================================================================
# Argument parsing
# ============================================================================
RUN_DIR=""
COMPARE_DIR=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --compare|-c) COMPARE_DIR="$2"; shift 2 ;;
        -*) die "Unknown argument: $1" ;;
        *)  [[ -z "$RUN_DIR" ]] && RUN_DIR="$1" || die "Extra argument: $1"; shift ;;
    esac
done

[[ -z "$RUN_DIR" ]] && die "Usage: $0 <run-dir> [--compare <other-run-dir>]"
[[ -d "$RUN_DIR" ]] || die "Run directory not found: $RUN_DIR"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo ""
echo -e "\033[1m  POC Post-Run Analysis\033[0m"
echo -e "  Run: $RUN_DIR"
[[ -n "$COMPARE_DIR" ]] && echo -e "  Compare: $COMPARE_DIR"
echo ""

# ============================================================================
# Step 1 — Merge: 1-second time-bucketed CSV (NTP-aligned, wide)
# ============================================================================
log "Step 1 — Building merged_all.csv (1-second NTP-aligned buckets)..."
python3 "$SCRIPT_DIR/merge_run_csv.py" \
    --run-dir "$RUN_DIR" \
    --out     "$RUN_DIR/merged_all.csv" \
    && ok "  $RUN_DIR/merged_all.csv"   \
    || warn "  merge_run_csv.py failed — check for missing input CSVs"

# ============================================================================
# Step 2 — Flat (non-aligned) event CSV: every raw measurement event in one file
# ============================================================================
log "Step 2 — Building merged_flat.csv (un-bucketed raw events)..."
python3 - <<'PYEOF' "$RUN_DIR"
import csv, sys
from pathlib import Path
from datetime import datetime, timezone

rd = Path(sys.argv[1])

def _epoch(s: str) -> float:
    dt = datetime.fromisoformat(s)
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return dt.timestamp()

events = []

# latency / interval from collect_latency.py CSVs
for fname, src_label in [("latency_rpi.csv","rpi"), ("latency_jetson.csv","jetson"),
                          ("latency_jetson_d6.csv","jetson_d6")]:
    p = rd / fname
    if not p.exists():
        continue
    with open(p, newline="") as f:
        for row in csv.DictReader(f):
            try:
                t = float(row["recv_time_s"])
            except (KeyError, ValueError):
                continue
            topic = row.get("topic", "")
            if row.get("interval_ms"):
                try:
                    events.append((t, src_label, "interval_ms", topic, float(row["interval_ms"])))
                except ValueError:
                    pass
            if row.get("latency_ms"):
                try:
                    events.append((t, src_label, "latency_ms", topic, float(row["latency_ms"])))
                except ValueError:
                    pass

# net stats from collect_net_stats.py
for fname, src_label in [("net_stats_rpi.csv","rpi_net"),
                          ("net_stats_jetson.csv","jetson_net")]:
    p = rd / fname
    if not p.exists():
        continue
    with open(p, newline="") as f:
        for line in f:
            line = line.replace("\x00", "")
        # Re-open cleanly
    with open(p, newline="") as f:
        clean = (l.replace("\x00","") for l in f)
        for row in csv.DictReader(clean):
            if not row.get("timestamp"):
                continue
            try:
                t = _epoch(row["timestamp"])
            except (KeyError, ValueError):
                continue
            for metric in ("rx_bps","tx_bps","rx_drop","max_rx_queue"):
                if row.get(metric):
                    try:
                        events.append((t, src_label, metric, "eth0", float(row[metric])))
                    except ValueError:
                        pass

# per-topic bandwidth
for fname, src_label in [("topic_bw.csv","bw_d5"),
                          ("topic_bw_d4.csv","bw_d4"),
                          ("topic_bw_d6.csv","bw_d6")]:
    p = rd / fname
    if not p.exists():
        continue
    with open(p, newline="") as f:
        clean = (l.replace("\x00","") for l in f)
        for row in csv.DictReader(clean):
            if not row.get("timestamp"):
                continue
            try:
                t = _epoch(row["timestamp"])
                events.append((t, src_label, "bps", row.get("topic",""), float(row["bps"])))
            except (KeyError, ValueError):
                pass

# STM32 heap
for fname, src_label in [("stm32_chassis.csv","stm32_chassis"),
                          ("stm32_sensors.csv","stm32_sensors")]:
    p = rd / fname
    if not p.exists():
        continue
    with open(p, newline="") as f:
        for row in csv.DictReader(f):
            try:
                t = _epoch(row["wall_clock"])
            except (KeyError, ValueError):
                continue
            for metric in ("heap_used","heap_max","heap_free","alloc_fail"):
                if row.get(metric):
                    try:
                        events.append((t, src_label, metric, "", float(row[metric])))
                    except ValueError:
                        pass

# CPU load (RPi)
p = rd / "cpu_rpi.csv"
if p.exists():
    with open(p, newline="") as f:
        for row in csv.DictReader(f):
            try:
                t = float(row["timestamp"])
            except (KeyError, ValueError):
                continue
            for metric in ("cpu_pct","mem_pct","temp_c"):
                if row.get(metric):
                    try:
                        events.append((t, "rpi_cpu", metric, "", float(row[metric])))
                    except ValueError:
                        pass

# SoftIRQ deltas
for fname, src_label in [("softirq_rpi.csv","rpi_softirq"),
                          ("softirq_jetson.csv","jetson_softirq")]:
    p = rd / fname
    if not p.exists():
        continue
    with open(p, newline="") as f:
        for row in csv.DictReader(f):
            try:
                t = float(row["timestamp"])
            except (KeyError, ValueError):
                continue
            for metric in ("NET_RX_delta","NET_TX_delta","SCHED_delta","TIMER_delta"):
                if row.get(metric):
                    try:
                        events.append((t, src_label, metric, "", float(row[metric])))
                    except ValueError:
                        pass

if not events:
    print("[WARN] No events found — merged_flat.csv will be empty")
    sys.exit(0)

t_min = min(e[0] for e in events)
events.sort(key=lambda e: e[0])

out = rd / "merged_flat.csv"
with open(out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["elapsed_s","source","metric","topic","value"])
    for t, src, metric, topic, val in events:
        w.writerow([f"{t - t_min:.4f}", src, metric, topic, f"{val:.6g}"])

print(f"[OK] {out}  ({len(events)} events from t0={t_min:.3f})")
PYEOF
ok "  $RUN_DIR/merged_flat.csv"

# ============================================================================
# Step 3 — latency_summary.csv + unified_timeline.png via analyze_latency.py
# ============================================================================
log "Step 3 — Running analyze_latency.py (stats + unified timeline)..."
python3 "$SCRIPT_DIR/tracing/analyze_latency.py" \
    --run-dir "$RUN_DIR" \
    --out-dir "$RUN_DIR" \
    && ok "  $RUN_DIR/latency_summary.csv" \
    && ok "  $RUN_DIR/unified_timeline.png" \
    || warn "  analyze_latency.py had errors — check for missing matplotlib or input files"

# ============================================================================
# Step 4 (optional) — Side-by-side comparison with --compare run
# ============================================================================
if [[ -n "$COMPARE_DIR" ]]; then
    [[ -d "$COMPARE_DIR" ]] || die "--compare directory not found: $COMPARE_DIR"
    log "Step 4 — Side-by-side comparison: $COMPARE_DIR vs $RUN_DIR"

    # Build a list of available latency CSVs from each run (rpi + jetson)
    baseline_csvs=""
    poc_csvs=""
    for lbl in rpi jetson; do
        [[ -f "$COMPARE_DIR/latency_${lbl}.csv" ]] && baseline_csvs+=" $COMPARE_DIR/latency_${lbl}.csv"
        [[ -f "$RUN_DIR/latency_${lbl}.csv"      ]] && poc_csvs+="      $RUN_DIR/latency_${lbl}.csv"
    done

    if [[ -n "$baseline_csvs" && -n "$poc_csvs" ]]; then
        python3 "$SCRIPT_DIR/tracing/analyze_latency.py" \
            --baseline $baseline_csvs \
            --poc      $poc_csvs \
            --out-dir  "$RUN_DIR" \
            && ok "  jitter_boxplot.png written to $RUN_DIR" \
            || warn "  Comparison plot failed"
    else
        warn "  --compare: could not find latency CSVs in one or both directories"
    fi
fi

# ============================================================================
# Summary
# ============================================================================
echo ""
echo -e "\033[1m======================================================\033[0m"
echo -e "\033[0;32m  Post-Run Analysis Complete\033[0m"
echo -e "\033[1m======================================================\033[0m"
echo ""
echo "  Run directory: $RUN_DIR"
echo ""
echo "  Output files:"

for f in merged_all.csv merged_flat.csv latency_summary.csv \
          unified_timeline.png jitter_boxplot.png; do
    full="$RUN_DIR/$f"
    if [[ -f "$full" ]]; then
        size=$(du -sh "$full" 2>/dev/null | cut -f1)
        echo -e "    ${GREEN}✓${NC} $f  ($size)"
    fi
done

echo ""
echo "  Quick look:"
echo "    head -3  $RUN_DIR/merged_all.csv"
echo "    head -10 $RUN_DIR/latency_summary.csv"
echo "    open     $RUN_DIR/unified_timeline.png"
echo ""
[[ -n "$COMPARE_DIR" ]] && echo "  Compare boxplot: $RUN_DIR/jitter_boxplot.png" && echo ""
