#!/usr/bin/env python3
"""
collect_stm32_memory.py — Read USB serial ports of both STM32 Nucleo boards,
                           filter for memory-reporter JSON lines, and save to CSV.

The firmware's memory_reporter.h emits lines of the form:
    {"type":"STM32_MEM","node":"chassis","ts_ms":12345,"heap_used":...,"alloc_fail":...}

Every other line from the STM32 (MROS2_INFO, sensor printf) is discarded.
No firmware change to output suppression is required.

Usage:
    # Auto-detect both /dev/ttyACM* ports and collect:
    python3 collect_stm32_memory.py --out /tmp/stm32_memory_poc.csv

    # Explicit ports:
    python3 collect_stm32_memory.py \
        --chassis /dev/ttyACM0 --sensors /dev/ttyACM1 \
        --out /tmp/stm32_memory_poc.csv

    # Only one board connected:
    python3 collect_stm32_memory.py --chassis /dev/ttyACM0 --sensors none

Requirements:
    pip install pyserial
"""

import argparse
import csv
import glob
import json
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
except ImportError:
    sys.exit("[ERROR] pyserial not found. Install with: pip install pyserial")

BAUD_RATE     = 115200
POLL_TIMEOUT  = 0.1      # serial read timeout in seconds
MEM_TYPE_KEY  = "STM32_MEM"

CSV_FIELDS = [
    "wall_clock",
    "type",
    "node",
    "ts_ms",
    "heap_used",
    "heap_max",
    "heap_free",
    "alloc_fail",
    "stack_free",
]


def auto_detect_ports():
    """Return up to two /dev/ttyACM* ports sorted by device name."""
    found = sorted(glob.glob("/dev/ttyACM*"))
    return found[:2] if found else []


def read_serial_thread(port: str, label: str, rows: list, lock: threading.Lock,
                       stop_event: threading.Event):
    """Background thread: read serial lines, filter JSON, append to shared rows list."""
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=POLL_TIMEOUT)
    except serial.SerialException as e:
        print(f"[{label}] ERROR opening {port}: {e}")
        return

    print(f"[{label}] Listening on {port} at {BAUD_RATE} baud")
    buf = b""

    while not stop_event.is_set():
        chunk = ser.read(256)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line_bytes, buf = buf.split(b"\n", 1)
            line = line_bytes.decode("utf-8", errors="replace").strip()
            if f'"type":"{MEM_TYPE_KEY}"' not in line:
                continue  # silently drop all non-memory lines
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                continue
            row = {
                "wall_clock": datetime.utcnow().isoformat(),
                "type":       data.get("type", ""),
                "node":       data.get("node", label),
                "ts_ms":      data.get("ts_ms", ""),
                "heap_used":  data.get("heap_used", ""),
                "heap_max":   data.get("heap_max", ""),
                "heap_free":  data.get("heap_free", ""),
                "alloc_fail": data.get("alloc_fail", ""),
                "stack_free": data.get("stack_free", ""),
            }
            with lock:
                rows.append(row)

            # Print a brief live summary
            fail = data.get("alloc_fail", 0)
            used = data.get("heap_used", 0)
            free = data.get("heap_free", 0)
            flag = "  *** ALLOC FAIL ***" if fail else ""
            print(f"  [{label}] ts={data.get('ts_ms'):6}ms  "
                  f"used={used//1024:4d}KB  free={free//1024:4d}KB{flag}")

    ser.close()


def writer_thread(out_path: Path, rows: list, lock: threading.Lock,
                  stop_event: threading.Event):
    """Flush buffered rows to CSV every second."""
    with open(out_path, "w", newline="") as fout:
        writer = csv.DictWriter(fout, fieldnames=CSV_FIELDS)
        writer.writeheader()
        fout.flush()
        written = 0

        while not stop_event.is_set():
            time.sleep(1.0)
            with lock:
                new_rows = rows[written:]
                written += len(new_rows)
            for row in new_rows:
                writer.writerow(row)
            if new_rows:
                fout.flush()

        # Final flush on exit
        with lock:
            for row in rows[written:]:
                writer.writerow(row)
        fout.flush()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--chassis", default=None,
                        help="Serial port for chassis STM32 (e.g. /dev/ttyACM0), or 'none'")
    parser.add_argument("--sensors", default=None,
                        help="Serial port for sensors STM32 (e.g. /dev/ttyACM1), or 'none'")
    parser.add_argument("--out", type=Path,
                        default=Path("/tmp/stm32_memory_poc.csv"))
    parser.add_argument("--duration", type=float, default=0,
                        help="Seconds to collect (0 = run until Ctrl-C)")
    args = parser.parse_args()

    # Auto-detect if not specified
    if args.chassis is None and args.sensors is None:
        detected = auto_detect_ports()
        if not detected:
            sys.exit("[ERROR] No /dev/ttyACM* ports found. Connect STM32 boards and retry.")
        print(f"[INFO] Auto-detected ports: {detected}")
        args.chassis = detected[0] if len(detected) > 0 else "none"
        args.sensors = detected[1] if len(detected) > 1 else "none"

    ports = {}
    if args.chassis and args.chassis.lower() != "none":
        ports["chassis"] = args.chassis
    if args.sensors and args.sensors.lower() != "none":
        ports["sensors"] = args.sensors

    if not ports:
        sys.exit("[ERROR] No valid ports specified.")

    print(f"[INFO] Output: {args.out}")
    print("[INFO] Press Ctrl-C to stop.\n")

    shared_rows = []
    lock        = threading.Lock()
    stop_event  = threading.Event()

    threads = []
    for label, port in ports.items():
        t = threading.Thread(target=read_serial_thread,
                             args=(port, label, shared_rows, lock, stop_event),
                             daemon=True)
        t.start()
        threads.append(t)

    wt = threading.Thread(target=writer_thread,
                          args=(args.out, shared_rows, lock, stop_event),
                          daemon=True)
    wt.start()
    threads.append(wt)

    try:
        start = time.monotonic()
        while True:
            if args.duration > 0 and (time.monotonic() - start) >= args.duration:
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    print("\n[INFO] Stopping...")
    stop_event.set()
    time.sleep(2.0)  # give threads a moment to flush

    with lock:
        total = len(shared_rows)
    print(f"[OK] Collected {total} memory samples — saved to {args.out}")


if __name__ == "__main__":
    main()
