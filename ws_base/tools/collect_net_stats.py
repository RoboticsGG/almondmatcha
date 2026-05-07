#!/usr/bin/env python3
"""
collect_net_stats.py — Continuously log socket-buffer and interface-bandwidth
                       statistics to CSV on a Linux SBC (RPi or Jetson).

Metrics captured every INTERVAL_S seconds:
  - /proc/net/udp    → per-socket rx_queue, tx_queue (socket buffer fill levels)
  - /proc/net/dev    → cumulative bytes/packets/errors per interface
  - 'ss -u -a -n'    → number of UDP sockets + per-socket queues (summary)

Run on the SBC (RPi / Jetson) during the POC experiment:
    python3 collect_net_stats.py --out /tmp/net_stats_rpi.csv

Then pull back to the base PC:
    scp pi@<rpi_ip>:/tmp/net_stats_rpi.csv ws_base/tools/monitoring/data/

Usage:
    python3 collect_net_stats.py [--iface IFACE] [--interval SECS] [--out FILE]
                                 [--duration SECS]

Arguments:
    --iface     Network interface to monitor.  Use 'auto' (default) to detect
                the interface whose IP is on the DDS LAN (192.168.1.0/24).
                Override with a literal name (e.g. eth0, enp3s0) if needed.
    --interval  Sampling interval in seconds (default: 0.5)
    --out       Output CSV file path (default: /tmp/net_stats_<hostname>.csv)
    --duration  How long to run in seconds; 0 = run until Ctrl-C (default: 0)
"""

import argparse
import csv
import os
import re
import socket
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


def _detect_iface(subnet: str = "192.168.1.") -> str:
    """
    Return the interface whose IPv4 address is on the DDS LAN subnet.
    Strategy:
      1. 'ip -4 addr' → find an interface with an IP starting with *subnet*
      2. Fallback: first non-loopback, non-virtual interface in /proc/net/dev
    """
    try:
        out = subprocess.check_output(
            ["ip", "-4", "addr"], text=True, timeout=3
        )
        # Lines alternate between interface headers ('2: enp3s0: ...') and
        # address lines ('    inet 192.168.1.5/24 ...').
        current_iface = None
        for line in out.splitlines():
            m_iface = re.match(r"^\d+:\s+(\S+):", line)
            if m_iface:
                current_iface = m_iface.group(1).rstrip(":@")
                continue
            m_addr = re.search(r"inet\s+(" + re.escape(subnet) + r"\d+)", line)
            if m_addr and current_iface:
                return current_iface
    except (subprocess.TimeoutExpired, FileNotFoundError,
            subprocess.CalledProcessError):
        pass

    # Fallback: first non-loopback interface in /proc/net/dev
    try:
        with open("/proc/net/dev") as f:
            for line in f:
                name = line.split(":")[0].strip()
                if name and name != "lo" and not name.startswith("docker"):
                    return name
    except FileNotFoundError:
        pass

    return "eth0"  # last-resort historic default


def read_proc_net_dev(iface: str) -> dict:
    """Read /proc/net/dev and return stats for the given interface."""
    result = {}
    try:
        with open("/proc/net/dev") as f:
            for line in f:
                if iface not in line:
                    continue
                parts = line.split()
                # Format: iface: rx_bytes rx_packets rx_errs rx_drop ... tx_bytes tx_packets ...
                if len(parts) >= 17:
                    result["rx_bytes"]   = int(parts[1])
                    result["rx_packets"] = int(parts[2])
                    result["rx_errs"]    = int(parts[3])
                    result["rx_drop"]    = int(parts[4])
                    result["tx_bytes"]   = int(parts[9])
                    result["tx_packets"] = int(parts[10])
                    result["tx_errs"]    = int(parts[11])
                    result["tx_drop"]    = int(parts[12])
    except FileNotFoundError:
        pass
    return result


def read_udp_socket_stats() -> dict:
    """
    Read /proc/net/udp and return aggregate queue stats.
    Returns: {total_sockets, max_rx_queue_bytes, max_tx_queue_bytes, total_rx_queue, total_tx_queue}
    """
    stats = dict(udp_sockets=0, max_rx_queue=0, max_tx_queue=0,
                 total_rx_queue=0, total_tx_queue=0)
    try:
        with open("/proc/net/udp") as f:
            next(f)  # skip header
            for line in f:
                parts = line.split()
                if len(parts) < 8:
                    continue
                # Column 4 is "tx_queue:rx_queue" in hex
                queues = parts[4].split(":")
                if len(queues) == 2:
                    tx_q = int(queues[0], 16)
                    rx_q = int(queues[1], 16)
                    stats["udp_sockets"]    += 1
                    stats["total_tx_queue"] += tx_q
                    stats["total_rx_queue"] += rx_q
                    stats["max_tx_queue"]    = max(stats["max_tx_queue"], tx_q)
                    stats["max_rx_queue"]    = max(stats["max_rx_queue"], rx_q)
    except FileNotFoundError:
        pass
    return stats


def read_ss_summary() -> dict:
    """Run 'ss -s' and parse the UDP line for a quick overview."""
    result = {"ss_udp_total": 0}
    try:
        out = subprocess.check_output(["ss", "-s"], text=True, timeout=2)
        for line in out.splitlines():
            if line.startswith("UDP:"):
                m = re.search(r"(\d+)", line)
                if m:
                    result["ss_udp_total"] = int(m.group(1))
    except (subprocess.TimeoutExpired, FileNotFoundError, subprocess.CalledProcessError):
        pass
    return result


# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--iface",    default="auto",
                        help="Interface name, or 'auto' to detect from default route (default: auto)")
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--out",      type=Path,
                        default=Path(f"/tmp/net_stats_{socket.gethostname()}.csv"))
    parser.add_argument("--duration", type=float, default=0,
                        help="seconds to run (0 = infinite)")
    args = parser.parse_args()

    if args.iface == "auto":
        args.iface = _detect_iface()
        print(f"[INFO] Auto-detected DDS interface: {args.iface}")

    print(f"[INFO] Logging to: {args.out}")
    print(f"[INFO] Interface:  {args.iface}  |  Interval: {args.interval}s")
    print("[INFO] Press Ctrl-C to stop.")

    fields = [
        "timestamp", "elapsed_s",
        # interface counters (deltas since previous sample)
        "rx_bytes_delta", "tx_bytes_delta",
        "rx_packets_delta", "tx_packets_delta",
        "rx_errs", "tx_errs", "rx_drop", "tx_drop",
        # bandwidth estimate (bytes/s)
        "rx_bps", "tx_bps",
        # UDP socket buffer stats
        "udp_sockets", "total_rx_queue", "total_tx_queue",
        "max_rx_queue", "max_tx_queue",
        # ss summary
        "ss_udp_total",
    ]

    prev_dev  = {}
    prev_time = None
    start_t   = time.monotonic()

    with open(args.out, "w", newline="") as fout:
        writer = csv.DictWriter(fout, fieldnames=fields)
        writer.writeheader()
        fout.flush()

        try:
            while True:
                now  = time.monotonic()
                elapsed = now - start_t

                dev   = read_proc_net_dev(args.iface)
                udp   = read_udp_socket_stats()
                ss    = read_ss_summary()
                dt    = (now - prev_time) if prev_time else args.interval

                rx_delta = dev.get("rx_bytes",   0) - prev_dev.get("rx_bytes",   0)
                tx_delta = dev.get("tx_bytes",   0) - prev_dev.get("tx_bytes",   0)
                rp_delta = dev.get("rx_packets", 0) - prev_dev.get("rx_packets", 0)
                tp_delta = dev.get("tx_packets", 0) - prev_dev.get("tx_packets", 0)

                row = {
                    "timestamp":        datetime.utcnow().isoformat(),
                    "elapsed_s":        round(elapsed, 3),
                    "rx_bytes_delta":   rx_delta,
                    "tx_bytes_delta":   tx_delta,
                    "rx_packets_delta": rp_delta,
                    "tx_packets_delta": tp_delta,
                    "rx_errs":          dev.get("rx_errs", 0),
                    "tx_errs":          dev.get("tx_errs", 0),
                    "rx_drop":          dev.get("rx_drop", 0),
                    "tx_drop":          dev.get("tx_drop", 0),
                    "rx_bps":           round(rx_delta / dt) if prev_time else 0,
                    "tx_bps":           round(tx_delta / dt) if prev_time else 0,
                    "udp_sockets":      udp["udp_sockets"],
                    "total_rx_queue":   udp["total_rx_queue"],
                    "total_tx_queue":   udp["total_tx_queue"],
                    "max_rx_queue":     udp["max_rx_queue"],
                    "max_tx_queue":     udp["max_tx_queue"],
                    "ss_udp_total":     ss["ss_udp_total"],
                }
                writer.writerow(row)
                fout.flush()

                # Print brief live status
                print(f"\r[{row['timestamp']}] "
                      f"rx={row['rx_bps']//1024:5d} kBps  "
                      f"tx={row['tx_bps']//1024:5d} kBps  "
                      f"udp_sockets={row['udp_sockets']}  "
                      f"max_rx_q={row['max_rx_queue']} B    ",
                      end="", flush=True)

                prev_dev  = dev
                prev_time = now

                if args.duration > 0 and elapsed >= args.duration:
                    break

                time.sleep(args.interval)

        except KeyboardInterrupt:
            pass

    print(f"\n[OK] Done. File: {args.out}")


if __name__ == "__main__":
    main()
