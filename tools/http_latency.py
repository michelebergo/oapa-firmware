#!/usr/bin/env python3
"""Separates the delay the OAPA firmware adds from the delay of the network.

For each sample, back to back: ICMP ping to the board (answered by lwIP, no
web server involved), GET /api/status on the board, ICMP ping to a control
device on the same network (e.g. the ASIAIR). Home mesh networks produce
multi-hundred-ms outliers for every device, so absolute limits measured from
a PC say nothing about the firmware.

Pass criteria:
- every HTTP request answered, and the board did not reboot;
- firmware overhead: median HTTP <= median board ping + 50 ms;
- the board has no more ping outliers (> 150 ms) than the control device,
  plus 5 - the sampling noise seen on 120 samples (two devices on the same
  mesh measured 8 and 8 in one window).

    python tools/http_latency.py 192.168.1.15 --control 192.168.1.53
"""
import argparse
import json
import re
import statistics
import subprocess
import sys
import time
import urllib.request

OVERHEAD_LIMIT_MS = 50
OUTLIER_MS = 150
OUTLIER_NOISE = 5
PING_TIME = re.compile(r"time[=<](\d+)ms")


def ping_ms(host):
    out = subprocess.run(["ping", "-n", "1", "-w", "3000", host], capture_output=True, text=True).stdout
    m = PING_TIME.search(out)
    return float(m.group(1)) if m else None


def http_ms(host):
    t0 = time.perf_counter()
    status = json.loads(urllib.request.urlopen(f"http://{host}/api/status", timeout=5).read())
    return (time.perf_counter() - t0) * 1000, status["boots"]


def summary(name, values):
    ordered = sorted(values)
    p90 = ordered[max(0, int(len(ordered) * 0.9) - 1)]
    outliers = sum(v > OUTLIER_MS for v in values)
    print(f"{name:14s} n={len(values):3d} median={statistics.median(ordered):5.0f} ms "
          f"p90={p90:5.0f} ms max={ordered[-1]:5.0f} ms  >{OUTLIER_MS}ms={outliers}")
    return statistics.median(ordered), outliers


def main():
    p = argparse.ArgumentParser()
    p.add_argument("host")
    p.add_argument("--control", required=True, help="another device on the same network")
    p.add_argument("--count", type=int, default=120)
    p.add_argument("--interval", type=float, default=0.3)
    args = p.parse_args()

    board_ping, board_http, control_ping = [], [], []
    http_failures, lost_pings, boots = 0, 0, set()
    for i in range(args.count):
        bp = ping_ms(args.host)
        try:
            ms, boot = http_ms(args.host)
            board_http.append(ms)
            boots.add(boot)
        except Exception as error:
            http_failures += 1
            print(f"#{i:03d} HTTP failed: {error}")
        cp = ping_ms(args.control)
        for value, bucket in ((bp, board_ping), (cp, control_ping)):
            if value is None:
                lost_pings += 1
            else:
                bucket.append(value)
        time.sleep(args.interval)

    if not board_http or not board_ping or not control_ping:
        print("FAIL: not enough answers to compare")
        return 1
    ping_median, board_outliers = summary("board ping", board_ping)
    http_median, _ = summary("board HTTP", board_http)
    _, control_outliers = summary("control ping", control_ping)
    overhead = http_median - ping_median
    print(f"HTTP failures={http_failures} lost pings={lost_pings} boots={sorted(boots)} overhead={overhead:.0f} ms")

    checks = [
        (http_failures == 0 and len(boots) == 1, "every HTTP request answered, no reboot"),
        (overhead <= OVERHEAD_LIMIT_MS, f"firmware overhead {overhead:.0f} ms <= {OVERHEAD_LIMIT_MS} ms"),
        (board_outliers <= control_outliers + OUTLIER_NOISE,
         f"board ping outliers {board_outliers} <= control {control_outliers} + {OUTLIER_NOISE}"),
    ]
    for passed, text in checks:
        print(("PASS " if passed else "FAIL ") + text)
    return 0 if all(passed for passed, _ in checks) else 1


if __name__ == "__main__":
    sys.exit(main())
