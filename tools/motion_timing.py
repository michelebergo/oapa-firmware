#!/usr/bin/env python3
"""Times an 8000-step relative jog on X over serial, polling "?" every 20 ms,
in both directions. With --baseline, fails unless the median is within 3 %.

    python tools/motion_timing.py COM5 --runs 5
    python tools/motion_timing.py COM5 --runs 5 --baseline 6.12
"""
import argparse
import re
import statistics
import sys
import time

import serial

FRAME = re.compile(r"<(\w+)\|MPos:([-\d.]+),([-\d.]+),")
STEPS = 8000
FEED = 2000
TOLERANCE = 0.03


def status(port):
    port.write(b"?\n")
    frame = port.readline().decode(errors="replace")
    port.readline()  # "ok"
    m = FRAME.search(frame)
    if not m:
        raise RuntimeError(f"unparseable status frame: {frame!r}")
    return m.group(1), float(m.group(2))


def timed_jog(port, steps):
    _, start = status(port)
    port.write(f"$J=G91G21X{steps}F{FEED}\n".encode())
    port.readline()  # "ok"
    t0 = time.perf_counter()
    while True:
        state, pos = status(port)
        elapsed = time.perf_counter() - t0
        if state == "Idle":
            return elapsed, pos - (start + steps)
        if elapsed > 60:
            raise RuntimeError("jog did not finish within 60 s")
        time.sleep(0.02)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("port")
    p.add_argument("--runs", type=int, default=5)
    p.add_argument("--baseline", type=float, help="median seconds measured on 1.2.2")
    args = p.parse_args()

    port = serial.Serial(args.port, 115200, timeout=2)
    time.sleep(2.5)  # CH340 resets the board on open
    port.reset_input_buffer()

    durations, bad_endings = [], 0
    for run in range(args.runs):
        for steps in (STEPS, -STEPS):
            seconds, error = timed_jog(port, steps)
            durations.append(seconds)
            bad_endings += error != 0
            print(f"run {run + 1} {steps:+d}: {seconds:.3f} s, end error {error:+.2f} steps")
    port.close()

    median = statistics.median(durations)
    print(f"median {median:.3f} s over {len(durations)} jogs")
    failed = bad_endings > 0
    if bad_endings:
        print(f"FAIL: {bad_endings} jogs did not end exactly on target")
    if args.baseline is not None:
        drift = (median - args.baseline) / args.baseline
        print(f"vs baseline {args.baseline:.3f} s: {drift:+.1%} (limit ±{TOLERANCE:.0%})")
        failed |= abs(drift) > TOLERANCE
    print("FAIL" if failed else "PASS")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
