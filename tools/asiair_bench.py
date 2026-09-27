#!/usr/bin/env python3
"""Drives the board against tools/asiair_emu.py: calibration, then automatic
alignment, each checked against the emulator's truth. Real motors move.

    python tools/asiair_bench.py 192.168.1.15 --emulator 192.168.1.54 --factor 60 --mode full
    python tools/asiair_bench.py 192.168.1.15 --emulator 192.168.1.54 --mode source-end
"""
import argparse
import json
import sys
import time
import urllib.error
import urllib.parse
import urllib.request


def get(host, path):
    try:
        with urllib.request.urlopen(f"http://{host}{path}", timeout=8) as response:
            return json.loads(response.read())
    except (urllib.error.URLError, OSError, ValueError):
        return None


def post(host, path, **fields):
    data = urllib.parse.urlencode(fields).encode()
    request = urllib.request.Request(f"http://{host}{path}", data=data, method="POST")
    try:
        with urllib.request.urlopen(request, timeout=8) as response:
            return response.status, response.read().decode()
    except urllib.error.HTTPError as error:
        return error.code, error.read().decode()


def wait(host, path, done, timeout_s, label):
    deadline = time.time() + timeout_s
    last = None
    while time.time() < deadline:
        body = get(host, path)
        if body is not None:
            last = body
            if done(body):
                return body
        time.sleep(2)
    print(f"TIMEOUT waiting for {label}: {last}")
    sys.exit(1)


def check(ok, text):
    print(("PASS " if ok else "FAIL ") + text)
    return 0 if ok else 1


def main():
    p = argparse.ArgumentParser()
    p.add_argument("board")
    p.add_argument("--emulator", required=True, help="IP of the machine running asiair_emu.py")
    p.add_argument("--factor", type=float, default=60)
    p.add_argument("--mode", choices=["full", "source-end"], default="full")
    a = p.parse_args()
    failures = 0

    # Full mode starts from factors 1 so calibration has to find them; source-end needs usable factors.
    start_factor = 1 if a.mode == "full" else a.factor
    print("settings:", post(a.board, "/api/settings", asiairHost=a.emulator, factorX=start_factor,
                            factorY=start_factor, calThresholdArcmin=2))
    wait(a.board, "/api/loop", lambda l: l["sourceReady"], 120, "ASIAIR source ready")

    if a.mode == "source-end":
        print("loop start:", post(a.board, "/api/loop/start", go=1))
        result = wait(a.board, "/api/loop", lambda l: l["phase"] == "ended", 900, "run end")
        failures += check(result["outcome"] == "stopped_source_ended",
                          f"run ends when the PA ends: {result['outcome']} ({result['reason']})")
        return failures

    print("calibrate:", post(a.board, "/api/calibrate/start", go=1))
    cal = wait(a.board, "/api/calibrate", lambda c: c["state"] in ("done", "failed", "stopped"), 900, "calibration")
    print("calibration:", cal)
    for axis in ("x", "y"):
        failures += check(cal["state"] == "done" and abs(cal[axis]["factor"] - a.factor) <= 0.10 * a.factor,
                          f"{axis} factor {cal[axis]['factor']:.2f} vs {a.factor} (10 %), sign {cal[axis]['sign']}")
    failures += check(cal["offset"]["x"] == 0 and cal["offset"]["y"] == 0, f"axes back at the start: {cal['offset']}")
    if failures:
        return failures

    print("save:", post(a.board, "/api/calibrate/save", go=1))
    print("loop start:", post(a.board, "/api/loop/start", go=1))
    result = wait(a.board, "/api/loop", lambda l: l["phase"] == "ended", 1800, "alignment")
    failures += check(result["outcome"] == "finished",
                      f"alignment: {result['outcome']} ({result['reason']}), {result['moves']} moves, "
                      f"final az {result['az']:+.2f}' alt {result['alt']:+.2f}'")
    print("ALL PASS" if failures == 0 else f"{failures} FAILED")
    return failures


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
