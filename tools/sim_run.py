#!/usr/bin/env python3
"""Runs one simulated alignment on the board over HTTP and reports the outcome.
The real motors move; the error they "correct" is simulated on the board.

    python tools/sim_run.py 192.168.1.15 --expect finished --max-pass-us 200
"""
import argparse
import json
import sys
import time
import urllib.error
import urllib.parse
import urllib.request


def post(host, path, **fields):
    data = urllib.parse.urlencode(fields).encode()
    request = urllib.request.Request(f"http://{host}{path}", data=data, method="POST")
    try:
        with urllib.request.urlopen(request, timeout=8) as response:
            return response.status, response.read().decode()
    except urllib.error.HTTPError as error:
        return error.code, error.read().decode()


def loop_status(host):
    with urllib.request.urlopen(f"http://{host}/api/loop", timeout=8) as response:
        return json.loads(response.read())


def main():
    p = argparse.ArgumentParser()
    p.add_argument("host")
    p.add_argument("--expect", nargs="+", default=["finished"])
    p.add_argument("--max-pass-us", type=int, default=None,
                   help="optional limit on the longest loop pass; by default the value is only reported")
    p.add_argument("--factor", type=float, default=60)
    p.add_argument("--az", type=float, default=30)
    p.add_argument("--alt", type=float, default=-20)
    p.add_argument("--noise", type=float, default=0.05)
    p.add_argument("--refresh-ms", type=int, default=4000)
    p.add_argument("--timeout-s", type=int, default=1800)
    args = p.parse_args()

    code, body = post(args.host, "/api/sim", enable=1, factor=args.factor, az=args.az, alt=args.alt,
                      noise=args.noise, refresh_ms=args.refresh_ms)
    print("sim:", code, body)
    if code != 202:
        return 1
    time.sleep(1.0)
    code, body = post(args.host, "/api/loop/start")
    print("start:", code, body)
    if code != 202:
        return 1

    started, last_moves, status = time.time(), -1, {}
    while time.time() - started < args.timeout_s:
        try:
            status = loop_status(args.host)
        except Exception as error:  # home mesh hiccups: keep polling
            print("status error:", error)
            time.sleep(2)
            continue
        if status["moves"] != last_moves:
            last_moves = status["moves"]
            plan = status["plan"]
            print(f"{time.time() - started:6.0f}s {status['phase']:9s} moves={status['moves']:3d} "
                  f"total={status['total']:7.2f}' plan=({plan['x']:.2f},{plan['y']:.2f}) {status['reason']}")
        if status["phase"] == "ended":
            break
        time.sleep(1.0)

    post(args.host, "/api/sim", enable=0)
    print("final:", json.dumps(status))
    ok = status.get("phase") == "ended" and status.get("outcome") in args.expect
    if args.max_pass_us is not None:
        ok = ok and status.get("passMaxUs", 10**9) < args.max_pass_us
    print("PASS" if ok else "FAIL",
          f"outcome={status.get('outcome')} expected={args.expect} passMaxUs={status.get('passMaxUs')}"
          + (f" limit<{args.max_pass_us}" if args.max_pass_us is not None else " (reported only)"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
